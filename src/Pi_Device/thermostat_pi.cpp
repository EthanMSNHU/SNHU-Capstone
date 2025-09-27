/*
  Raspberry Pi Thermostat (REAL DHT22 + Buttons + Relays + SQLite)
  ----------------------------------------------------------------
  Build (on the Pi):
    g++ -std=c++17 thermostat_pi.cpp -DUSE_SQLITE -DUSE_RPI -lpigpio -pthread -lsqlite3 -O2 -o thermostat
  Run:
    sudo ./thermostat
*/

#include <algorithm>   // std::transform, std::clamp
#include <chrono>      // timing utilities
#include <cstdint>     // fixed-size ints
#include <cstdlib>     // std::rand
#include <ctime>       // std::time
#include <iomanip>     // std::put_time
#include <iostream>    // std::cout, std::cerr
#include <optional>    // std::optional
#include <sstream>     // std::istringstream
#include <stdexcept>   // std::runtime_error
#include <string>      // std::string
#include <thread>      // std::thread, sleep
#include <vector>      // (not strictly required)

#include <sqlite3.h>   // SQLite C API
#include <pigpio.h>    // pigpio GPIO + timing API

// ========= CONFIGURE WIRING (BCM numbering) =========
static const unsigned PIN_DHT       = 4;    // DHT22 data pin
static const unsigned PIN_HEAT      = 17;   // Heat relay pin
static const unsigned PIN_COOL      = 27;   // Cool relay pin
static const bool     RELAY_ACTIVE_LOW = true; // true if relay turns ON at logic LOW

// Buttons (active-LOW; we enable internal pull-ups)
static const unsigned PIN_BTN_HEAT  = 5;    // set System=Heat
static const unsigned PIN_BTN_COOL  = 6;    // set System=Cool
static const unsigned PIN_BTN_OFF   = 13;   // set System=Off
static const unsigned PIN_BTN_AUTO  = 19;   // set Control=Auto
static const unsigned PIN_BTN_ON    = 26;   // set Control=On

// Debounce / glitch filter in milliseconds
static const uint32_t DEBOUNCE_MS   = 80;   // 80ms debounce for noisy buttons

// ========= UTIL =========
namespace util {
  // Returns local time in ISO-8601 (YYYY-MM-DDTHH:MM:SS)
  inline std::string now_iso8601() {
    using namespace std::chrono;
    auto t  = system_clock::to_time_t(system_clock::now()); // current time_t
    std::tm tm{};                                           // tm struct
    localtime_r(&t, &tm);                                   // local time (thread-safe)
    std::ostringstream oss;                                 // string builder
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");         // format
    return oss.str();                                       // return string
  }
}

// ========= CORE TYPES =========
enum class SystemMode { Off, Heat, Cool };  // which equipment is permitted
enum class ControlMode { Auto, On };        // thermostat logic or forced ON

// Human-readable names for modes
static inline const char* to_string(SystemMode m) {
  switch (m) {
    case SystemMode::Off:  return "Off";
    case SystemMode::Heat: return "Heat";
    case SystemMode::Cool: return "Cool";
  }
  return "?";
}
static inline const char* to_string(ControlMode m) {
  switch (m) {
    case ControlMode::Auto: return "Auto";
    case ControlMode::On:   return "On";
  }
  return "?";
}

// Simple structs for DB logging
struct Reading     { std::string ts; double tempF{}; double humidity{}; };
struct StateChange { std::string ts; std::string what; std::string from; std::string to; };

// ========= SQLite Store =========
class SqliteStore {
public:
  // Open (or create) DB and ensure tables exist
  explicit SqliteStore(const std::string& file="thermostat.db") {
    if (sqlite3_open(file.c_str(), &db_) != SQLITE_OK)                 // open db
      throw std::runtime_error("SQLite open failed");
    exec("PRAGMA journal_mode=WAL;");                                   // faster/safer journaling
    exec("CREATE TABLE IF NOT EXISTS readings("
         "id INTEGER PRIMARY KEY, ts TEXT NOT NULL, tempF REAL NOT NULL, humidity REAL NOT NULL);");
    exec("CREATE TABLE IF NOT EXISTS events("
         "id INTEGER PRIMARY KEY, ts TEXT NOT NULL, what TEXT NOT NULL, from_state TEXT NOT NULL, to_state TEXT NOT NULL);");
    exec("CREATE TABLE IF NOT EXISTS settings("
         "key TEXT PRIMARY KEY, value TEXT NOT NULL);");
  }

  ~SqliteStore(){ if (db_) sqlite3_close(db_); }                        // close db on destruction

  // Insert a sensor reading row
  void logReading(const Reading& r){
    sqlite3_stmt* s=nullptr;                                            // prepared statement
    prep("INSERT INTO readings(ts,tempF,humidity) VALUES(?,?,?);",&s);  // prepare SQL
    sqlite3_bind_text  (s,1,r.ts.c_str(),-1,SQLITE_TRANSIENT);          // bind ts
    sqlite3_bind_double(s,2,r.tempF);                                   // bind tempF
    sqlite3_bind_double(s,3,r.humidity);                                // bind humidity
    stepf(s);                                                           // run + finalize
  }

  // Insert an event row
  void logEvent(const StateChange& e){
    sqlite3_stmt* s=nullptr;
    prep("INSERT INTO events(ts,what,from_state,to_state) VALUES(?,?,?,?);",&s);
    sqlite3_bind_text(s,1,e.ts.c_str(),-1,SQLITE_TRANSIENT);            // ts
    sqlite3_bind_text(s,2,e.what.c_str(),-1,SQLITE_TRANSIENT);          // what (system/control)
    sqlite3_bind_text(s,3,e.from.c_str(),-1,SQLITE_TRANSIENT);          // from_state
    sqlite3_bind_text(s,4,e.to.c_str(),-1,SQLITE_TRANSIENT);            // to_state
    stepf(s);
  }

  // Upsert setting key/value (saves current setpoint, modes, etc.)
  void saveSetting(const std::string& k,const std::string& v){
    sqlite3_stmt* s=nullptr;
    prep("INSERT INTO settings(key,value) VALUES(?,?) "
         "ON CONFLICT(key) DO UPDATE SET value=excluded.value;",&s);
    sqlite3_bind_text(s,1,k.c_str(),-1,SQLITE_TRANSIENT);               // key
    sqlite3_bind_text(s,2,v.c_str(),-1,SQLITE_TRANSIENT);               // value
    stepf(s);
  }

  // Load a setting value if present
  std::optional<std::string> loadSetting(const std::string& k){
    sqlite3_stmt* s=nullptr;
    prep("SELECT value FROM settings WHERE key=?;",&s);                  // prepare
    sqlite3_bind_text(s,1,k.c_str(),-1,SQLITE_TRANSIENT);               // bind key
    std::optional<std::string> out;
    if (sqlite3_step(s)==SQLITE_ROW)                                    // step row
      out = (const char*)sqlite3_column_text(s,0);                      // read value
    sqlite3_finalize(s);                                                // finalize
    return out;                                                         // return optional
  }

private:
  sqlite3* db_{};                                                       // raw DB handle

  // Execute an immediate SQL (no bindings)
  void exec(const char* sql){
    char* err=nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &err) != SQLITE_OK) {  // run
      std::string m = err ? err : "unknown";                            // copy error
      sqlite3_free(err);
      throw std::runtime_error("SQLite exec: " + m);                    // throw
    }
  }

  // Prepare a statement
  void prep(const char* sql, sqlite3_stmt** s){
    if (sqlite3_prepare_v2(db_, sql, -1, s, nullptr) != SQLITE_OK)      // compile
      throw std::runtime_error("SQLite prepare failed");
  }

  // Step and finalize, expecting SQLITE_DONE
  void stepf(sqlite3_stmt* s){
    if (sqlite3_step(s) != SQLITE_DONE) {                               // execute
      sqlite3_finalize(s);
      throw std::runtime_error("SQLite step failed");
    }
    sqlite3_finalize(s);                                                // cleanup
  }
};

// ========= REAL DHT22 Reader (pigpio timing) =========
class DHT22 {
public:
  explicit DHT22(unsigned bcmPin) : pin_(bcmPin) {
    // pigpio is already initialised by main(); set idle state and pull-up
    gpioSetMode(pin_, PI_OUTPUT);   // start as output to send start signal
    gpioWrite(pin_, 1);             // idle high
    gpioSetPullUpDown(pin_, PI_PUD_UP); // enable pull-up
  }

  // Read DHT22; returns true on success with tempF & humidity set
  bool read(double& tempF, double& humidity) {
    uint8_t data[5] = {0,0,0,0,0};                // 5 data bytes: hum_hi, hum_lo, temp_hi, temp_lo, checksum

    // 1) Host start: pull low >=1ms (use 18ms for safety), then high, then input
    gpioSetMode(pin_, PI_OUTPUT);                 // drive line
    gpioWrite(pin_, 0);                           // pull low
    gpioSleep(PI_TIME_RELATIVE, 0, 18000);        // 18ms
    gpioWrite(pin_, 1);                           // pull high
    gpioSetMode(pin_, PI_INPUT);                  // switch to input to read sensor
    gpioSetPullUpDown(pin_, PI_PUD_UP);           // keep pull-up

    // 2) Sensor response: ~80us LOW, ~80us HIGH, ~50us LOW before bits
    if (!waitForLevel(0, 1000)) return false;     // wait for sensor pulls LOW
    if (!waitForLevel(1, 1000)) return false;     // then HIGH
    if (!waitForLevel(0, 1000)) return false;     // then LOW (start of bit stream)

    // 3) Read 40 bits: for each bit, 50us LOW then HIGH; length of HIGH defines 0/1
    for (int i=0; i<40; ++i) {
      if (!waitForLevel(1, 1000)) return false;   // wait HIGH (start of bit)
      uint32_t start = gpioTick();                // timestamp at HIGH start
      if (!waitForLevel(0, 1000)) return false;   // wait LOW (bit ends)
      uint32_t high_us = gpioTick() - start;      // HIGH duration
      int bit = (high_us > 60) ? 1 : 0;           // threshold ~60us between 0 and 1
      data[i/8] = (uint8_t)((data[i/8] << 1) | (bit & 1)); // shift in bit
    }

    // 4) Verify checksum
    uint8_t sum = (uint8_t)(data[0] + data[1] + data[2] + data[3]); // sum of first 4 bytes
    if (sum != data[4]) return false;            // mismatch => bad read

    // 5) Convert to engineering units
    int hum_raw  = ((int)data[0] << 8) | data[1];    // humidity * 10
    int temp_raw = ((int)data[2] << 8) | data[3];    // temp * 10 (sign in MSB)
    bool neg = temp_raw & 0x8000;                    // negative flag
    if (neg) temp_raw &= 0x7FFF;                     // strip sign bit

    double hum   = hum_raw  / 10.0;                  // percent
    double tempC = temp_raw / 10.0;                  // Celsius
    if (neg) tempC = -tempC;                         // apply sign

    humidity = std::clamp(hum,   0.0, 100.0);        // clamp to plausible range
    tempF    = std::clamp(tempC * 9.0/5.0 + 32.0,    // C -> F
                           -40.0, 212.0);
    return true;                                     // success
  }

private:
  unsigned pin_;                                     // BCM pin for DHT data

  // Wait until pin matches level or timeout (us); returns true if level observed
  bool waitForLevel(int level, int timeout_us) {
    uint32_t start = gpioTick();                     // start timestamp (us)
    while ((gpioTick() - start) < (uint32_t)timeout_us) {
      if (gpioRead(pin_) == level) return true;      // got requested level
    }
    return false;                                    // timed out
  }
};

// ========= Actuator (relay driver; prints when state changes) =========
class RPiActuator {
public:
  RPiActuator(unsigned heatPin, unsigned coolPin, bool activeLow)
    : heatPin_(heatPin), coolPin_(coolPin), activeLow_(activeLow) {
    gpioSetMode(heatPin_, PI_OUTPUT);                // heat relay pin as output
    gpioSetMode(coolPin_, PI_OUTPUT);                // cool relay pin as output
    write(heatPin_, false);                          // start OFF
    write(coolPin_, false);                          // start OFF
  }

  void setHeat(bool on){
    if (heat_ != on) {                               // only act on change
      heat_ = on; write(heatPin_, on);               // drive relay
      std::cout << "[Actuator] Heat " << (on?"ON":"OFF") << "\n";
    }
  }

  void setCool(bool on){
    if (cool_ != on) {                               // only act on change
      cool_ = on; write(coolPin_, on);               // drive relay
      std::cout << "[Actuator] Cool " << (on?"ON":"OFF") << "\n";
    }
  }

  bool heat() const { return heat_; }                // current heat relay state
  bool cool() const { return cool_; }                // current cool relay state

private:
  unsigned heatPin_, coolPin_;                       // relay pins
  bool activeLow_;                                   // relay logic polarity
  bool heat_{false}, cool_{false};                   // cached states

  // Convert logical ON/OFF to pin level considering activeLow
  void write(unsigned pin, bool on){
    int level = activeLow_ ? (on ? 0 : 1) : (on ? 1 : 0); // LOW means ON if active-low
    gpioWrite(pin, level);                                 // set output
  }
};

// ========= Thermostat Core =========
class Thermostat {
public:
  // Configuration with defaults
  struct Config {
    double setpointF = 72.0;                         // target temp (F)
    double deadbandF = 1.0;                          // hysteresis band (F)
    SystemMode  system  = SystemMode::Off;           // Off/Heat/Cool
    ControlMode control = ControlMode::Auto;         // Auto/On
  };

  // Inject dependencies (sensor, actuator, database)
  Thermostat(DHT22& s, RPiActuator& a, SqliteStore& st)
    : sensor_(s), act_(a), store_(st) {
    // Restore persisted settings if present
    if (auto v=store_.loadSetting("setpointF")) cfg_.setpointF = std::stod(*v);
    if (auto v=store_.loadSetting("deadbandF")) cfg_.deadbandF = std::stod(*v);
    if (auto v=store_.loadSetting("system"))    cfg_.system    = parseSystem(*v);
    if (auto v=store_.loadSetting("control"))   cfg_.control   = parseControl(*v);
  }

  // Mutators save to DB so settings persist across runs
  void setSetpoint(double f){ cfg_.setpointF=f; store_.saveSetting("setpointF", std::to_string(f)); }
  void setDeadband(double f){ cfg_.deadbandF=f; store_.saveSetting("deadbandF", std::to_string(f)); }

  void setSystem(SystemMode m){
    if (cfg_.system != m)                                                  // log only on change
      store_.logEvent({ util::now_iso8601(), "system", to_string(cfg_.system), to_string(m) });
    cfg_.system = m;                                                       // apply
    store_.saveSetting("system", to_string(m));                            // persist
    if (cfg_.system == SystemMode::Off) { act_.setHeat(false); act_.setCool(false); } // ensure OFF
  }

  void setControl(ControlMode m){
    if (cfg_.control != m)                                                 // log only on change
      store_.logEvent({ util::now_iso8601(), "control", to_string(cfg_.control), to_string(m) });
    cfg_.control = m;                                                      // apply
    store_.saveSetting("control", to_string(m));                           // persist
  }

  // One control loop tick: read sensor, log reading, drive relays
  void tick() {
    double tF=0.0, hum=0.0; bool ok=false;                                 // output vars
    // Retry a few times because DHT22 can be finicky
    for (int i=0;i<3 && !ok;i++){ ok = sensor_.read(tF, hum); if(!ok) std::this_thread::sleep_for(std::chrono::milliseconds(250)); }
    if (!ok) return;                                                       // skip this tick on failure

    store_.logReading({ util::now_iso8601(), tF, hum });                   // DB log

    if (cfg_.system == SystemMode::Off) {                                  // system OFF => both relays OFF
      act_.setHeat(false); act_.setCool(false); return;
    }

    if (cfg_.control == ControlMode::On) {                                 // manual override
      if (cfg_.system == SystemMode::Heat) { act_.setHeat(true);  act_.setCool(false); }
      if (cfg_.system == SystemMode::Cool) { act_.setCool(true);  act_.setHeat(false); }
      return;                                                              // done
    }

    // Auto mode: classic thermostat hysteresis
    if (cfg_.system == SystemMode::Heat) {
      double onT  = cfg_.setpointF - cfg_.deadbandF;                       // turn ON below this
      double offT = cfg_.setpointF + 0.1;                                  // turn OFF slightly above setpoint
      if      (tF < onT)   { act_.setHeat(true);  act_.setCool(false); }
      else if (tF >= offT) { act_.setHeat(false); }
    } else if (cfg_.system == SystemMode::Cool) {
      double onT  = cfg_.setpointF + cfg_.deadbandF;                       // turn ON above this
      double offT = cfg_.setpointF - 0.1;                                  // turn OFF slightly below setpoint
      if      (tF > onT)   { act_.setCool(true);  act_.setHeat(false); }
      else if (tF <= offT) { act_.setCool(false); }
    }
  }

  // Expose current config for status printing
  Config config() const { return cfg_; }

private:
  DHT22&       sensor_;                                                   // real DHT22 reader
  RPiActuator& act_;                                                      // relay driver
  SqliteStore& store_;                                                    // DB
  Config       cfg_;                                                      // current settings

  // Helpers to parse strings from DB back into enums
  static SystemMode  parseSystem(const std::string& s){
    std::string t=s; std::transform(t.begin(),t.end(),t.begin(),::tolower);
    if (t=="off")  return SystemMode::Off;
    if (t=="heat") return SystemMode::Heat;
    if (t=="cool") return SystemMode::Cool;
    return SystemMode::Off;
  }
  static ControlMode parseControl(const std::string& s){
    std::string t=s; std::transform(t.begin(),t.end(),t.begin(),::tolower);
    if (t=="auto") return ControlMode::Auto;
    if (t=="on")   return ControlMode::On;
    return ControlMode::Auto;
  }
};

// ========= Small helpers =========
static inline bool btnPressed(unsigned bcmPin) { return gpioRead(bcmPin) == 0; } // active-LOW => pressed at 0

// ========= MAIN =========
int main() {
  try {
    std::srand((unsigned)std::time(nullptr));                              // random seed (not critical)

    if (gpioInitialise() < 0) {                                            // init pigpio library
      std::cerr<<"pigpio init failed\n"; 
      return 1;
    }

    // Configure relay outputs and ensure they're OFF at start
    gpioSetMode(PIN_HEAT, PI_OUTPUT);
    gpioSetMode(PIN_COOL, PI_OUTPUT);
    gpioWrite(PIN_HEAT, RELAY_ACTIVE_LOW?1:0);                             // OFF for active-low
    gpioWrite(PIN_COOL, RELAY_ACTIVE_LOW?1:0);                             // OFF for active-low

    // Configure button inputs: pull-ups + glitch filter for debounce
    auto cfgButton = [](unsigned pin){
      gpioSetMode(pin, PI_INPUT);                                          // input
      gpioSetPullUpDown(pin, PI_PUD_UP);                                   // internal pull-up
      gpioGlitchFilter(pin, DEBOUNCE_MS * 1000);                           // microseconds
    };
    cfgButton(PIN_BTN_HEAT);
    cfgButton(PIN_BTN_COOL);
    cfgButton(PIN_BTN_OFF);
    cfgButton(PIN_BTN_AUTO);
    cfgButton(PIN_BTN_ON);

    // Construct DB, sensor, actuator, and controller
    SqliteStore store;                                                     // opens/creates DB
    DHT22       sensor(PIN_DHT);                                           // real DHT22
    RPiActuator actuator(PIN_HEAT, PIN_COOL, RELAY_ACTIVE_LOW);            // relay driver
    Thermostat  t(sensor, actuator, store);                                // control logic

    std::cout << "Pi Thermostat running (REAL DHT22, buttons active).\n";
    std::cout << "Commands: status | set <tempF> | deadband <F> | mode <off|heat|cool> | control <auto|on> | quit\n";

    bool running = true;                                                   // controls loop exit

    // Control loop: read DHT22, log to DB, drive relays every second
    std::thread controlLoop([&](){
      while (running) {
        t.tick();                                                          // one control pass
        std::this_thread::sleep_for(std::chrono::seconds(1));              // 1 Hz
      }
    });

    // Button polling loop: watch for rising "press" (active-LOW go 1->0)
    std::thread buttonLoop([&](){
      bool pH=false,pC=false,pO=false,pA=false,pOn=false;                  // previous states
      while (running) {
        bool h  = btnPressed(PIN_BTN_HEAT);
        bool c  = btnPressed(PIN_BTN_COOL);
        bool o  = btnPressed(PIN_BTN_OFF);
        bool au = btnPressed(PIN_BTN_AUTO);
        bool on = btnPressed(PIN_BTN_ON);

        if (h  && !pH)  { t.setSystem(SystemMode::Heat);  std::cout<<"[Button] System=Heat\n"; }
        if (c  && !pC)  { t.setSystem(SystemMode::Cool);  std::cout<<"[Button] System=Cool\n"; }
        if (o  && !pO)  { t.setSystem(SystemMode::Off);   std::cout<<"[Button] System=Off\n"; }
        if (au && !pA)  { t.setControl(ControlMode::Auto);std::cout<<"[Button] Control=Auto\n"; }
        if (on && !pOn) { t.setControl(ControlMode::On);  std::cout<<"[Button] Control=On\n"; }

        pH=h; pC=c; pO=o; pA=au; pOn=on;                                     // update previous
        std::this_thread::sleep_for(std::chrono::milliseconds(20));          // ~50 Hz poll
      }
    });

    // Minimal CLI to interact without buttons
    auto print_status = [&](){
      auto c = t.config();
      std::cout << "Status: System="<<to_string(c.system)
                << " Control="<<to_string(c.control)
                << " Setpoint="<<c.setpointF<<"F"
                << " Deadband="<<c.deadbandF<<"F\n";
    };

    // Read commands until user types quit/exit or sends EOF
    for (std::string line; std::cout<<"> " && std::getline(std::cin, line); ) {
      std::istringstream iss(line);                                        // tokenizer
      std::string cmd; if (!(iss>>cmd)) continue;                          // first token
      std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);      // case-insensitive

      if (cmd=="quit" || cmd=="exit") break;                               // stop program
      else if (cmd=="status") { print_status(); }                          // print modes + setpoint
      else if (cmd=="set") {                                               // set <tempF>
        double f; if (iss>>f) { t.setSetpoint(f); print_status(); }
        else std::cout<<"Usage: set <tempF>\n";
      } else if (cmd=="deadband") {                                        // deadband <F>
        double f; if (iss>>f) { t.setDeadband(f); print_status(); }
        else std::cout<<"Usage: deadband <F>\n";
      } else if (cmd=="mode") {                                            // mode <off|heat|cool>
        std::string m; if (!(iss>>m)) { std::cout<<"Usage: mode <off|heat|cool>\n"; continue; }
        if      (m=="off")  t.setSystem(SystemMode::Off);
        else if (m=="heat") t.setSystem(SystemMode::Heat);
        else if (m=="cool") t.setSystem(SystemMode::Cool);
        else { std::cout<<"Modes: off | heat | cool\n"; continue; }
        print_status();
      } else if (cmd=="control") {                                         // control <auto|on>
        std::string m; if (!(iss>>m)) { std::cout<<"Usage: control <auto|on>\n"; continue; }
        if      (m=="auto") t.setControl(ControlMode::Auto);
        else if (m=="on")   t.setControl(ControlMode::On);
        else { std::cout<<"Controls: auto | on\n"; continue; }
        print_status();
      } else {
        std::cout << "Unknown command. Try: status | set | deadband | mode | control | quit\n";
      }
    }

    // Graceful shutdown
    running = false;                                                       // tell threads to stop
    controlLoop.join();                                                    // wait control loop
    buttonLoop.join();                                                     // wait button loop
    gpioTerminate();                                                       // release pigpio
    std::cout << "Goodbye.\n";
    return 0;

  } catch (const std::exception& ex) {                                     // catch any throw
    std::cerr << "Fatal: " << ex.what() << "\n";                           // report error
    gpioTerminate();                                                       // best-effort cleanup
    return 1;
  }
}
