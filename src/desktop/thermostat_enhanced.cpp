/*
  Thermostat Enhanced (Quiet Console + Print-on-Change + Proper OFF Handling)
  ---------------------------------------------------------------------------
  - Console stays clean: use `status` to see Mode | Setpoint | Deadband.
  - Ticks log to SQLite (thermostat.db) on every second: table `readings`.
  - Actuator messages (Heat/Cool ON/OFF) print only when state actually changes.
  - Switching to Mode::Off turns both actuators OFF once; no repeating OFF calls per tick.
  - Tick file logging is OFF by default (DB logging remains ON).

  Commands:
    set <tempF>
    mode <off|heat|cool>
    status
    next
    logconsole on|off   (tick lines -> console; default OFF)
    logfile on|off      (tick lines -> thermostat_ticks.log; default OFF)
    actlog on|off       (actuator ON/OFF messages; default ON)
    help
    quit

  Build (MSYS2 / MinGW on Windows):
    "C:\\msys64\\mingw64\\bin\\g++.exe" -std=c++17 thermostat_enhanced.cpp -DUSE_SQLITE ^
      -I"C:\\msys64\\mingw64\\include" -L"C:\\msys64\\mingw64\\lib" -lsqlite3 -O2 -o thermostat.exe

  Run:
    .\\thermostat.exe
*/

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef USE_SQLITE
#include <sqlite3.h>
#endif

// ---------- Utilities ----------
namespace util {
  inline std::string now_iso8601() {
    using namespace std::chrono;
    auto t  = system_clock::to_time_t(system_clock::now());
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
  }
  inline int hm_to_minutes(const std::string& hm) {
    int H=0,M=0; char c=':';
    std::istringstream iss(hm);
    if (!(iss >> H >> c >> M) || c!=':' || H<0 || H>23 || M<0 || M>59)
      throw std::invalid_argument("Bad HH:MM: " + hm);
    return H*60 + M;
  }
  inline std::string minutes_to_hm(int minutes) {
    minutes = (minutes % (24*60) + (24*60)) % (24*60);
    int H = minutes / 60, M = minutes % 60;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << H << ":"
        << std::setw(2) << std::setfill('0') << M;
    return oss.str();
  }
}

// ---------- Core Types ----------
enum class Mode { Off, Heat, Cool };
static inline const char* to_string(Mode m) {
  switch (m) { case Mode::Off: return "Off"; case Mode::Heat: return "Heat"; case Mode::Cool: return "Cool"; }
  return "Unknown";
}
struct Reading    { std::string timestamp; double temperatureF{}; };
struct StateChange{ std::string timestamp; Mode from{}, to{}; };

class ISensor { public: virtual ~ISensor() = default; virtual double readFahrenheit() = 0; };
class IActuator {
public:
  virtual ~IActuator() = default;
  virtual void setHeat(bool on) = 0;
  virtual void setCool(bool on) = 0;
};
class IDataStore {
public:
  virtual ~IDataStore() = default;
  virtual void logReading(const Reading& r) = 0;
  virtual void logStateChange(const StateChange& e) = 0;
  virtual void saveSetting(const std::string& key, const std::string& value) = 0;
  virtual std::optional<std::string> loadSetting(const std::string& key) = 0;
};

// ---------- A&DS: RingBuffer + Moving Average ----------
template <typename T>
class RingBuffer {
public:
  explicit RingBuffer(size_t cap) : cap_(cap){ buf_.reserve(cap); }
  void push(const T& v){ if(buf_.size()<cap_) buf_.push_back(v); else { buf_[head_]=v; head_=(head_+1)%cap_; } }
  template <class F> double average(F f) const {
    if (buf_.empty()) return 0.0;
    double s=0; for (auto& x: buf_) s += f(x); return s / double(buf_.size());
  }
private:
  std::vector<T> buf_; size_t cap_; size_t head_=0;
};
class MovingAverageFilter {
public:
  explicit MovingAverageFilter(size_t win): buf_(win) {}
  double add(double v){ buf_.push(v); return buf_.average([](double x){return x;}); }
private:
  RingBuffer<double> buf_;
};

// ---------- Demo Sensor ----------
class MockSensor : public ISensor {
public:
  explicit MockSensor(double startF=70.0): cur_(startF) {}
  double readFahrenheit() override {
    double ambient = 72.0;
    cur_ += (ambient - cur_) * 0.02;
    cur_ += (std::rand()%100 - 50) * 0.01;
    return cur_;
  }
private: double cur_;
};

// ---------- Actuator (print on change only) ----------
class ConsoleActuator : public IActuator {
public:
  void setHeat(bool on) override {
    if (heat_ != on) { heat_ = on; if (consoleAnnounce_) std::cout << "[Actuator] Heat " << (heat_?"ON":"OFF") << "\n"; }
  }
  void setCool(bool on) override {
    if (cool_ != on) { cool_ = on; if (consoleAnnounce_) std::cout << "[Actuator] Cool " << (cool_?"ON":"OFF") << "\n"; }
  }
  void setConsoleAnnounce(bool b){ consoleAnnounce_ = b; }
private:
  bool heat_ = false, cool_ = false, consoleAnnounce_ = true;
};

// ---------- Databases: CSV + SQLite ----------
class CsvStore : public IDataStore {
public:
  explicit CsvStore(const std::string& base="thermostat_log")
    : r_(base+"_readings.csv"), e_(base+"_events.csv"), s_(base+"_settings.csv") {
    touch(r_, "timestamp,temperatureF\n");
    touch(e_, "timestamp,from,to\n");
    touch(s_, "key,value\n");
  }
  void logReading(const Reading& r) override { std::ofstream f(r_, std::ios::app); f << r.timestamp << "," << r.temperatureF << "\n"; }
  void logStateChange(const StateChange& ev) override { std::ofstream f(e_, std::ios::app); f << ev.timestamp << "," << to_string(ev.from) << "," << to_string(ev.to) << "\n"; }
  void saveSetting(const std::string& k,const std::string& v) override {
    auto kv = loadAll();
    kv[k]=v;
    std::ofstream f(s_, std::ios::trunc); f << "key,value\n";
    for (auto& [K,V]: kv) f << K << "," << escape(V) << "\n";
  }
  std::optional<std::string> loadSetting(const std::string& k) override {
    auto kv = loadAll(); auto it = kv.find(k); if (it==kv.end()) return std::nullopt; return it->second;
  }
private:
  std::string r_,e_,s_;
  static void touch(const std::string& p, const char* hdr){ std::ifstream in(p); if(!in.good()){ std::ofstream f(p); f<<hdr; } }
  static std::string escape(const std::string& s){ std::string o; for(char c: s){ if(c==',') o+="\\,"; else o+=c; } return o; }
  static std::string unescape(const std::string& s){ std::string o; for(size_t i=0;i<s.size();++i){ if(s[i]=='\\'&&i+1<s.size()&&s[i+1]==','){o+=',';++i;} else o+=s[i]; } return o; }
  std::map<std::string,std::string> loadAll(){
    std::map<std::string,std::string> kv; std::ifstream f(s_); std::string line; std::getline(f,line);
    while(std::getline(f,line)){ if(line.empty()) continue; auto p=line.find(','); if(p==std::string::npos) continue;
      kv[line.substr(0,p)] = unescape(line.substr(p+1)); }
    return kv;
  }
};

#ifdef USE_SQLITE
class SqliteStore : public IDataStore {
public:
  explicit SqliteStore(const std::string& file="thermostat.db"){
    if (sqlite3_open(file.c_str(), &db_) != SQLITE_OK) throw std::runtime_error("SQLite open failed");
    exec("PRAGMA journal_mode=WAL;");
    exec("CREATE TABLE IF NOT EXISTS readings(id INTEGER PRIMARY KEY, ts TEXT NOT NULL, tempF REAL NOT NULL);");
    exec("CREATE TABLE IF NOT EXISTS events(id INTEGER PRIMARY KEY, ts TEXT NOT NULL, from_state TEXT NOT NULL, to_state TEXT NOT NULL);");
    exec("CREATE TABLE IF NOT EXISTS settings(key TEXT PRIMARY KEY, value TEXT NOT NULL);");
  }
  ~SqliteStore() override { if (db_) sqlite3_close(db_); }
  void logReading(const Reading& r) override {
    sqlite3_stmt* s=nullptr; prep("INSERT INTO readings(ts,tempF) VALUES(?,?);",&s);
    sqlite3_bind_text(s,1,r.timestamp.c_str(),-1,SQLITE_TRANSIENT);
    sqlite3_bind_double(s,2,r.temperatureF);
    stepf(s);
  }
  void logStateChange(const StateChange& e) override {
    sqlite3_stmt* s=nullptr; prep("INSERT INTO events(ts,from_state,to_state) VALUES(?,?,?);",&s);
    sqlite3_bind_text(s,1,e.timestamp.c_str(),-1,SQLITE_TRANSIENT);
    sqlite3_bind_text(s,2,to_string(e.from),-1,SQLITE_TRANSIENT);
    sqlite3_bind_text(s,3,to_string(e.to),-1,SQLITE_TRANSIENT);
    stepf(s);
  }
  void saveSetting(const std::string& k,const std::string& v) override {
    sqlite3_stmt* s=nullptr; prep("INSERT INTO settings(key,value) VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value;",&s);
    sqlite3_bind_text(s,1,k.c_str(),-1,SQLITE_TRANSIENT);
    sqlite3_bind_text(s,2,v.c_str(),-1,SQLITE_TRANSIENT);
    stepf(s);
  }
  std::optional<std::string> loadSetting(const std::string& k) override {
    sqlite3_stmt* s=nullptr; prep("SELECT value FROM settings WHERE key=?;",&s);
    sqlite3_bind_text(s,1,k.c_str(),-1,SQLITE_TRANSIENT);
    std::optional<std::string> out;
    if (sqlite3_step(s)==SQLITE_ROW) out = reinterpret_cast<const char*>(sqlite3_column_text(s,0));
    sqlite3_finalize(s); return out;
  }
private:
  sqlite3* db_ = nullptr;
  void exec(const char* sql){ char* err=nullptr; if(sqlite3_exec(db_,sql,nullptr,nullptr,&err)!=SQLITE_OK){ std::string m=err?err:"unknown"; sqlite3_free(err); throw std::runtime_error("SQLite exec: "+m);} }
  void prep(const char* sql, sqlite3_stmt** s){ if(sqlite3_prepare_v2(db_,sql,-1,s,nullptr)!=SQLITE_OK) throw std::runtime_error("SQLite prepare failed"); }
  void stepf(sqlite3_stmt* s){ if(sqlite3_step(s)!=SQLITE_DONE){ sqlite3_finalize(s); throw std::runtime_error("SQLite step failed"); } sqlite3_finalize(s); }
};
#endif

// ---------- A&DS: Schedule (binary search) ----------
struct ScheduleEntry { int minuteOfDay; double setpointF; };
class Schedule {
public:
  void add(const std::string& hm, double sp){ entries_.push_back({util::hm_to_minutes(hm), sp});
    std::sort(entries_.begin(), entries_.end(), [](auto&a,auto&b){return a.minuteOfDay<b.minuteOfDay;}); }
  void setDefault(double sp){ def_ = sp; }
  double currentSetpoint(int nowMin) const {
    if (entries_.empty()) return def_;
    int idx = lastLE(nowMin); if (idx>=0) return entries_[idx].setpointF; return entries_.back().setpointF;
  }
  std::optional<ScheduleEntry> nextChange(int nowMin) const {
    if (entries_.empty()) return std::nullopt;
    auto it = std::upper_bound(entries_.begin(), entries_.end(), nowMin, [](int v,const ScheduleEntry& e){return v<e.minuteOfDay;});
    if (it==entries_.end()) return entries_.front(); return *it;
  }
  const std::vector<ScheduleEntry>& entries() const { return entries_; }
private:
  std::vector<ScheduleEntry> entries_; double def_ = 72.0;
  int lastLE(int nowMin) const {
    int lo=0, hi=(int)entries_.size()-1, ans=-1;
    while(lo<=hi){ int mid=(lo+hi)/2; if(entries_[mid].minuteOfDay<=nowMin){ ans=mid; lo=mid+1;} else hi=mid-1; }
    return ans;
  }
};

// ---------- Quiet Tick Logger ----------
class TickLogger {
public:
  explicit TickLogger(const std::string& path): path_(path) {}
  void setConsole(bool on){ std::scoped_lock lk(mu_); c_ = on; }
  void setFile(bool on){ std::scoped_lock lk(mu_); f_ = on; }
  void log(double tempF, const std::string& mode, double setF, double deadbandF) {
    std::scoped_lock lk(mu_);
    const std::string line = "[" + util::now_iso8601() + "] "
      "T=" + fmt(tempF) + "F Set=" + fmt(setF) + "F Mode=" + mode + " (deadband=" + fmt(deadbandF) + "F)\n";
    if (c_) std::cout << line;
    if (f_) { std::ofstream out(path_, std::ios::app); out << line; }
  }
private:
  std::mutex mu_; std::string path_; bool c_=false, f_=false; // default: file OFF now
  static std::string fmt(double v){ std::ostringstream o; o<<std::fixed<<std::setprecision(2)<<v; return o.str(); }
};

// ---------- Thermostat FSM ----------
class Thermostat {
public:
  struct Config { double setpointF=72.0; double deadbandF=1.0; Mode mode=Mode::Off; };
  Thermostat(ISensor& s, IActuator& a, IDataStore& d, TickLogger* log=nullptr)
    : sensor_(s), actuator_(a), store_(d), filter_(10), logger_(log) {
    if (auto v=store_.loadSetting("setpointF")) cfg_.setpointF = std::stod(*v);
    if (auto v=store_.loadSetting("deadbandF")) cfg_.deadbandF = std::stod(*v);
    if (auto v=store_.loadSetting("mode"))      cfg_.mode = parseMode(*v);
  }
  void setSchedule(const Schedule& sch){ sched_ = sch; }
  void setSetpoint(double f){ cfg_.setpointF=f; store_.saveSetting("setpointF", std::to_string(f)); }
  void setMode(Mode m){
    if (cfg_.mode != m) store_.logStateChange({ util::now_iso8601(), cfg_.mode, m });
    cfg_.mode = m;
    store_.saveSetting("mode", to_string(m));
    if (cfg_.mode == Mode::Off) {        // turn OFF once when switching into Off
      actuator_.setHeat(false);
      actuator_.setCool(false);
    }
  }
  void setDeadband(double f){ cfg_.deadbandF=f; store_.saveSetting("deadbandF", std::to_string(f)); }

  void tick(){
    double raw = sensor_.readFahrenheit();
    double tF  = filter_.add(raw);
    store_.logReading({ util::now_iso8601(), tF });  // <-- DB logging every tick

    // schedule align
    int nowMin = nowMinuteOfDay();
    double sp = sched_.currentSetpoint(nowMin);
    if (std::abs(sp - cfg_.setpointF) > 1e-6) { cfg_.setpointF = sp; store_.saveSetting("setpointF", std::to_string(sp)); }

    switch (cfg_.mode){
      case Mode::Off:
        // No repeated OFF calls; actuators already forced OFF in setMode(Off).
        break;
      case Mode::Heat:
        controlHeat(tF);
        break;
      case Mode::Cool:
        controlCool(tF);
        break;
    }
    if (logger_) logger_->log(tF, to_string(cfg_.mode), cfg_.setpointF, cfg_.deadbandF);
  }

  Config config() const { return cfg_; }
  std::optional<ScheduleEntry> nextScheduleChange() const { return sched_.nextChange(nowMinuteOfDay()); }

private:
  ISensor& sensor_; IActuator& actuator_; IDataStore& store_;
  Config cfg_; MovingAverageFilter filter_; Schedule sched_; TickLogger* logger_;

  static Mode parseMode(const std::string& s){ std::string t=s; std::transform(t.begin(),t.end(),t.begin(),::tolower);
    if(t=="off") return Mode::Off; if(t=="heat") return Mode::Heat; if(t=="cool") return Mode::Cool; return Mode::Off; }
  static int nowMinuteOfDay(){ std::time_t tt=std::time(nullptr); std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm,&tt);
#else
    localtime_r(&tt,&tm);
#endif
    return tm.tm_hour*60 + tm.tm_min;
  }
  void controlHeat(double tF){
    static bool heatOn=false;
    double onT = cfg_.setpointF - cfg_.deadbandF;
    double offT= cfg_.setpointF + 0.1;
    if (!heatOn && tF < onT) { actuator_.setHeat(true); actuator_.setCool(false); heatOn=true; }
    else if (heatOn && tF >= offT) { actuator_.setHeat(false); heatOn=false; }
  }
  void controlCool(double tF){
    static bool coolOn=false;
    double onT = cfg_.setpointF + cfg_.deadbandF;
    double offT= cfg_.setpointF - 0.1;
    if (!coolOn && tF > onT) { actuator_.setCool(true); actuator_.setHeat(false); coolOn=true; }
    else if (coolOn && tF <= offT) { actuator_.setCool(false); coolOn=false; }
  }
};

// ---------- CLI ----------
static void help(){
  std::cout <<
    "Commands:\n"
    "  set <tempF>\n"
    "  mode <off|heat|cool>\n"
    "  status\n"
    "  next\n"
    "  logconsole on|off   (tick lines -> console)\n"
    "  logfile on|off      (tick lines -> thermostat_ticks.log)\n"
    "  actlog on|off       (actuator ON/OFF messages)\n"
    "  help\n"
    "  quit\n";
}

int main(){
  try{
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    MockSensor sensor(70.0);
    ConsoleActuator actuator; actuator.setConsoleAnnounce(true);

#ifdef USE_SQLITE
    SqliteStore store;
#else
    CsvStore store;
#endif

    Schedule schedule; schedule.setDefault(72.0);
    schedule.add("06:30", 70.0);
    schedule.add("08:00", 68.0);
    schedule.add("17:30", 72.0);
    schedule.add("22:30", 66.0);

    TickLogger tlog("thermostat_ticks.log");
    tlog.setConsole(false); // quiet by default
    tlog.setFile(false);    // file logging OFF by default (DB logging is still ON)

    Thermostat t(sensor, actuator, store, &tlog);
    t.setSchedule(schedule);

    auto print_status = [&](){
      auto c = t.config();
      std::cout << "Status: Mode=" << to_string(c.mode)
                << " | Setpoint=" << c.setpointF << "F"
                << " | Deadband=" << c.deadbandF << "F\n";
    };

    std::cout << "Thermostat Enhanced Demo (Quiet + Print-on-Change + Proper OFF)\n";
    help();

    bool running = true;
    std::thread loop([&](){
      while(running){
        t.tick();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    });

    for (std::string line; std::cout << "> " && std::getline(std::cin, line); ){
      std::istringstream iss(line); std::string cmd; if(!(iss>>cmd)) continue;
      std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

      if (cmd=="quit" || cmd=="exit") break;
      else if (cmd=="help") help();
      else if (cmd=="status") print_status();
      else if (cmd=="set") {
        double f; if (iss>>f) { t.setSetpoint(f); print_status(); } else std::cout<<"Usage: set <tempF>\n";
      } else if (cmd=="mode") {
        std::string m; if(!(iss>>m)){ std::cout<<"Usage: mode <off|heat|cool>\n"; continue; }
        std::transform(m.begin(), m.end(), m.begin(), ::tolower);
        if (m=="off") t.setMode(Mode::Off);
        else if (m=="heat") t.setMode(Mode::Heat);
        else if (m=="cool") t.setMode(Mode::Cool);
        else { std::cout<<"Modes: off | heat | cool\n"; continue; }
        print_status();
      } else if (cmd=="next") {
        auto n = t.nextScheduleChange();
        if (n) std::cout << "Next: " << util::minutes_to_hm(n->minuteOfDay) << " -> " << n->setpointF << "F\n";
        else std::cout << "No scheduled changes.\n";
      } else if (cmd=="logconsole") {
        std::string v; if(!(iss>>v)){ std::cout<<"Usage: logconsole on|off\n"; continue; }
        std::transform(v.begin(), v.end(), v.begin(), ::tolower);
        tlog.setConsole(v=="on");
        std::cout << "Console tick logging: " << (v=="on"?"ON":"OFF") << "\n";
      } else if (cmd=="logfile") {
        std::string v; if(!(iss>>v)){ std::cout<<"Usage: logfile on|off\n"; continue; }
        std::transform(v.begin(), v.end(), v.begin(), ::tolower);
        tlog.setFile(v=="on");
        std::cout << "File tick logging: " << (v=="on"?"ON":"OFF") << " (thermostat_ticks.log)\n";
      } else if (cmd=="actlog") {
        std::string v; if(!(iss>>v)){ std::cout<<"Usage: actlog on|off\n"; continue; }
        std::transform(v.begin(), v.end(), v.begin(), ::tolower);
        actuator.setConsoleAnnounce(v=="on");
        std::cout << "Actuator console logging: " << (v=="on"?"ON":"OFF") << "\n";
      } else {
        std::cout << "Unknown command. Type 'help' for options.\n";
      }
    }

    running = false; loop.join();
    std::cout << "Goodbye.\n";
    return 0;

  } catch(const std::exception& ex){
    std::cerr << "Fatal error: " << ex.what() << "\n";
    return 1;
  }
}
