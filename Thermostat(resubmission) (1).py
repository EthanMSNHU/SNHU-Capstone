#
# Thermostat - This is the Python code used to demonstrate
# the functionality of the thermostat that we have prototyped throughout
# the course. 
#
# This code works with the test circuit that was built for module 7.
#
# Functionality:
#
# The thermostat has three states: off, heat, cool
#
# The lights will represent the state that the thermostat is in.
#
# If the thermostat is set to off, the lights will both be off.
#
# If the thermostat is set to heat, the Red LED will be fading in 
# and out if the current temperature is below the set temperature;
# otherwise, the Red LED will be on solid.
#
# If the thermostat is set to cool, the Blue LED will be fading in 
# and out if the current temperature is above the set temperature;
# otherwise, the Blue LED will be on solid.
#
# One button will cycle through the three states of the thermostat.
#
# One button will raise the setpoint by a degree.
#
# One button will lower the setpoint by a degree.
#
# The LCD display will display the date and time on one line and
# alternate the second line between the current temperature and 
# the state of the thermostat along with its set temperature.
#
# The Thermostat will send a status update to the TemperatureServer
# over the serial port every 30 seconds in a comma delimited string
# including the state of the thermostat, the current temperature
# in degrees Fahrenheit, and the setpoint of the thermostat.
#
#------------------------------------------------------------------
# Change History
#------------------------------------------------------------------
# Version   |   Description
#------------------------------------------------------------------
#    1          Initial Development
#------------------------------------------------------------------

##
## Import necessary to provide timing in the main loop
##
from time import sleep
from datetime import datetime

##
## Imports required to allow us to build a fully functional state machine
##
from statemachine import StateMachine, State

##
## Imports necessary to provide connectivity to the 
## thermostat sensor and the I2C bus
##
import board
import adafruit_ahtx0

##
## These are the packages that we need to pull in so that we can work
## with the GPIO interface on the Raspberry Pi board and work with
## the 16x2 LCD display
##
# import board - already imported for I2C connectivity
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd

## This imports the Python serial package to handle communications over the
## Raspberry Pi's serial port. 
import serial

##
## Imports required to handle our Button, and our PWMLED devices
##
from gpiozero import Button, PWMLED

##
## This package is necessary so that we can delegate the blinking
## lights to their own thread so that more work can be done at the
## same time
##
from threading import Thread

##
## This is needed to get coherent matching of temperatures.
##
from math import floor

##
## DEBUG flag - boolean value to indicate whether or not to print 
## status messages on the console of the program
## 
DEBUG = True

##
## Create an I2C instance so that we can communicate with
## devices on the I2C bus.
##
i2c = board.I2C()

##
## Initialize our Temperature and Humidity sensor
##
thSensor = adafruit_ahtx0.AHTx0(i2c)

##
## Initialize our serial connection
##
## Because we imported the entire package instead of just importing Serial and
## some of the other flags from the serial package, we need to reference those
## objects with dot notation.
##
## e.g. ser = serial.Serial
##
ser = serial.Serial(
		port='/dev/ttyS0', # This would be /dev/ttyAM0 prior to Raspberry Pi 3
		baudrate = 115200, # This sets the speed of the serial interface in
						   # bits/second
		parity=serial.PARITY_NONE,      # Disable parity
		stopbits=serial.STOPBITS_ONE,   # Serial protocol will use one stop bit
		bytesize=serial.EIGHTBITS,      # We are using 8-bit bytes 
		timeout=1          # Configure a 1-second timeout
)

##
## Our two LEDs, utilizing GPIO 18, and GPIO 23
##
redLight = PWMLED(18)
blueLight = PWMLED(23)


##
## ManagedDisplay - Class intended to manage the 16x2 
## Display
##
## This code is largely taken from the work done in module 4, and
## converted into a class so that we can more easily consume the 
## operational capabilities.
##
class ManagedDisplay():
	##
	## Class Initialization method to setup the display
	##
	def __init__(self):
		##
		## Setup the six GPIO lines to communicate with the display.
		##
		## You need to make sure that the port mappings match the
		## physical wiring of the display interface to the GPIO interface.
		##
		## compatible with all versions of RPI as of Jan. 2019
		##
		self.lcd_rs = digitalio.DigitalInOut(board.D17)
		self.lcd_en = digitalio.DigitalInOut(board.D27)
		self.lcd_d4 = digitalio.DigitalInOut(board.D5)
		self.lcd_d5 = digitalio.DigitalInOut(board.D6)
		self.lcd_d6 = digitalio.DigitalInOut(board.D13)
		self.lcd_d7 = digitalio.DigitalInOut(board.D26)

		# Modify this if you have a different sized character LCD
		self.lcd_columns = 16
		self.lcd_rows = 2 

		# Initialise the lcd class
		self.lcd = characterlcd.Character_LCD_Mono(self.lcd_rs, self.lcd_en, 
					self.lcd_d4, self.lcd_d5, self.lcd_d6, self.lcd_d7, 
					self.lcd_columns, self.lcd_rows)

		# wipe LCD screen before we start
		self.lcd.clear()

	##
	## cleanupDisplay - Method used to cleanup the digitalIO lines that
	## are used to run the display.
	##
	def cleanupDisplay(self):
		# Clear the LCD first - otherwise we won't be able to update it.
		self.lcd.clear()
		self.lcd_rs.deinit()
		self.lcd_en.deinit()
		self.lcd_d4.deinit()
		self.lcd_d5.deinit()
		self.lcd_d6.deinit()
		self.lcd_d7.deinit()
		
	##
	## clear - Convenience method used to clear the display
	##
	def clear(self):
		self.lcd.clear()

	##
	## updateScreen - Convenience method used to update the message.
	##
	def updateScreen(self, message):
		self.lcd.clear()
		self.lcd.message = message

	## End class ManagedDisplay definition  

##
## Initialize our display
##
screen = ManagedDisplay()

##
## TemperatureMachine - This is our StateMachine implementation class.
## The purpose of this state machine is to manage the three states
## handled by our thermostat:
##
##  off
##  heat
##  cool
##
##
class TemperatureMachine(StateMachine):
	"A state machine designed to manage our thermostat"

	##
	## Define the three states for our machine.
	##
	off = State(initial = True)   # The off state, when the thermostat is off
	heat = State()                # The heat state, when the thermostat is heating
	cool = State()                # The cool state, when the thermostat is cooling

	##
	## Default temperature setPoint is 72 degrees Fahrenheit
	##
	setPoint = 72

	##
	## cycle - event that provides the state machine behavior
	## of transitioning between the three states of our thermostat
	##
	cycle = (
		heat.to(cool) |  # Transition from heat to cool
		cool.to(off)     # Transition from cool to off
	)

	def on_enter_heat(self):
		# When entering the "heat" state, update the LED light and print debug
		self.update_lights("heat")

		if(DEBUG):
			print("* Changing state to heat")

	def on_exit_heat(self):
		# When exiting the "heat" state, turn off all lights
		self.update_lights("off")
		 
	def on_enter_cool(self):
		# When entering the "cool" state, update the LED light and print debug
		self.update_lights("cool")

		if(DEBUG):
			print("* Changing state to cool")
	
	def on_exit_cool(self):
		# When exiting the "cool" state, turn off all lights
		self.update_lights("off")
		
	def on_enter_off(self):
		# When entering the "off" state, turn off all lights and reset indicators
		self.update_lights("off")
		self.turn_off_all_indicators()

		if(DEBUG):
			print("* Changing state to off")
	
	def processTempStateButton(self):
		# Method to cycle through temperature states (off, heat, cool)
		if(DEBUG):
			print("Cycling Temperature State")

		self.machine.next_state()  # Transition to next state

	def processTempIncButton(self):
		# Method to increase the set temperature by one degree
		if(DEBUG):
			print("Increasing Set Point")

		self.setPoint += 1  # Increase set point
		self.update_status_lights()

	def processTempDecButton(self):
		# Method to decrease the set temperature by one degree
		if(DEBUG):
			print("Decreasing Set Point")

		self.setPoint -= 1  # Decrease set point
		self.update_status_lights()

	def updateLights(self):
		# Method to update the LED lights based on the current temperature and state
		temp = floor(self.getFahrenheit())  # Get the current temperature in Fahrenheit
		redLight.off()
		blueLight.off()
	
		## Verify values for debug purposes
		if(DEBUG):
			print(f"State: {self.current_state.id}")
			print(f"SetPoint: {self.setPoint}")
			print(f"Temp: {temp}")

		## Determine visual identifiers

		# Turn off all lights first
		self.set_light("heating", False)
		self.set_light("cooling", False)
		self.set_light("idle", False)

		# Check for heating state
		if self.state == "heat":
			if self.currentTemp < self.setPoint:
				self.set_light("heating", True)  # Turn on heating light if temp is below setpoint
			else:
				self.set_light("idle", True)  # Turn on idle light if temp is at or above setpoint

		# Check for cooling state
		elif self.state == "cool":
			if self.currentTemp > self.setPoint:
				self.set_light("cooling", True)  # Turn on cooling light if temp is above setpoint
			else:
				self.set_light("idle", True)  # Turn on idle light if temp is at or below setpoint

		# Turn on idle light when thermostat is off
		elif self.state == "off":
			self.set_light("idle", True)  # Idle light on during off state

	##
	## run - kickoff the display management functionality of the thermostat
	##
	def run(self):
		myThread = Thread(target=self.manageMyDisplay)  # Start a new thread to handle display updates
		myThread.start()

	##
	## Get the temperature in Fahrenheit
	##
	def getFahrenheit(self):
		# Convert the temperature from Celsius to Fahrenheit
		t = thSensor.temperature
		return (((9/5) * t) + 32)

	##
	## Configure output string for the Thermostat Server
	##
	def setupSerialOutput(self):
		# Prepare the data to send to the server in a CSV format
		output = f"{self.state},{self.currentTemp},{self.setPoint}"
		return output
	
	## Continue display output
	endDisplay = False

	##
	## This function is designed to manage the LCD Display
	##
	def manageMyDisplay(self):
		# Loop to handle display updates
		counter = 1
		altCounter = 1
		while not self.endDisplay:
			# Only display if the DEBUG flag is set
			if(DEBUG):
				print("Processing Display Info...")
	
			# Grab the current time        
			current_time = datetime.now()
	
			# Setup display line 1
			current_time = datetime.now().strftime("%m/%d/%Y %H:%M")
			self.update_lcd_line(1, current_time)
	
			# Setup Display Line 2
			if(altCounter < 6):
				current_temp_display = f"Temp: {self.currentTemp}F"
				self.update_lcd_line(2, current_temp_display)
				altCounter = altCounter + 1
			else:
				state_setpoint_display = f"{self.state} Set: {self.setPoint}F"
				self.update_lcd_line(2, state_setpoint_display)
	
			altCounter = altCounter + 1
			if(altCounter >= 11):
				# Run the routine to update the lights every 10 seconds
				# to keep operations smooth
				self.updateLights()
				altCounter = 1
	
			# Update Display
			screen.updateScreen(lcd_line_1 + lcd_line_2)
	
			# Update server every 30 seconds
			if(DEBUG):
				print(f"Counter: {counter}")
			
			if((counter % 30) == 0):
				output = self.setupSerialOutput() 
				self.serialPort.write(output.encode())
			   

				counter = 1
			else:
				counter = counter + 1
			sleep(1)

		## Cleanup display
		screen.cleanupDisplay()

tsm = TemperatureMachine()
tsm.run()

greenButton = Button(24)
greenButton.when_pressed = self.processTempStateButton  # Green button to cycle states

redButton = Button(25)
redButton.when_pressed = self.processTempIncButton  # Red button to increase temperature

blueButton = Button(12)
blueButton.when_pressed = self.processTempDecButton  # Blue button to decrease temperature

repeat = True

while repeat:
	try:
		# Wait for 30 seconds to maintain the loop
		sleep(30)

	except KeyboardInterrupt:
		# Catch the keyboard interrupt (CTRL-C) and exit cleanly
		print("Cleaning up. Exiting...")
		repeat = False
		
		# Close down the display
		tsm.endDisplay = True
		sleep(1)
