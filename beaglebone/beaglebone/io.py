from Adafruit_BBIO import GPIO
from time import sleep,time
import os

class UsrLEDs(object):
	def __init__(self):
		""" Ensure LEDs GPIOs are correctly configured """
		GPIO.setup("USR0",GPIO.OUT)
		GPIO.setup("USR1",GPIO.OUT)
		GPIO.setup("USR2",GPIO.OUT)
		GPIO.setup("USR3",GPIO.OUT)

	def write(self,val):
		""" Write a value between 0 and 15 to the 4 LEDs, considering them as bits """
		assert val>=0 and val<16
		GPIO.output("USR0",(val>>0)%2)
		GPIO.output("USR1",(val>>1)%2)
		GPIO.output("USR2",(val>>2)%2)
		GPIO.output("USR3",(val>>3)%2)

	def countdown(self):
		""" Do a 4 step countdown this the LED, each step is spaced by 1s """
		self.write(15)
		sleep(1)
		self.write(7)
		sleep(1)
		self.write(3)
		sleep(1)
		self.write(1)
		sleep(1)
		self.write(0)

	def reset(self):
		""" Reset the LEDs to their boot state, namely heartbeat/microSD/cpu/eMMC """
		for i,trigger in enumerate(['heartbeat','mmc0','cpu0','mmc1']):
			with open("/sys/class/leds/beaglebone:green:usr%s/trigger"%i,'w') as f:
				f.write(trigger)

usrLEDs = UsrLEDs()

def waitSwitchStart(switchPin):
	"""
		A Switch is connected to the provided pin.
		While waiting for the switch to be HIGH, USR LED are
		doing a one hot pattern.
		When the Switch is activated, a countdown is started.
	"""
	print("")
	print("[I] Waiting starting switch on %s"%switchPin)
	GPIO.setup(switchPin,GPIO.IN)

	t = time()
	while True:
		if GPIO.input(switchPin):
			print("[I] Countdown started")
			usrLEDs.countdown()
			break
		sleep(0.1)
		usrLEDs.write(1<<(int(time()-t)%4))

	print("[I] Go Go Go !")

# Fake behaviour of PWM module while Adafruit_BBIO is updated for 4.4
class PWM:
	pinStarted = set()
	pinNameMap = {
		'P9_14' : ( 0 , 0 )
	,	'P9_16' : ( 0 , 1 )
	}
	@staticmethod
	def start(pinName,frequency,duty_cycle):
		chipNb,subNb = PWM.pinNameMap[pinName]
		if not os.path.exists('/sys/class/pwm/pwmchip%s/pwm%s'%(chipNb,subNb)):
			if os.system("echo %s > /sys/class/pwm/pwmchip%s/export"%(subNb,chipNb)) != 0:
				raise IOError("Can't start pinName %s chip %s sub %s"%(pinName,subNb,chipNb))
		with open('/sys/class/pwm/pwmchip%s/pwm%s/enable'%(chipNb,subNb),'w') as f:
			f.write(str(1))
		PWM.pinStarted.add(pinName)
		nsPeriod = int(1.0e9/frequency)
		nsDuty   = int(nsPeriod*duty_cycle/100)
		try:
			with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'w') as f:
				f.write('0')
		except IOError:
			pass
		with open('/sys/class/pwm/pwmchip%s/pwm%s/period'%(chipNb,subNb),'w') as f:
			f.write(str(nsPeriod))
		with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'w') as f:
			f.write(str(nsDuty))

	@staticmethod
	def stop(pinName):
		chipNb,subNb = PWM.pinNameMap[pinName]
		with open('/sys/class/pwm/pwmchip%s/pwm%s/enable'%(chipNb,subNb),'w') as f:
			f.write(str(0))
		os.system("echo %s > /sys/class/pwm/pwmchip%s/unexport"%(subNb,chipNb))
		PWM.pinStarted.discard(pinName)

	@staticmethod
	def set_duty_cycle(pinName,dutyCycle):
		if not (isinstance(dutyCycle,int) and dutyCycle>=0 and dutyCycle<=100):
			raise ValueError("duty_cyle : Expects an integer between 0 and 100 not <%s>:%s"%(type(dutyCycle),dutyCycle))
		chipNb,subNb = PWM.pinNameMap[pinName]
		with open('/sys/class/pwm/pwmchip%s/pwm%s/period'%(chipNb,subNb),'r') as f:
			nsPeriod = int(f.read())
		nsDutyCycle = int(nsPeriod*dutyCycle/100)
		if nsPeriod != 0:
			try:
				with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'w') as f:
					f.write(str(nsDutyCycle))
			except:
				print '[E] Could not set pin %s to duty_cycle (%s)'%(pinName,dutyCycle)

	@staticmethod
	def set_frequency(pinName,frequency):
		nsPeriod = int(1.0e9/frequency)
		chipNb,subNb = PWM.pinNameMap[pinName]
		with open('/sys/class/pwm/pwmchip%s/pwm%s/enable'%(chipNb,subNb),'w') as f:
			f.write(str(0))
		with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'r') as f:
			curNsDuty = int(f.read())
		with open('/sys/class/pwm/pwmchip%s/pwm%s/period'%(chipNb,subNb),'r') as f:
			curNsPeriod = int(f.read())
		with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'w') as f:
			f.write('0')
		with open('/sys/class/pwm/pwmchip%s/pwm%s/period'%(chipNb,subNb),'w') as f:
			f.write(str(nsPeriod))
		with open('/sys/class/pwm/pwmchip%s/pwm%s/duty_cycle'%(chipNb,subNb),'w') as f:
			f.write(str(int(nsPeriod*curNsDuty/curNsPeriod)))
		with open('/sys/class/pwm/pwmchip%s/pwm%s/enable'%(chipNb,subNb),'w') as f:
			f.write(str(1))
	@staticmethod
	def cleanup():
		for pinName in list(PWM.pinStarted):
			PWM.stop(pinName)
