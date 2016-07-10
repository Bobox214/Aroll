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

class PWM(object):
	_pinNameMap = {
		'P9_14' : ( 1 , 0 , '/sys/devices/platform/ocp/','48302000','48302200')
	,	'P9_16' : ( 1 , 1 , '/sys/devices/platform/ocp/','48302000','48302200')
	,	'P8_19' : ( 2 , 0 , '/sys/devices/platform/ocp/','48304000','48304200')
	,	'P8_13' : ( 2 , 1 , '/sys/devices/platform/ocp/','48304000','48304200')
	}
	def __init__(self,pinName,frequency,dutyCycle=0,enable=True):
		if pinName not in self._pinNameMap:
			raise ValueError("Pin name %s is not supported for doing PWM"%pinName)
		self._exported = False
		self.pinName = pinName
		self.pwmNb,self.pinNb,ocpPath,addr1,addr2 = self._pinNameMap[pinName]
		# Handle overlay
		overlayName = "BB-PWM%d"%self.pwmNb
		with open('/sys/devices/platform/bone_capemgr/slots','r') as f:
			needOverlay = overlayName not in f.read()
		if needOverlay:
			try:
				with open('/sys/devices/platform/bone_capemgr/slots','w') as f:
					f.write(overlayName)
			except IOError:
				raise IOError("Overlay BB-PWM%d, provided by https://github.com/beagleboard/bb.org-overlays, required by this library to use pin %s as PWM, cannot be added."%(self.pwmNb,self.pinName))
		# Check the pwmPath exists, allow to wait if overlay was just added by another PWM object.
		c = 0
		while True:
			lsL = os.listdir(ocpPath)
			addr1L = [x for x in lsL if x.startswith(addr1)]
			if len(addr1L) == 1:
				break
			sleep(0.5)
			c += 1
			if c==10:
				break
		if c==10:
			raise IOError("Overlay BB-PWM%d, provided by https://github.com/beagleboard/bb.org-overlays, required by this library to use pin %s as PWM, cannot be loaded."%(self.pwmNb,self.pinName))
		addr1Path = os.path.join(ocpPath,addr1L[0])
		c = 0
		while True:
			lsL = os.listdir(addr1Path)
			addr2L = [x for x in lsL if x.startswith(addr2)]
			if len(addr2L) == 1:
				break
			sleep(0.5)
			c += 1
			if c==10:
				break
		if c==10:
			raise IOError("Overlay BB-PWM%d, provided by https://github.com/beagleboard/bb.org-overlays, required by this library to use pin %s as PWM, cannot be loaded."%(self.pwmNb,self.pinName))
		self.pwmPath = os.path.join(addr1Path,addr2L[0],'pwm')
		#Find the generated pwnChipX dir
		lsL = os.listdir(self.pwmPath)
		if len(lsL)!=1:
			raise IOError("Unexpected content in directory %s, expected one and only one directory after overlay loading",self.pwmPath)
		self.pwmChipDir = os.path.join(self.pwmPath,lsL[0])
		self.pwmDir     = "%s/pwm%d"%(self.pwmChipDir,self.pinNb)
		self.export()
		self.setFrequency(frequency)
		self.setDutyCycle(dutyCycle)
		if enable:
			self.enable()

	def enable(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/enable'%self.pwmDir,'w') as f:
			f.write('1')

	def disable(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/enable'%self.pwmDir,'w') as f:
			f.write('0')

	def isEnabled(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/enable'%self.pwmDir,'r') as f:
			return  bool(f.read())

	def setFrequency(self,frequency):
		""" Change the frequency of the PWM for this pin. This always reset dutyCyle to 0 """
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		nsPeriod = int(1.0e9/frequency)
		# Check other pin of the chip
		otherPinNb = 0 if self.pinNb == 1 else 1
		try:
			with open('%s/pwm%d/period'%(self.pwmChipDir,otherPinNb),'r') as f:
				otherNsPeriod = int(f.read())
		except IOError:
			otherNsPeriod = None
		if 0 and otherNsPeriod is not None and otherNsPeriod != nsPeriod:
			raise ValueError("Pins using the same PWM chip must have the same frequency. PWM %d for pin %s is already configured with a frequency of %g Hz"%(
				self.pwmNb,self.pinName,1e9/otherNsPeriod
			))
		# Disable dutyCycle, this can fail is period is 0
		try:
			with open('%d/duty_cycle','w') as f:
				f.write('0')
		except IOError: pass
		# Set period
		try:
			with open('%s/period'%self.pwmDir,'w') as f:
				f.write(str(nsPeriod))
		except IOError:
			raise IOError("Frequency cannot be set to %f Hz for pin %s. The other pin on the same chip is active and with another frequency"%(frequency,self.pinName))
	
	def getFrequency(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/period'%self.pwmDir,'r') as f:
			nsPeriod = int(f.read())
		return 1e9/nsPeriod
		

	def setDutyCycle(self,dutyCycle):
		if not (isinstance(dutyCycle,int) and dutyCycle>=0 and dutyCycle<=100):
			raise ValueError("duty_cyle : Expects an integer between 0 and 100 not <%s>:%s"%(type(dutyCycle),dutyCycle))
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/period'%self.pwmDir,'r') as f:
			nsPeriod = int(f.read())
		nsDutyCycle = int(nsPeriod*dutyCycle/100)
		with open('%s/duty_cycle'%self.pwmDir,'w') as f:
			f.write(str(nsDutyCycle))
	
	def getDutyCycle(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		with open('%s/period'%self.pwmDir,'r') as f:
			nsPeriod = int(f.read())
		with open('%s/duty_cycle'%self.pwmDir,'r') as f:
			nsDuty   = int(f.read())
		return int(100*nsDuty/nsPeriod)

	def export(self):
		if self._exported:
			raise ValueError('Pin %s has already been exported'%self.pinName)
		try:
			with open('%s/export'%self.pwmChipDir,'w') as f:
				f.write(str(self.pinNb))
		except IOError: pass
		with open('%s/period'%self.pwmDir,'r') as f:
			nsPeriod = int(f.read())
		self._exported = True

	def unexport(self):
		if not self._exported:
			raise ValueError('Pin %s is not exported'%self.pinName)
		if self.isEnabled():
			self.disable()
		with open('%s/unexport'%self.pwmChipDir,'w') as f:
			f.write(str(self.pinNb))
		self._exported = False

	def isExported(self):
		return self._exported

	def __del__(self):
		if self._exported:
			self.unexport()
