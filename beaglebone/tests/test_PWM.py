import unittest
import context

from beaglebone.io import PWM

class TestPWM(unittest.TestCase):
	def clean(self):
		for chipNb in [0]:
			for pinNb in [0,1]:
				try:
					with open('/sys/class/pwm/pwmchip%d/unexport'%chipNb,'w') as f:
						f.write(str(pinNb))
				except IOError: pass
		
	def test_P9_14(self):
		self.clean()
		pwm = PWM('P9_14',20000,10)
		self.assertTrue(pwm.isExported())
		self.assertTrue(pwm.isEnabled())
		self.assertEqual(pwm.getFrequency(),20000)
		self.assertEqual(pwm.getDutyCycle(),10)

	def test_P9_16(self):
		self.clean()
		pwm = PWM('P9_16',20000,10)
		self.assertTrue(pwm.isExported())
		self.assertTrue(pwm.isEnabled())
		self.assertEqual(pwm.getFrequency(),20000)
		self.assertEqual(pwm.getDutyCycle(),10)

	def test_P8_19(self):
		self.clean()
		pwm = PWM('P8_19',200,0)
		self.assertTrue(pwm.isExported())
		self.assertTrue(pwm.isEnabled())
		self.assertEqual(pwm.getFrequency(),200)
		self.assertEqual(pwm.getDutyCycle(),0)

	def test_P8_13(self):
		self.clean()
		pwm = PWM('P8_13',200,100)
		self.assertTrue(pwm.isExported())
		self.assertTrue(pwm.isEnabled())
		self.assertEqual(pwm.getFrequency(),200)
		self.assertEqual(pwm.getDutyCycle(),100)

if __name__=='__main__':
	unittest.main()
