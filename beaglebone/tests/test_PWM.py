import unittest
import context

from beaglebone.io import PWM

class TestPWM(unittest.TestCase):
	
	def test_P9_14(self):
		pwm = PWM('P9_14',20000,10)
		self.assertTrue(pwm.isExported())
		self.assertTrue(pwm.isEnabled())
		self.assertEqual(pwm.getFrequency(),20000)
		self.assertEqual(pwm.getDutyCycle(),10)

if __name__=='__main__':
	unittest.main()
