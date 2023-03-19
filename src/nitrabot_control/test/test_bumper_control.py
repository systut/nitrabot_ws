"""
@brief Tester for BumperControl 
Test case numbering rules
TestXX_YYY
	- XX: function id, Example: 00 - first function
	- YY: 
		- Normal: 001～
		- Abnormal: 101～
"""
# Standard library
import os
import sys
import unittest

# External library

# Internal library
sys.path.insert(0, os.path.abspath(
	os.path.join(os.path.dirname(__file__), "./../")))
from src.bumper_control.bumper_control import BumperControl 


class TestBumperControl(unittest.TestCase):
	"""!
	"""
	@classmethod
	def setUpClass(cls):
		cls.obj = BumperControl()

	def execute_test(self, test_function, test_param):
		"""! Execute testing 
		:param test_function: 
		:param test_param: 
		:return: errmsg (unicode): error msg
		"""
		raise_flg = test_param['raise']

		params = test_param['params']

		if 'returns' in test_param:
			returns = test_param['returns']
		else:
			returns = None

		print_msg = 'func = {}; params = {}; returns = {}; raise = {}'

		print_msg = print_msg.format(test_function.__name__, params,
									 returns, raise_flg)
		print(print_msg)

		if raise_flg is True:
			with self.assertRaises(Exception) as ex:
				test_function(**params)

			errmsg = 'exception.message : {}'.format(ex.exception)
			print(errmsg)
			
			self.assertTrue(errmsg.find(returns) > -1)

			return None
		else:
			result = test_function(**params)

			if returns is not None:
				self.assertEqual(result[:-1], returns[:-1])

			return result

	def test00_001_initialize_system_parameter(self):
		"""! Test case of _initialize_system_parameter 
		"""
		test_function = self.obj._initialize_system_parameter

		test_params = (
			{
				"raise": False,
				"params": {
				}
			},
		)
		
		for test_param in test_params:
			result = self.execute_test(test_function, test_param)

	def test01_001_generate_backing_motion(self):
		"""! Test case of _generate_backing_motion 
		"""
		test_function = self.obj._generate_backing_motion

		test_params = (
			{
				"raise": False,
				"params": {
				}
			},
		)
		
		for test_param in test_params:
			result = self.execute_test(test_function, test_param)
