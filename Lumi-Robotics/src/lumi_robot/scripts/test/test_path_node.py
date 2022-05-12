# coding: utf-8
"""
テストケースのNumbering ルール
TestXX_YYY
    - XX: 機能別 例: 00 - 初期化
    - YY: 正常・異常別
        - 正常: 001～
        - 異常: 101～
"""
# 標準ライブラリ
import os
import sys
import unittest

# 外部ライブラリ

# 内部ライブラリ
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), './../')))
from path_node import PathNode


def log_in_out(func):
    """
    関数開始、終了を表示する。
    Args:
        func ():
    Returns:
    """

    def decorated_func(*args, **kwargs):
        print("Start ", func.__name__)
        result = func(*args, **kwargs)
        print("End   ", func.__name__)
        return result

    return decorated_func


class TestPathNode(unittest.TestCase):

    # 初期処理
    @classmethod
    def setUpClass(cls):
        """
        begin all tests
        :return:
        """
        cls.obj = PathNode()

    @classmethod
    def tearDownClass(cls):
        pass

    def setUp(self):
        """Begin each test"""
        pass

    def tearDown(self):
        """Start each test"""
        pass

    def execute_test(self, test_function, test_param):
        """
        テストの実行関数
        :param test_function: テスト関数
        :param test_param: テストのパラメータ
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

            print('test errmsg: {}'.format(ex.exception))

            errmsg = 'exception.message : {}'.format(ex.exception)

            self.assertTrue(errmsg.find(returns) > -1)

            return None
        else:
            result = test_function(**params)

            if returns is not None:
                self.assertEqual(result[:-1], returns[:-1])

            return result

    # 初期化処理のテスト
    @log_in_out
    def test00_001_add_waypoint(self):
        test_function = self.obj.add_waypoint

        # テストのパラメータ
        test_params = (
            ###################################################################
            # 正常系のテスト
            ###################################################################
            {'raise': False,
             'params': {
                 'new_point': {'x': 0, 'y': 0, 'yaw': 90},
                 'file_path': 'test_waypoint.json'
             }},
        )

        # テストのパラメータずつの実行
        for test_param in test_params:
            self.execute_test(test_function, test_param)