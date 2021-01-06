# Copyright 2020 Andy McEvoy.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import pytest

import rclpy
import rclpy.node

import urdf_to_dh.generate_dh as gdh


class TestGenerageDhParams(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print('\nsetUpClass')
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        print('\ntearDownClass')
        rclpy.shutdown()

    def setUp(self):
        print('\nsetUp')
        self.dh_param_node = gdh.GenerateDhParams()

    def tearDown(self):
        print('\ntearDown')

    def test_function(self):
        print('\ntest_function')
        self.dh_param_node.InitializeDhNode()
        assert 1 == 0


if __name__ == '__main__':
    unittest.main()
