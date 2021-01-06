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

import rclpy
import rclpy.node
# from rclpy.exceptions import ParameterAlreadyDeclaredException
# from rcl_interfaces.msg import ParameterType


class GenerateDhParams(rclpy.node.Node):

    def __init__(self):
        super().__init__('generate_dh_param_node')
        self.declare_parameter('urdf_file')

    def InitializeDhNode(self):
        self.get_logger().info('Initializing...')
        urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value
        self.get_logger().info('URDF file = %s' % urdf_file)


def main():
    print('Starting GenerateDhParams Node...')
    rclpy.init()
    node = GenerateDhParams()
    node.InitializeDhNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
