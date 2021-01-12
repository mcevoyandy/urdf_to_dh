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
from ament_index_python.packages import get_package_share_directory

import xml.etree.ElementTree as ET
import numpy as np
import os
import pprint



class GenerateDhParams(rclpy.node.Node):

    def __init__(self):
        super().__init__('generate_dh_param_node')
        self.declare_parameter('urdf_file')
        self.urdf_joints = {}
        self.urdf_file = ''

    def InitializeDhNode(self):
        self.get_logger().info('Initializing...')

        self.urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value
        self.get_logger().info('URDF file = %s' % self.urdf_file)

    def calculate_dh_params(self):

        # Get the root of the URDF and extract all of the joints
        urdf_root = self.get_urdf_root()
        for child in urdf_root:
            if child.tag == 'joint':
                self.process_joint(child)

        # For each joint, compute location of child w.r.t the parent's frame
        # Get the joint axis in the parent frame


        pprint.pprint(self.urdf_joints)


    def get_urdf_root(self):
        """Parse a URDF for joints.

        Args:
            urdf_path (string): The absolute path to the URDF to be analyzed.

        Returns:
            root (xml object): root node of the URDF.
        """
        try:
            tree = ET.parse(self.urdf_file)
        except ET.ParseError:
            logging.error('Could not parse urdf file.')

        return tree.getroot()

    def process_joint(self, joint):
        """Extracts the relevant joint info into a dictionary.
        Args:
        Returns:
        """
        axis = np.array([1, 0, 0])
        xyz = np.zeros(3)
        rpy = np.zeros(3)

        joint_name = joint.get('name')

        for child in joint:
            if child.tag == 'axis':
                axis = np.array(child.get('xyz').split(), dtype=float)
            elif child.tag == 'origin':
                xyz = np.array(child.get('xyz').split(), dtype=float)
                rpy = np.array(child.get('rpy').split(), dtype=float)

        self.urdf_joints[joint_name] = {'axis': axis, 'xyz': xyz, 'rpy': rpy}

def main():
    print('Starting GenerateDhParams Node...')
    rclpy.init()
    node = GenerateDhParams()
    node.InitializeDhNode()

    node.calculate_dh_params()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
