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

from __future__ import annotations

import math
import pprint
import sys

import rclpy
import rclpy.node

import anytree
from anytree import AnyNode
from anytree import LevelOrderIter
from anytree import RenderTree

import numpy as np

import pandas as pd

from . import kinematics_helpers as kh
from . import geometry_helpers as gh
from . import urdf_helpers as uh
from . import maker_helpers as mh


class GenerateDhParams(rclpy.node.Node):

    def __init__(self):
        super().__init__('generate_dh_param')

        self.declare_parameter('urdf_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('verbose', False)
        # The map {joint_name: joint_info}, where joint_info is a map with keys
        # 'axis', 'xyz', 'rpy', 'parent', 'child', 'dh', 'type'.
        self.urdf_joints: dict[str, dict] = {}
        self.urdf_links: dict[str, dict] = {}
        self.urdf_file = ''
        self.urdf_tree_nodes: list[AnyNode] = []
        self.root_node: AnyNode | None = None
        # The simplified tree is the tree where all fixed joints not required
        # for the kinematics are removed. It contains only links.
        self.simplified_tree_root: AnyNode | None = None
        self.verbose = False
        self.marker_pub = mh.MarkerPublisher()

    def initialize_dh_node(self) -> None:
        self.get_logger().info('Initializing...')

        self.urdf_file = self.get_parameter('urdf_file').get_parameter_value().string_value
        self.get_logger().info(f'URDF file = {self.urdf_file}')

        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

    def parse_urdf(self) -> None:
        # Get the root of the URDF and extract all of the joints
        urdf_root = uh.get_urdf_root(self.urdf_file)

        # Parse all links first and add to tree
        for child in urdf_root:
            if child.tag == 'link':
                self.urdf_links[child.get('name')] = {
                        'rel_tf': np.eye(4),
                        'abs_tf': np.eye(4),
                        'dh_tf': np.eye(4),
                        'abs_dh_tf': np.eye(4),
                        'dh_found': False,
                }
                node = AnyNode(id=child.get('name'), parent=None, children=None, type='link')
                self.urdf_tree_nodes.append(node)

        # Parse all joints and add to tree
        for child in urdf_root:
            if child.tag == 'joint':
                joint_name, joint_data = uh.process_joint(child)
                self.urdf_joints[joint_name] = joint_data
                node = AnyNode(id=joint_name, parent=None, children=None, type='joint')

                # Find parent and child link
                for n in self.urdf_tree_nodes:
                    if n.id == joint_data['parent']:
                        node.parent = n
                    if n.id == joint_data['child']:
                        n.parent = node
                self.urdf_tree_nodes.append(node)

        # Find root link
        num_nodes_no_parent = 0
        for n in self.urdf_tree_nodes:
            if n.parent is None:
                num_nodes_no_parent += 1
                self.root_node = n

        if num_nodes_no_parent != 1:
            print('Error: Should only be one root link', file=sys.stderr)
            return

        # Root link DH will be identity, set dh_found = True
        # TODO: Probably not needed since order iter is used
        self.urdf_links[self.root_node.id]['dh_found'] = True
        print('URDF Tree:')
        for pre, _, node in RenderTree(self.root_node):
            joint_type_str = '' if node.type == 'link' else f' ({self.urdf_joints[node.id]["type"]})'
            print(f'{pre}{node.id}{joint_type_str}')

        # Construct the simplified tree, without fixed joints
        # except for leaf links.

        root_node = AnyNode(
                id=self.root_node.id,
                parent=None,
                children=None,
        )
        self.simplified_tree_root = root_node

        leafs = anytree.findall(self.root_node, filter_=lambda n: n.is_leaf)
        for leaf in leafs:
            r, *path = leaf.path
            last_added_link = root_node
            for n in path:
                if n.type == 'joint':
                    continue
                simplified_node = anytree.find(
                        root_node,
                        filter_=lambda node: node.id == n.id,
                        )
                if simplified_node:
                    # Already in the tree.
                    last_added_link = simplified_node
                    continue
                if ((self.urdf_joints[n.parent.id]['type'] != 'fixed')
                        or n.is_leaf
                ):
                    node = AnyNode(
                            id=n.id,
                            parent=last_added_link,
                            children=None,
                    )
                    last_added_link = node

        print('\nSimplified URDF Tree (without irrelevant joints for the kinematics):')
        for pre, _, node in RenderTree(self.simplified_tree_root):
            print(f'{pre}{node.id}')

        if self.verbose:
            print('Joint Info:')
            pprint.pprint(self.urdf_joints)

    def calculate_tfs_in_world_frame(self) -> None:
        self._print('Calculate world tfs:')
        for n in LevelOrderIter(self.root_node):
            if (n.type == 'link') and (n.parent is not None):
                self._print(f'\nTransform from "{n.parent.parent.id}" to "{n.id}":')
                parent_tf_world = self.urdf_links[n.parent.parent.id]['abs_tf']
                xyz = self.urdf_joints[n.parent.id]['xyz']
                rpy = self.urdf_joints[n.parent.id]['rpy']
                tf = np.eye(4)
                tf[0:3, 0:3] = kh.get_extrinsic_rotation(rpy)
                tf[0:3, 3] = xyz
                self.urdf_links[n.id]['rel_tf'] = tf

                abs_tf = parent_tf_world @ tf
                self.urdf_links[n.id]['abs_tf'] = abs_tf

                self._print(f'relative {tf.flatten().tolist()}')
                self._print(f'absolute {abs_tf.flatten().tolist()}')

    def calculate_dh_params(self) -> None:
        self._print('calculate_dh_params')
        # Node process order:
        self._print('process_order =\n{}'.format([urdf_node.id for urdf_node in LevelOrderIter(self.root_node)]))
        # List of ['joint', 'parent', 'child', 'd', 'theta', 'r', 'alpha']
        robot_dh_params = []

        for urdf_node in LevelOrderIter(self.root_node):
            if urdf_node.type == 'link' and (not self.urdf_links[urdf_node.id]['dh_found']):
                self._print(f'\n\nprocess dh params for {urdf_node.id}')

                # Transform from current link frame to world frame
                link_to_world = self.urdf_links[urdf_node.id]['abs_tf']

                # DH frame from parent link frame to world frame
                parent_to_world_dh = self.urdf_links[urdf_node.parent.parent.id]['abs_dh_tf']

                # Transform from link frame to parent DH frame
                link_to_parent_dh = kh.inv_tf(parent_to_world_dh) @ link_to_world

                # Find DH parameters
                # Publish Joint axis for visual verification
                self.marker_pub.publish_arrow(
                        urdf_node.id,
                        np.zeros(3),
                        self.urdf_joints[urdf_node.parent.id]['axis'],
                        [1.0, 0.0, 1.0, 0.2],
                )
                axis = link_to_parent_dh[0:3, 0:3] @ self.urdf_joints[urdf_node.parent.id]['axis']

                dh_params = self._get_joint_dh_params(link_to_parent_dh, axis)

                dh_frame = kh.get_dh_frame(dh_params)
                abs_dh_frame = parent_to_world_dh @ dh_frame

                self.urdf_links[urdf_node.id]['dh_tf'] = dh_frame

                self.urdf_links[urdf_node.id]['abs_dh_tf'] = abs_dh_frame
                self.marker_pub.publish_frame('world', abs_dh_frame)
                robot_dh_params.append([urdf_node.parent.id, urdf_node.parent.parent.id, urdf_node.id] + list(dh_params))

        pd_frame = pd.DataFrame(robot_dh_params, columns=['joint', 'parent', 'child', 'd', 'theta', 'r', 'alpha'])
        pd_frame['r'] = pd_frame['r'].round(6)
        pd_frame['d'] = pd_frame['d'].round(6)
        pd_frame['theta'] = np.degrees(pd_frame['theta']).round(5)
        pd_frame['alpha'] = np.degrees(pd_frame['alpha']).round(5)
        print('\nDH Parameters: (csv)')
        print(pd_frame.to_csv())
        print('\nDH Parameters: (markdown)')
        print(pd_frame.to_markdown())

    def _get_joint_dh_params(self, rel_link_frame, axis) -> np.ndarray:
        dh_params = np.zeros(4)
        origin_xyz = rel_link_frame[0:3, 3]
        z_axis = np.array([0., 0., 1.])
        self._print(axis)

        if gh.are_collinear(np.zeros(3), z_axis, origin_xyz, axis):
            # Collinear case
            self._print('  Process collinear case.')
            dh_params = self._process_collinear_case(origin_xyz, rel_link_frame[0:3, 0])
        elif gh.are_parallel(z_axis, axis):
            # Parallel case
            self._print('  Process parallel case.')
            dh_params = self._process_parallel_case(origin_xyz)
        elif gh.lines_intersect(np.zeros(3), z_axis, origin_xyz, axis)[0]:
            # Intersect case
            self._print('  Process intersection case.')
            self._print(rel_link_frame)
            dh_params = self._process_intersection_case(origin_xyz, axis)
        else:
            # Skew case
            self._print('  Process skew case.')
            dh_params = self._process_skew_case(origin_xyz, axis)

        # Visualize the "d" component
        # self.publish_arrow(joint_data['parent'], np.zeros(3), pointA, 0.0, 0.0, 1.0, 0.5)

        # # Visualize the "r" component
        # self.publish_arrow(joint_data['parent'], pointA, pointB-pointA, 1.0, 0.0, 1.0, 1.0)

        # # Visualize the intersection and alignment with the next joint axis
        # self.publish_arrow(joint_data['parent'], pointB, joint_data['xyz']-pointB, 0.0, 1.0, 1.0, 0.5)
        self._print(dh_params)
        return dh_params

    def _process_collinear_case(self, origin, xaxis) -> np.ndarray:
        dh_params = np.zeros(4)
        dh_params[0] = origin[2]
        return dh_params

    def _process_parallel_case(self, origin) -> np.ndarray:
        dh_params = np.zeros(4)
        dh_params[0] = origin[2]
        dh_params[1] = math.atan2(origin[1], origin[0])
        dh_params[2] = math.sqrt(origin[0]**2 + origin[1]**2)
        return dh_params

    def _process_intersection_case(self, origin, axis) -> np.ndarray:
        dh_params = np.zeros(4)
        dh_params[0] = gh.lines_intersect(np.zeros(3), np.array([0, 0, 1]), origin, axis)[1][0]

        zaxis = np.array([0., 0., 1.])

        for i in range(3):
            if abs(axis[i]) < 1.e-5:
                axis[i] = 0.0

        cn = np.cross(zaxis, axis)
        for i in range(3):
            if abs(cn[i]) < 1.e-6:
                cn[i] = 0.0
        if (cn[0] < 0.0):
            cn = -cn
        dh_params[1] = math.atan2(cn[1], cn[0])

        dh_params[2] = 0.0

        vn = cn / np.linalg.norm(cn)
        dh_params[3] = math.atan2(np.dot(np.cross(zaxis, axis), vn), np.dot(zaxis, axis))

        return dh_params

    def _print(self, *args) -> None:
        if self.verbose:
            print(*args)

    def _process_skew_case(self, origin, direction) -> np.ndarray:
        pointA = np.zeros(3)
        dh_params = np.zeros(4)

        # Find closest points along parent z-axis (pointA) and joint axis (pointB)
        t = -(origin[0] * direction[0] + origin[1] * direction[1]) / (direction[0]**2 + direction[1]**2)
        pointB = origin + t * direction
        pointA[2] = pointB[2]

        # 'd' is offset along parent z axis
        dh_params[0] = pointA[2]

        # 'r' is the length of the common normal
        dh_params[2] = np.linalg.norm(pointB - pointA)

        # 'theta' is the angle between the x-axis and the common normal
        dh_params[1] = math.atan2(pointB[1], pointB[0])

        # 'alpha' is the angle between the current z-axis and the joint axis
        # Awesome way to get signed angle:
        # https://stackoverflow.com/questions/5188561/signed-angle-between-two-3d-vectors-with-same-origin-within-the-same-plane/33920320#33920320
        cn = pointB - pointA
        vn = cn / np.linalg.norm(cn)
        zaxis = np.array([0., 0., 1.])
        dh_params[3] = math.atan2(np.dot(np.cross(zaxis, direction), vn), np.dot(zaxis, direction))

        return dh_params


def main():
    print('Starting GenerateDhParams Node...')
    rclpy.init()
    node = GenerateDhParams()
    node.initialize_dh_node()
    node.parse_urdf()
    node.calculate_tfs_in_world_frame()
    node.calculate_dh_params()

    try:
        rclpy.spin(node)
    except:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
