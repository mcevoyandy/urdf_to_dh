import rclpy
import rclpy.node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class MarkerPublisher(rclpy.node.Node):

    def __init__(self):
        super().__init__('urdf_to_dh_marker_pub')
        self.marker_id = 0
        self.marker_publisher = self.create_publisher(Marker, 'topic', 10)


    def publish_arrow(self, frame, origin, direction, color):
        # Make publish arrow. Start at xyz, then xyz + axis
        marker_msg = Marker()
        marker_msg.header.frame_id = frame
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.id = self.marker_id
        self.marker_id += 1
        marker_msg.type = Marker.ARROW
        marker_msg.action = Marker.ADD
        marker_msg.frame_locked = True
        marker_msg.color.r = color[0]
        marker_msg.color.g = color[1]
        marker_msg.color.b = color[2]
        marker_msg.color.a = color[3]
        marker_msg.scale.x = 0.01
        marker_msg.scale.y = 0.02
        start_pt = Point()
        start_pt.x = origin[0]
        start_pt.y = origin[1]
        start_pt.z = origin[2]
        marker_msg.points.append(start_pt)
        end_pt = Point()
        end_pt.x = origin[0] + direction[0] * 0.2
        end_pt.y = origin[1] + direction[1] * 0.2
        end_pt.z = origin[2] + direction[2] * 0.2
        marker_msg.points.append(end_pt)

        self.marker_publisher.publish(marker_msg)

    def publish_frame(self, frame_name, tf):
        self.publish_arrow(frame_name, tf[0:3, 3], tf[0:3, 0], [1.0, 0.0, 0.0, 0.5])
        self.publish_arrow(frame_name, tf[0:3, 3], tf[0:3, 1], [0.0, 1.0, 0.0, 0.5])
        self.publish_arrow(frame_name, tf[0:3, 3], tf[0:3, 2], [0.0, 0.0, 1.0, 0.5])
