import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import json

class SemanticMapNode(Node):
    def __init__(self):
        super().__init__('semantic_map_node')

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/semantic_map/markers',
            10
        )

        # 🔑 타이머로 주기적 publish
        self.timer = self.create_timer(1.0, self.load_and_publish)

        self.get_logger().info('Semantic map node started')

    def load_and_publish(self):
        markers = MarkerArray()

        # 테스트용 마커 하나 (이거 안 뜨면 무조건 코드 문제)
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'test'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = 0.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.1
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color.r = 1.0
        m.color.a = 1.0

        markers.markers.append(m)

        self.marker_pub.publish(markers)
        self.get_logger().info('Published markers')


def main():
    rclpy.init()
    node = SemanticMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
