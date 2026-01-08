import json
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

GRID_SIZE = 0.2 # 그리드 셀의 크기 (미터 단위)

class SemanticViz(Node):
    def __init__(self):
        super().__init__('semantic_viz')

        self.pub = self.create_publisher(
            MarkerArray,
            'semantic_map',
            10
        )

        self.timer = self.create_timer(1.0, self.publish_markers)

        with open('semantic_grid_map.json') as f:
            self.data = json.load(f)['grid']

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for key, cell in self.data.items():
            gx, gy = map(int, key.split(','))

            x = gx * GRID_SIZE
            y = gy * GRID_SIZE

            # ===== PLACE =====
            for place in cell.get('places', []):
                m = Marker()
                m.header.frame_id = 'map'
                m.id = marker_id
                marker_id += 1

                m.type = Marker.CUBE
                m.action = Marker.ADD

                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.05

                m.scale.x = GRID_SIZE
                m.scale.y = GRID_SIZE
                m.scale.z = 0.1

                # 색상 조건
                if place.get('symbolic', {}).get('category') == 'path':
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8
                else:
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 0.6

                marker_array.markers.append(m)

            # ===== OBJECT =====
            for obj in cell.get('objects', []):
                m = Marker()
                m.header.frame_id = 'map'
                m.id = marker_id
                marker_id += 1

                m.type = Marker.CUBE
                m.action = Marker.ADD

                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = 0.2

                m.scale.x = GRID_SIZE * 0.5
                m.scale.y = GRID_SIZE * 0.5
                m.scale.z = 0.3

                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.8

                marker_array.markers.append(m)

        self.pub.publish(marker_array)


def main():
    rclpy.init()
    node = SemanticViz()
    rclpy.spin(node)
    rclpy.shutdown()
