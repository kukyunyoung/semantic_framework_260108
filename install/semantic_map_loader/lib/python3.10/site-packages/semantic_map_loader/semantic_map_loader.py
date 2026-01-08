import json
import rclpy
import random
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Point

class SemanticMapLoader(Node):
    def __init__(self):
        super().__init__('semantic_map_loader')

        with open('/home/n/semantic_map_ui/public/semantic_grid_map.json') as f:
            data = json.load(f)


        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/semantic_map/markers',
            1
        )


        self.timer = self.create_timer(1.0, lambda: self.publish_map(data))
    
    def color_from_id(self, pid):
        random.seed(pid)   # id마다 항상 같은 색
        return (
            random.random(),
            random.random(),
            random.random()
        )

    def publish_map(self, data):
        grid = data['grid']
        meta = data['meta']

        # ===== OccupancyGrid =====
        og = OccupancyGrid()
        og.header.frame_id = 'map'
        og.info.resolution = meta['spacing']
        og.info.width = meta['cols']
        og.info.height = meta['rows']

        og.info.origin = Pose()
        og.info.origin.position.x = meta['leftTop']['x']
        og.info.origin.position.y = (
            meta['leftTop']['y'] - meta['rows'] * meta['spacing']
        )
        og.info.origin.orientation.w = 1.0

        og.data = []

        for y in reversed(range(meta['rows'])):
            for x in range(meta['cols']):
                cell = grid.get(f"{x},{y}", {})
                value = 0

                if cell.get('places'):
                    pid = cell['places'][0]['id']
                    if pid == 'wall':
                        value = 100
                    elif pid == 'path':
                        value = 0

                og.data.append(value)

        self.map_pub.publish(og)

        # ===== MarkerArray (place centers) =====
        markers = MarkerArray()
        mid = 0

        place_markers = {}
        mid = 0
        
        # ===== PLACE =====
        for key, cell in grid.items():
            gx, gy = map(int, key.split(','))

            wx = meta['leftTop']['x'] + gx * meta['spacing']
            wy = meta['leftTop']['y'] - gy * meta['spacing']

            for place in cell.get('places', []):
                pid = place.get('id')
                if pid in ['wall', 'path']:
                    continue

                if pid not in place_markers:
                    m = Marker()
                    m.header.frame_id = 'map'
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.ns = 'places'
                    m.id = mid
                    mid += 1

                    m.type = Marker.CUBE_LIST
                    m.action = Marker.ADD
                    m.scale.x = meta['spacing']
                    m.scale.y = meta['spacing']
                    m.scale.z = 0.05

                    r, g, b = self.color_from_id(pid)
                    m.color.r = r
                    m.color.g = g
                    m.color.b = b
                    m.color.a = 0.4

                    place_markers[pid] = m

                pt = Point(x=wx, y=wy, z=0.02)
                place_markers[pid].points.append(pt)

            # ===== OBJECT =====
            for obj in cell.get('objects', []):
                center = obj.get('explicit', {}).get('center_grid')
                if not center:
                    continue

                # ===== SPHERE =====
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'objects'
                marker.id = mid
                mid += 1

                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = center['x']
                marker.pose.position.y = center['y']
                marker.pose.position.z = 0.2
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0

                # ===== TEXT =====
                oid = obj.get('id', '')
                label = obj.get('symbolic', {}).get('label', '')

                text = Marker()
                text.header.frame_id = 'map'
                text.header.stamp = m.header.stamp
                text.ns = 'object_labels'
                text.id = mid; mid += 1

                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD

                text.pose.position.x = center['x']
                text.pose.position.y = center['y']
                text.pose.position.z = 0.45   # 구 위에

                text.scale.z = 0.2            # 글자 크기
                
                text.color.r = 1.0
                text.color.g = 1.0
                text.color.b = 1.0
                text.color.a = 1.0

                text.text = f"{label}\n({oid})"

                markers.markers.append(text)
                markers.markers.append(marker)

        markers.markers.extend(place_markers.values())
        self.marker_pub.publish(markers)
        self.get_logger().info('Semantic map published')


def main():
    rclpy.init()
    node = SemanticMapLoader()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
