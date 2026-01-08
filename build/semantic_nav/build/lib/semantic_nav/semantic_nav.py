import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class PlaceGoalNode(Node):
    def __init__(self):
        super().__init__('place_goal_node')

        # === JSON 로드 ===
        with open('/home/n/semantic_map_ui/public/semantic_grid_map.json') as f:
            data = json.load(f)

        self.grid = data['grid']

        # === Nav2 Action Client ===
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info('PlaceGoalNode ready')

    def find_place_center(self, place_name):
        for cell in self.grid.values():
            for place in cell.get('places', []):
                if place.get('id') == place_name:
                    return place.get('explicit', {}).get('center_grid')
        return None

    def send_goal(self, place_name):
        center = self.find_place_center(place_name)

        if center is None:
            self.get_logger().error(f'Place "{place_name}" not found')
            return

        self.nav_client.wait_for_server()

        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = center['x']
        pose.pose.position.y = center['y']
        pose.pose.orientation.w = 1.0

        goal.pose = pose

        self.get_logger().info(
            f'Sending goal to place "{place_name}" '
            f'({center["x"]:.2f}, {center["y"]:.2f})'
        )

        self.nav_client.send_goal_async(goal)


def main():
    rclpy.init()
    node = PlaceGoalNode()

    # === 테스트용: 여기 place 이름 바꿔서 실행 ===
    node.send_goal('room3')

    rclpy.spin(node)


if __name__ == '__main__':
    main()
