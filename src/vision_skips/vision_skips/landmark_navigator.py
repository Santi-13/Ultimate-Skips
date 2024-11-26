import os
from typing import List, Tuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class LandmarkNavigator(Node):
    def __init__(self):
        super().__init__('landmark_navigator')
        self.landmarks = self.read_landmarks_from_file()
        self.goal_publisher = self.create_publisher(PoseStamped, 'send_goal', 10)
        
    def read_landmarks_from_file(self) -> List[Tuple[str, float, float]]:
        """Read landmarks from a .txt file."""
        filename = os.path.join(os.getcwd(), 'landmarks.txt')
        try:
            with open(filename, 'r') as f:
                landmarks = []
                for line in f.readlines():
                    name, x, y = line.strip().split(',')
                    landmarks.append((name, float(x), float(y)))
            return landmarks
        except FileNotFoundError:
            self.get_logger().error(f"File {filename} not found.")
            return []

    def display_options(self):
        """Display available landmark options to the user."""
        print("Available landmarks:")
        for i, (name, _, _) in enumerate(self.landmarks, 1):
            print(f"{i}. {name}")

    def get_user_input(self) -> int:
        """Get user input and validate it."""
        while True:
            try:
                choice = int(input("Enter the number of your chosen landmark: "))
                if 1 <= choice <= len(self.landmarks):
                    return choice
                else:
                    print("Invalid choice. Please enter a number between 1 and", len(self.landmarks))
            except ValueError:
                print("Invalid input. Please enter a number.")

    def send_goal(self, index: int):
        """Send the selected landmark as a goal."""
        name, x, y = self.landmarks[index - 1]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # Default orientation (no rotation)
        
        self.get_logger().info(f'Sending goal to landmark "{name}" at ({x}, {y})')
        self.goal_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    navigator = LandmarkNavigator()

    try:
        while True:
            navigator.display_options()
            choice = navigator.get_user_input()
            navigator.send_goal(choice)
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
