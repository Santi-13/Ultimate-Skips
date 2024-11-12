import rclpy
from nav_msgs.msg import OccupancyGrid
import numpy as np
from PIL import Image

class MapConverter:
    def __init__(self):
        self.map_subscriber = self.create_subscription('/map', OccupancyGrid, self.callback_map)
        
    def callback_map(self, msg):
        # Convert occupancy grid to numpy array
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # Convert to grayscale image
        img = Image.fromarray(data.astype(np.uint8))
        
        # Save as PGM file
        img.save('/home/sanmaster/ultimate_skips/Ultimate-Skips/src/explorer_skips/maps')

if __name__ == '__main__':
    rclpy.init_node('map_converter')
    converter = MapConverter()
    rclpy.spin()
