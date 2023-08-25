import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

class GridMapVisualizer:
    def __init__(self):
        rospy.init_node('grid_map_visualizer')

        # Set parameters
        self.map_size = 400
        self.resolution = 0.05

        # Create a CvBridge
        self.bridge = CvBridge()

        # Create a blank image with a bit lighter gray color
        self.blank_image = np.ones((self.map_size, self.map_size), dtype=np.uint8) * 100

        # Store max lidar range
        self.max_lidar_range = 35.0  # Adjust this value as needed

        # Subscribe to LaserScan topic
        rospy.Subscriber('/scan2', LaserScan, self.scan_callback)

        # Main loop
        rospy.spin()

    def scan_callback(self, scan_msg):
        scan = scan_msg
        maxAngle = scan.angle_max
        minAngle = scan.angle_min
        angleInc = scan.angle_increment
        ranges = scan.ranges
        num_pts = len(ranges)

        disc_factor = self.map_size / (2 * self.max_lidar_range)

        image_copy = np.copy(self.blank_image)
        closest_edge_distance = self.max_lidar_range

        for i in range(num_pts):
            angle = minAngle + float(i) * angleInc
            distance = ranges[i]

            if 0 <= distance <= self.max_lidar_range:
                x = int((distance * math.cos(angle + math.pi/2) + self.max_lidar_range) * disc_factor)
                y = int((-distance * math.sin(angle + math.pi/2) + self.max_lidar_range) * disc_factor)

                if 0 <= x < self.map_size and 0 <= y < self.map_size:
                    cv2.line(image_copy, (self.map_size // 2, self.map_size // 2), (x, y), 200, 1)
                    
                    if distance < closest_edge_distance:
                        closest_edge_distance = distance
                        closest_edge_x = x
                        closest_edge_y = y

        # Draw the LiDAR square at the center
        self.draw_lidar_square(image_copy)

        # Draw a black dot at the closest laser edge
        cv2.circle(image_copy, (closest_edge_x, closest_edge_y), 3, 0, -1)

        # Display the map
        self.display_map(image_copy)

    def draw_lidar_square(self, image):
        # Reduce the size of the square at the center to represent the LiDAR
        lidar_size = 5  # Adjust this value for the size of the square
        center_x = self.map_size // 2
        center_y = self.map_size // 2
        start_x = center_x - lidar_size // 2
        start_y = center_y - lidar_size // 2
        image[start_y:start_y + lidar_size, start_x:start_x + lidar_size] = 0

    def display_map(self, image):
        # Zoom in the visualization by cropping a smaller region
        zoom_factor = 0.6  # Adjust this value to zoom in/out
        crop_size = int(self.map_size * zoom_factor)
        cropped_image = image[
            (self.map_size - crop_size) // 2 : (self.map_size + crop_size) // 2,
            (self.map_size - crop_size) // 2 : (self.map_size + crop_size) // 2
        ]

        # Resize the cropped image back to the original window size
        resized_image = cv2.resize(cropped_image, (self.map_size, self.map_size))

        # Flip the visualization horizontally to the left and then mirror
        resized_image = np.flip(resized_image, axis=1)
        resized_image = np.flip(resized_image, axis=1)

        # Show the map
        cv2.imshow('Map', resized_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    visualizer = GridMapVisualizer()

