#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyttsx3
import os
import time
from collections import Counter

class TravelToHazmatNode(Node):
    def __init__(self):
        super().__init__('travel_to_hazmat')
        
        # Initialize the text-to-speech engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)  # Voice speed
        self.engine.setProperty('volume', 1.0)  # Voice volume (0.0 to 1.0)
        
        # Set language to English
        voices = self.engine.getProperty('voices')
        for voice in voices:
            if "english" in voice.name.lower():  # Find an English voice
                self.engine.setProperty('voice', voice.id)
                break

        # Path to the hazmat file
        self.hazmat_file_path = "hazmatDetected.txt"
        
        # Read and announce hazmat types once
        self.read_and_announce_hazmat()
        
    def read_and_announce_hazmat(self):
        # Check if the file exists and has content
        if os.path.exists(self.hazmat_file_path):
            hazmat_types = []

            # Process each line in the file to get only the hazmat type
            with open(self.hazmat_file_path, 'r') as file:
                for line in file:
                    # Extract hazmat type before "at" (ignore coordinates)
                    if "Hazmat detected:" in line:
                        hazmat_type = line.split("Hazmat detected:")[1].split(" at")[0].strip()
                        hazmat_types.append(hazmat_type)

            # Count unique hazmat types detected
            hazmat_count = Counter(hazmat_types)
            total_hazmat = len(hazmat_count)
            
            if total_hazmat > 0:
                # Generate the announcement message in English
                hazmat_list = ", ".join(hazmat_count.keys())
                message = f"{total_hazmat} types of hazmats have been detected: {hazmat_list}."
                self.get_logger().info(message)
                self.engine.say(message)
                self.engine.runAndWait()
                time.sleep(0.5)  # Short pause to ensure full message playback

                # Prompt to select a hazmat type to travel to
                select_message = "Select the number of which hazmat you want to travel."
                self.get_logger().info(select_message)
                self.engine.say(select_message)
                self.engine.runAndWait()
                time.sleep(3)  # Short pause to ensure full message playback
            else:
                self.get_logger().info("No new hazmat types found to announce.")
        else:
            self.get_logger().info("The hazmatDetected.txt file does not exist or is empty.")

def main(args=None):
    rclpy.init(args=args)
    node = TravelToHazmatNode()
    rclpy.spin(node)  # Keeps the node active to allow future interactions if needed

if __name__ == '__main__':
    main()
