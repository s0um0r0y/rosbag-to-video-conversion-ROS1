#!/usr/bin/env python3
import os
import argparse
import rosbag
from cv_bridge import CvBridge
import cv2

def extract_images(bag_file, output_dir, image_topic, frame_prefix='frame'):
    os.makedirs(output_dir, exist_ok=True)
    bridge = CvBridge()
    frame_count = 0

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, _ in bag.read_messages(topics=[image_topic]):
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame_path = os.path.join(output_dir, f"{frame_prefix}{frame_count:04d}.jpg")
            cv2.imwrite(frame_path, cv_img)
            frame_count += 1
    return frame_count

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract images from ROS bag')
    parser.add_argument('--bag_file', required=True, help='Input ROS bag file')
    parser.add_argument('--output_dir', default='frames', help='Output directory for images')
    parser.add_argument('--image_topic', default='/camera/image_raw', help='ROS image topic')
    args = parser.parse_args()
    
    total_frames = extract_images(args.bag_file, args.output_dir, args.image_topic)
    print(f"Extracted {total_frames} images to {args.output_dir}/")


