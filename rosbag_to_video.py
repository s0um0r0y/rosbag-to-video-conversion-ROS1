# import os
# import cv2
# import rosbag
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# def extract_images_from_rosbag(bag_file, image_topic, output_dir):
#     os.makedirs(output_dir, exist_ok=True)
#     bag = rosbag.Bag(bag_file, 'r')
#     bridge = CvBridge()
#     count = 0

#     for topic, msg, t in bag.read_messages(topics=[image_topic]):
#         if isinstance(msg, Image):
#             cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             image_path = os.path.join(output_dir, f"frame_{count:06d}.jpg")
#             cv2.imwrite(image_path, cv_image)
#             count += 1
    
#     bag.close()
#     print(f"Extracted {count} images to {output_dir}")
#     return count

# def create_video_from_images(image_dir, output_video, fps=30):
#     images = sorted([img for img in os.listdir(image_dir) if img.endswith(".jpg")])
#     if not images:
#         print("No images found in the directory!")
#         return
    
#     first_image = cv2.imread(os.path.join(image_dir, images[0]))
#     height, width, _ = first_image.shape
#     fourcc = cv2.VideoWriter_fourcc(*'mp4v')
#     video = cv2.VideoWriter(output_video, fourcc, fps, (width, height))

#     for image in images:
#         frame = cv2.imread(os.path.join(image_dir, image))
#         video.write(frame)
    
#     video.release()
#     print(f"Video saved to {output_video}")

# if __name__ == "__main__":
#     bag_file = "/home/soumoroy/Downloads/headon-stop.bag"  # Change this to your ROS bag file
#     image_topic = "/zed/zed_node/rgb_raw/image_raw_color"  # Change to your topic name
#     output_dir = "frames"
#     output_video = "output.mp4"
#     fps = 30

#     print("Extracting images...")
#     extract_images_from_rosbag(bag_file, image_topic, output_dir)
    
#     print("Creating video...")
#     create_video_from_images(output_dir, output_video, fps)

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


