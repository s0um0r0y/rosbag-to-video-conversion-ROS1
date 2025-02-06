# rosbag-to-video-conversion-ROS1
steps to convert rosbag to video in mp4 format to prevent latency

## To process a ROS bag file and convert its images into a video, follow this streamlined approach using Python and FFmpeg:
### Step 1: Extract Images from ROS Bag

`python3 rosbag_to_video.py --bag_file input.bag --output_dir frames --image_topic /camera/image_raw`

### Step 2: Convert Images to Video with FFmpeg
After extraction, use this FFmpeg command to create an MP4 video:

`ffmpeg -framerate 30 -pattern_type glob -i "frames/frame*.jpg" 
       -c:v libx264 -crf 25 -pix_fmt yuv420p output_video.mp4`
       
### Key Parameters:
framerate 30: Set input frame rate (adjust based on bag frequency)

c:v libx264: H.264 video codec

crf 25: Quality control (lower = better quality)

pix_fmt yuv420p: Ensures compatibility with media players
