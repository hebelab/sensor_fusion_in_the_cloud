import rosbag
from std_msgs.msg import Header

# Specify the input and output bag files
input_bag_file = 'RTK_23-08-12.bag'
output_bag_file = 'modified.bag'

# Specify the new frame_id
new_frame_id = "map"

# Open the input and output bag files
with rosbag.Bag(output_bag_file, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag_file).read_messages():
        # Modify the frame_id in the header if applicable
        if hasattr(msg, 'header') and isinstance(msg.header, Header):
            msg.header.frame_id = new_frame_id
        # Write the modified message to the output bag file
        outbag.write(topic, msg, t)
