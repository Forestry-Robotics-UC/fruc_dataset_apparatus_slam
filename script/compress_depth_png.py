
import rosbag
from sensor_msgs.msg import CompressedImage


with rosbag.Bag('output1.bag', 'w') as outbag:
	for topic, msg, t in rosbag.Bag('/storage/rosbags/choupal/choupal.bag').read_messages():
		if topic == "/realsense/aligned_depth_to_color/image_raw/compressedDepth" or topic == "/mynteye/depth/image_raw/compressedDepth":
			msg.format = msg.format + " png"
			print("Updated compressed format! \"%s\"" % (msg.format))

		outbag.write(topic, msg, t)