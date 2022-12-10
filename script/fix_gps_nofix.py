import rosbag
from sensor_msgs.msg import CompressedImage


with rosbag.Bag('output1.bag', 'w') as outbag:
	for topic, msg, t in rosbag.Bag('/storage/rosbags/choupal/choupal.bag').read_messages():
		if topic == "/mi9t/gps/assisted" or topic == "/mimix3/gps/assisted":
			msg.status.status = 0
			print("Updated compressed format! \"%s\"" % (msg))

		outbag.write(topic, msg, t)