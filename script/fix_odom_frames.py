
#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry

import message_filters


class FixOdom():

    def __init__(self):

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        #Publishers
        self.pub_odom = rospy.Publisher("~odom_fixed", Odometry, queue_size=10)

        #Subscribers
        self.sub_odom_topic = message_filters.Subscriber("~odom_topic", Odometry, queue_size=100, buff_size=2**27)
        
        #This will synchronize the two pointclouds. And a single callback funtion will be called.
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_odom_topic], 100, 0.1)
        ts.registerCallback(self.odomCallback)

    #Callbacks
    def odomCallback(self, odom_data): #Function that runs when both pointclouds arrive
        """
        This is the callback function. It's main job is to be called when a pointcloud arrives.
        This function will merge the pointclouds from successive iteration, and when it merges self.decay
        it will publish the aggregated (dense) pointcloud.
        Inputs:
            - self
            - lidar_data: Pointcloud2
        Outputs:
            - None
        """
        odom_data.header.frame_id = self.frame_id
        odom_data.child_frame_id = self.child_frame

        self.pub_odom.publish(odom_data)

def main():

    rospy.init_node('fix_odom_frames_node')

    fo = FixOdom()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("error")
        pass
    print("Exiting process...")