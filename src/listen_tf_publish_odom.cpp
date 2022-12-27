
#include <ros/ros.h>
#include <tf/transform_listener.h>  
#include <nav_msgs/Odometry.h>	// odom

int main(int argc, char** argv){
    
	ros::init(argc, argv, "listen_tf_publish_odom");
        
	ros::NodeHandle nh;
        ros::NodeHandle pp("~"); //private parameters
        
        std::string parent_frame, child_frame;
        double publish_rate, diag_cov;
        pp.param<std::string>("parent_frame", parent_frame, "odom");       //change if needed
        pp.param<std::string>("child_frame", child_frame, "base_link");    //change if needed
        pp.param<double>("publish_rate", publish_rate, 20.0);    //change if needed
        pp.param<double>("diag_cov", diag_cov, 0.00001);    //change if needed
        
        tf::TransformListener listener;
        ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 50); //remap if needed
        ros::Rate rate(publish_rate);
  
        //Set up TFs:
        listener.waitForTransform(parent_frame, child_frame, ros::Time::now(), ros::Duration(5.0));
        ROS_INFO("Listening to TF between %s and %s.", parent_frame.c_str(), child_frame.c_str() );        
 
        //Get TFs:
        while (nh.ok()){ 
            
            tf::StampedTransform transform; 
            nav_msgs::Odometry odom_msg;
            
             try{
                    listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
                    
                    //fill odom:
                    odom_msg.header.stamp = transform.stamp_;
                    odom_msg.header.frame_id = transform.frame_id_;
                    odom_msg.child_frame_id = transform.child_frame_id_;
                    odom_msg.pose.pose.position.x = transform.getOrigin().x();
                    odom_msg.pose.pose.position.y = transform.getOrigin().y();    
                    odom_msg.pose.pose.position.z = transform.getOrigin().z(); 
                    
                    //this is not supported:
                    //odom_msg.pose.pose.orientation = transform.getRotation();
                    
                    //must be done like this:
                    quaternionTFToMsg( transform.getRotation(), odom_msg.pose.pose.orientation);

                    odom_msg.pose.covariance[0] = diag_cov;
                    odom_msg.pose.covariance[7] = diag_cov;
                    odom_msg.pose.covariance[14] = diag_cov;
                    odom_msg.pose.covariance[21] = diag_cov;
                    odom_msg.pose.covariance[28] = diag_cov;
                    odom_msg.pose.covariance[35] = diag_cov;
                    
                    //publish odom:
                    pub_odom.publish(odom_msg);
                    
                    
            }catch (tf::TransformException ex){
                    ROS_WARN("%s exception: %s",ros::this_node::getName().c_str(),ex.what());
                    ros::Duration(2.0).sleep();
            }

            ros::spinOnce();
            rate.sleep();
        } 
        
        return 0;
};