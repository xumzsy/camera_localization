
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

namespace {
class SyncNode{
public:
    SyncNode(){};
private:
    geometry_msgs::PoseStamped get_tf_frame(std::string base, std::string target){
        tf::StampedTransform transform;
        frame_listener.waitForTransform(base,target,ros::Time(),ros::Duration(2.0));
        try
        {
            frame_listener.lookupTransform(base, target, ros::Time(0), transform_);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
        }
        
        // set frame data
        geometry_msgs::PoseStamped frame;
        frame.header.stamp = ros::Time::now();
        frame.header.frame_id = target;
        frame.pose.position.x = transform.getOrigin().getX();
        frame.pose.position.y = transform.getOrigin().getY();
        frame.pose.position.z = transform.getOrigin().getZ();
        frame.pose.orientation.x = transform.getRotation().getX();
        frame.pose.orientation.y = transform.getRotation().getY();
        frame.pose.orientation.z = transform.getRotation().getZ();
        frame.pose.orientation.w = transform.getRotation().getW();
        
        return frame;
    }
    
    
    // several interpolation method based on time
    // init method to calculate transformation
    // method to calculate, record and report error
    
    
    std::vector<geometry_msgs::PoseStamped> trajectory_;
    geometry_msgs::Pose transformation;
};
} // namespace

int main(int argc, char** argv){
    ros::init(argc, argv, "sync_node");
}
