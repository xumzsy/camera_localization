
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

namespace {
class SyncNode{
public:
    SyncNode() : initialized_(false){
        camera_pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseStamped>("/camera_pose", 10, &SyncNode::CameraPoseCallback, this);
        
    };
private:
    geometry_msgs::PoseStamped get_tf_frame(std::string base, std::string target){
        tf::StampedTransform transform;
        frame_listener_.waitForTransform(base,target,ros::Time(),ros::Duration(2.0));
        try
        {
            frame_listener_.lookupTransform(base, target, ros::Time(0), transform);
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
    
    void CameraPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& camera_pose){
        // Robot = relative_T * Camera
        geometry_msgs::PoseStamped robot_pose = get_tf_frame("base_link", "ee_link");
        std::cout<<"Time Gap: "<<(ros::Time::now().nsec - camera_pose->header.stamp.nsec)/1e6<<" ms"<<std::endl;
        if(!initialized_){
            //relative_transformation_ = robot_pose.pose;
            camera_pose_to_base_ = robot_pose.pose;
            std::cout<<"Successfully Initialized"<<std::endl;
            initialized_ = true;
        } else{
            // calculate new camera pose to base
            // camera_pose_to_base_ * camera_pose
            UpdateCameraPoseToBase(camera_pose->pose);
            
            // Report Error
            CalculateError(robot_pose.pose, camera_pose_to_base_);
        }
    }
    
    void UpdateCameraPoseToBase(const geometry_msgs::Pose& camera_pose){
        
        geometry_msgs::Quaternion q_to_base = camera_pose_to_base_.orientation;
        double R00 = 1.0 - 2*(q_to_base.y*q_to_base.y + q_to_base.z*q_to_base.z);
        double R01 = 2*(q_to_base.x*q_to_base.y - q_to_base.z*q_to_base.w);
        double R02 = 2*(q_to_base.x*q_to_base.z + q_to_base.y*q_to_base.w);
        double R10 = 2*(q_to_base.x*q_to_base.y + q_to_base.z*q_to_base.w);
        double R11 = 1.0 - 2*(q_to_base.x*q_to_base.x + q_to_base.z*q_to_base.z);
        double R12 = 2*(q_to_base.y*q_to_base.z - q_to_base.x*q_to_base.w);
        double R20 = 2*(q_to_base.x*q_to_base.z - q_to_base.y*q_to_base.w);
        double R21 = 2*(q_to_base.y*q_to_base.z + q_to_base.x*q_to_base.w);
        double R22 = 1.0 - 2*(q_to_base.x*q_to_base.x + q_to_base.y*q_to_base.y);
        
        // translation
        geometry_msgs::Point t2 = camera_pose.position;
        camera_pose_to_base_.position.x += R00 * t2.x + R01 * t2.y + R02 * t2.z;
        camera_pose_to_base_.position.y += R10 * t2.x + R11 * t2.y + R12 * t2.z;
        camera_pose_to_base_.position.z += R20 * t2.x + R21 * t2.y + R22 * t2.z;
        
        geometry_msgs::Quaternion q2 = camera_pose.orientation;
        geometry_msgs::Quaternion updated_q_to_base;
        updated_q_to_base.w = q_to_base.w * q2.w - q_to_base.x * q2.x -
                              q_to_base.y * q2.y - q_to_base.z * q2.z;
        updated_q_to_base.x = q_to_base.w * q2.x + q_to_base.x * q2.w +
                              q_to_base.y * q2.z - q_to_base.z * q2.y;
        updated_q_to_base.y = q_to_base.w * q2.y - q_to_base.x * q2.z +
                              q_to_base.y * q2.w + q_to_base.z * q2.x;
        updated_q_to_base.z = q_to_base.w * q2.z + q_to_base.x * q2.y -
                              q_to_base.y * q2.x + q_to_base.z * q2.w;
        camera_pose_to_base_.orientation = std::move(updated_q_to_base);
    }
    
    void CalculateError(const geometry_msgs::Pose& robot_pose,
                        const geometry_msgs::Pose& camera_pose){
        
        std::cout<<"Error of x"<<camera_pose.position.x - robot_pose.position.x<<std::endl;
        std::cout<<"Error of y"<<camera_pose.position.y - robot_pose.position.y<<std::endl;
        std::cout<<"Error of z"<<camera_pose.position.z - robot_pose.position.z<<std::endl;
        std::cout<<"Error of qx"<<camera_pose.orientation.x - robot_pose.orientation.x<<std::endl;
        std::cout<<"Error of qy"<<camera_pose.orientation.y - robot_pose.orientation.y<<std::endl;
        std::cout<<"Error of qz"<<camera_pose.orientation.z - robot_pose.orientation.z<<std::endl;
        std::cout<<"Error of qw"<<camera_pose.orientation.w - robot_pose.orientation.w<<std::endl;
    }
    
    
    // ROS API
    ros::NodeHandle node_handle_;
    ros::Subscriber camera_pose_subscriber_;
    tf::TransformListener frame_listener_;
    
    // several interpolation method based on time
    // init method to calculate transformation
    // method to calculate, record and report error
    
    
    // std::vector<geometry_msgs::PoseStamped> trajectory_;
    bool initialized_;
    geometry_msgs::Pose camera_pose_to_base_;
    //geometry_msgs::Pose relative_transformation_;
    
    
};
} // namespace


// main function
int main(int argc, char** argv){
    ros::init(argc, argv, "sync_node");
    SyncNode node;
    ros::spin();
    return 0;
}
