

#ifndef simple_node_h
#define simple_node_h

#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

namespace CameraLocalization {
    
public:
    SimpleNode();
private:
    void ImageCallback(const ::sensor_msgs::Image::ConstPtr& image);
    void AddressImage();
    
    // ROS API
    ros::NodeHandle node_handle_;
    ros::Subscriber raw_image_subscriber_;
    ros::Publisher camera_pose_publisher_;
    
    // camera infomation
    cv::Mat camera_info_;
    
    // feature related
    bool initialized_;
    std::vector<cv::KeyPoint> last_keypoints_;
    cv::Mat last_descriptors_;
    
    // trajectory
    std::vector<geometry_msgs::PoseStamped> camera_trajectory_;
    
} // namespace CameraLocalization


#endif /* simple_node_h */
