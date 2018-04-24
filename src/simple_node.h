

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
    
    void AddressImage(cv_bridge::CvImagePtr cv_ptr);
    
    void FilterKeyPoints(std::vector<cv::KeyPoint>& keypoints, int remain_num);
    
    void FilterMatchesByRatio(std::vector<std::vector<cv::DMatch>>& matches,
                              std::vector<cv::DMatch>& good_matches);
    
    void RotationMatToQuaternion(cv::Mat& R, geometry_msgs::Quaternion& quaternion);
    
    void InitializeIdentityPose(geometry_msgs::Pose& pose);
    
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
    
    cv::Ptr<Feature2D> feature_detector_;
    cv::Ptr<DescriptorMatcher> descriptor_matcher_;
    // trajectory
    std::vector<geometry_msgs::PoseStamped> camera_trajectory_;
    
} // namespace CameraLocalization


#endif /* simple_node_h */
