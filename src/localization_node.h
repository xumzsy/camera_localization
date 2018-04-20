
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

//#include "opencv.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

namespace CameraLocalization {
    

class CameraLocalizationNode{
    
public:
    CameraLocalizationNode(int max_feature_num);
    
private:
    struct FeatureMap{
        ::std::vector</*Point*/int> points;
        ::std::vector</*Descriptor*/int> descriptors;
        ::std::vector<int> history_time;
        ::std::vector<int> matched_times;
    };
    
    // Receive a image, transform it into opencv img and address it
    void RawImageCallback(const sensor_msgs::Image::ConstPtr& raw_image);
    
    // Receive a list of key points and descriptors, calculate camera pose
    // with the help of opencv
    void CalculatePose(cv_bridge::CvImageConstPtr cv_ptr);
    
    // Update the feature map
    void UpdateFeatureMap();
    
    // Publish camera pose
    void PublishCameraPoseCallback();
    
    ::ros::NodeHandle node_handle_;
    ::ros::Subscriber raw_image_subscriber_;
    ::ros::Publisher camera_pose_publisher_;
    
    const int max_feature_num_;
    
    // feature map ? considering matching, how to maintain it
    FeatureMap feature_map_;
    
    // trajectory
    ::std::vector<geometry_msgs::PoseStamped> camera_trajectory_;
};

} // namespace CameraLocalizationNode
