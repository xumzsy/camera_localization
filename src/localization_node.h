
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

namespace CameraLocalization {
    

class CameraLocalizationNode{
    
public:
    CameraLocalizationNode(int max_feature_num);
    
private:
    struct Parameters{
        
    };
    
    
    struct FeatureMap{
        std::vector<cv::KeyPoint> points;
        cv::Mat descriptors;
        std::vector<int> history_time;
        std::vector<int> matched_times;
        std::vector<int> image_index;
    };
    
    // Get Parameters
    void GetParameters();
    
    // Receive a image, transform it into opencv img and address it
    void ImageCallback(const sensor_msgs::Image::ConstPtr& raw_image);
    
    // Receive a list of key points and descriptors, calculate camera pose
    // with the help of opencv
    void AddressImage(cv_bridge::CvImageConstPtr cv_ptr);
    
    // Update the feature map
    void UpdateFeatureMap();
    
    ros::NodeHandle node_handle_;
    ros::Subscriber raw_image_subscriber_;
    ros::Publisher camera_pose_publisher_;
    cv::Ptr<Feature2D> feature_detector_;
    
    // camera info (parameters)
    double cx;
    double cy;
    double fx;
    double fy;
    
    
    const int max_feature_num_;
    bool initialized_;
    
    // feature map ? considering matching, how to maintain it
    FeatureMap feature_map_;
    
    // trajectory
    std::vector<geometry_msgs::PoseStamped> camera_trajectory_;
};

} // namespace CameraLocalizationNode
