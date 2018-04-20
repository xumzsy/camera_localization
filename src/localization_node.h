
#include <vector>

#include "opencv.h"
#include "ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"

namespace CameraLocalizationNode {
    

class CameraLocalizationNode{
    
public:
    CameraLocalizationNode();
    
private:
    struct FeatureMap{
        ::std::vector<Point> points;
        ::std::vector<Descriptor> descriptors;
        ::std::vector<int> history_time;
        ::std::vector<int> matched_times;
    };
    
    // Receive a image, transform it into opencv img and address it
    void RawImageCallback(const ::sensor_msgs::Image::ConstPtr& raw_image);
    
    // Receive a list of key points and descriptors, calculate camera pose
    // with the help of opencv
    void CalculatePose();
    
    // Update the feature map
    void UpdateFeatureMap();
    
    ::ros::NodeHandle node_handle_;
    ::ros::Subscriber raw_image_subscriber_;
    ::ros::Publisher camera_pose_publisher_;
    
    const int max_feature_num;
    
    // feature map ? considering matching, how to maintain it
    FeatureMap feature_map_;
    
    // trajectory
    ::std::vector<::geometry_msgs::PoseStamped> camera_trajectory_;
};

} // namespace CameraLocalizationNode
