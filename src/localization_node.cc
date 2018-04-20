
#include <cv_bridge/cv_bridge.h>

#include "localization_node.h"


namespace CameraLocalizationNode {
    
CameraLocalizationNode::CameraLocalizationNode(int max_feature_num)
    : max_feature_num_(max_feature_num){
    raw_image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>
        ("/image", 10, &CameraLocalizationNode::RawImageCallback, this);
    camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/pose", 10);
}
    
void CameraLocalizationNode::RawImageCallback(const ::sensor_msgs::Image::ConstPtr& raw_image){
    // convert ros image to opencv image cv_ptr->image
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare(raw_image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Calculate the pose
    CalculatePose(cv_ptr);
}

void CameraLocalizationNode::CalculatePose(cv_bridge::CvImageConstPtr cv_ptr){
    // Calculate key points and descriptor
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create();
    detector->setHessianThreshold(minHessian);
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
    detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );
}

void CameraLocalizationNode::UpdateFeatureMap(){
    
}
    
} // namespace CameraLocalizationNode

