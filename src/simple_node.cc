
#include <opencv2/xfeatures2d.h>
#include <opencv2/calib3d.h>
#include "simple_node.h"

namespace CameraLocalization {
    
SimpleNode::SimpleNode() : initialized_(false) {
    // ROS API
    raw_image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>
                            ("/usb_cam/image_raw", 10, &CameraLocalizationNode::ImageCallback, this);
    camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/pose", 10);
    
    // Feature detector (SURF)
    feature_detector_ = xfeatures2d::SURF::create();
}
    
void SimpleNode::ImageCallback(const ::sensor_msgs::Image::ConstPtr& image){
    // convert ros image to opencv image cv_ptr->image
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare(raw_image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    AddressImage(cv_ptr);
}

void SimpleNode::AddressImage(cv_bridge::CvImagePtr cv_ptr){
    // calculate key points and descriptors
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat feature_descriptors;
    
    feature_detector_->detect(cv_ptr->image, keypoints);
    FilterKeyPoints(keypoints, 1000);
    feature_detector_->compute(cv_ptr->image, keypoints, feature_descriptors);
    
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header = cv_ptr->header;
    
    if(!initialized_){
        InitializeIdentityPose(camera_pose.pose);
        initialized_ = true;
    } else{
        // calculate camera pose
        cv::FlannBasedMatcher descriptor_matcher_;
        std::vector<cv::DMatch> matches;
        descriptor_matcher_.match(feature_descriptors, last_descriptors_, matches);
        
        // filter matches
        std::vector<cv::DMatch> good_matches;
        
        // find good points and essentialMat matrix
        std::vector<KeyPoint> last_good_points;
        std::vector<KeyPoint> good_points;
        for(auto good_match:good_matches){
            last_good_points.push_back(last_keypoints_[good_match.queryIdx]);
            good_points.push_back(keypoints[trainIdx]);
        }
        
        cv::Mat essential_matrix;
        cv::findEssentialMat(good_points,
                             last_good_points,
                             camera_info_,
                             cv::RANSAC,
                             0.99,
                             1.00
                             essential_matrix);
        
        // decompose essential matrix to find rotation and transilation
        cv::Mat R;
        cv::Mat t;
        cv::recoverPose(essential_matrix, good_points, last_good_points, camera_info_, R, t);
        
        // transform R into querion
    }
    
    // Update trajectory and publish the pose
    camera_trajectory_.push_back(camera_pose);
    camera_pose_publisher_.publish(camera_pose);
    
    last_keypoints_ = std::move(keypoints);
    last_descriptors_ = std::move(descriptors);
}
    
void SimpleNode::FilterKeyPoints(std::vector<cv::KeyPoint>& keypoints, int remain_num){
    if(keypoints.size() <= remain_num) return;
    std::sort(keypoints.begin(), keypoints.end(),[]
              (const cv::KeyPoint& a, const cv::KeyPoint& b){return a.response > b.response;});
    while(keypoints.size() > remain_num) keypoints.pop_back();
}

void InitializeIdentityPose(geometry_msgs::Pose& pose){
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
}
    
    
} // namespace CameraLocalization
