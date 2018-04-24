#include <opencv2/features2d.h>
#include <opencv2/xfeatures2d.h>
#include <opencv2/calib3d.h>
#include <math.h>
#include "simple_node.h"

namespace CameraLocalization {
    
    SimpleNode::SimpleNode() : initialized_(false), camera_info_(cv::Mat::zeros(3,3,cv::CV_64F)) {
    // ROS API
    raw_image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>
                            ("/usb_cam/image_raw", 10, &CameraLocalizationNode::ImageCallback, this);
    camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/camera_pose", 10);
    
    // camera info (hard coded now)
    camera_info_.at(0,0) = 0.0;   // fx
    camera_info_.at(1,1) = 0.0;   // fy
    camera_info_.at(0,2) = 0.0;   // cx
    camera_info_.at(1,2) = 0.0;   // cy
    camera_info_.at(2,2) = 1.0;
        
    // Feature detector (SURF)
    feature_detector_ = xfeatures2d::SURF::create();
    descriptor_matcher_ = cv::FlannBasedMatcher::create();
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
        std::vector<std::vector<cv::DMatch>> matches;
        descriptor_matcher_.knnMatch(feature_descriptors, last_descriptors_, matches, 2);
        
        // filter matches by ratio bewteen best and second best match
        std::vector<cv::DMatch> good_matches;
        FilterMatchesByRatio(matches, good_matches);
        
        
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
        
        // transform R into quaterion
        RotationMatToQuaternion(R, camera_pose.pose.orientation);
        camera_pose.pose.position.x = t.at(0);
        camera_pose.pose.position.y = t.at(1);
        camera_pose.pose.position.z = t.at(2);
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
    
void SimpleNode::FilterMatchesByRatio(std::vector<std::vector<cv::DMatch>>& matches,
                                      std::vector<cv::DMatch>& good_matches){
    for(auto match:matches){
        if(match.size()==2 && match.back().distance/match.front().distance < 0.6){
            good_matches.push_back(match.front());
        }
    }
}

void SimpleNode::RotationMatToQuaternion(cv::Mat& R, geometry_msgs::Quaternion& quaternion){
    quaternion.w = sqrt(1 + R.at(0,0) + R.at(1,1) + R.at(2,2))/2;
    quaternion.x = (R.at(2,1) - R.at(1,2))/(4*quaternion.w);
    quaternion.y = (R.at(0,2) - R.at(2,0))/(4*quaternion.w);
    quaternion.x = (R.at(1,0) - R.at(0,1))/(4*quaternion.w);
}
    
void SimpleNode::InitializeIdentityPose(geometry_msgs::Pose& pose){
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
}
    
    
} // namespace CameraLocalization
