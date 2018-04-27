
#include <math.h>
#include "simple_node.h"

namespace CameraLocalization {
    
SimpleNode::SimpleNode() : initialized_(false), camera_info_(cv::Mat::zeros(3,3,CV_64F)) {
// ROS API
raw_image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>
                        ("/usb_cam/image_raw", 10, &SimpleNode::ImageCallback, this);
camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/camera_pose", 10);

// camera info (hard coded now)
camera_info_.at<double>(0,0) = 640.0;   // fx
camera_info_.at<double>(1,1) = 640.0;   // fy
camera_info_.at<double>(0,2) = 308.0;   // cx
camera_info_.at<double>(1,2) = 259.0;   // cy
camera_info_.at<double>(2,2) = 1.0;
    
// Feature detector (SURF)
feature_detector_ = cv::xfeatures2d::SURF::create();
descriptor_matcher_ = cv::FlannBasedMatcher::create();
}
    
void SimpleNode::ImageCallback(const ::sensor_msgs::Image::ConstPtr& image){
    // convert ros image to opencv image cv_ptr->image
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvShare(image);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    AddressImage(cv_ptr);
}

void SimpleNode::AddressImage(cv_bridge::CvImageConstPtr cv_ptr){
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
        descriptor_matcher_->knnMatch(feature_descriptors, last_descriptors_, matches, 2);
        std::cout<<"Match Num "<<matches.size()<<std::endl;
        // filter matches by ratio bewteen best and second best match
        std::vector<cv::DMatch> good_matches;
        FilterMatchesByRatio(matches, good_matches);
        std::cout<<"Good Match Num "<<good_matches.size()<<std::endl;
        
        // find good points and essentialMat matrix
        std::vector<cv::Point2f> last_good_points;
        std::vector<cv::Point2f> good_points;
        for(auto good_match:good_matches){
            last_good_points.push_back(last_keypoints_[good_match.queryIdx].pt);
            good_points.push_back(keypoints[good_match.trainIdx].pt);
        }
        std::cout<<last_good_points.size()<<std::endl;
        std::cout<<good_points.size()<<std::endl;
        cv::Mat essential_matrix;
        essential_matrix = cv::findEssentialMat(good_points,
                             last_good_points,
                             camera_info_,
                             cv::FM_8POINT,
                             0.1,
                             3.0);
        std::cout<<"Find essential matrix"<<std::endl;
        std::cout<<essential_matrix.empty()<<std::endl;
        // decompose essential matrix to find rotation and transilation
        cv::Mat R;
        cv::Mat t;
        cv::recoverPose(essential_matrix, good_points, last_good_points, camera_info_, R, t);
        
        // transform R into quaterion
        RotationMatToQuaternion(R, camera_pose.pose.orientation);
        camera_pose.pose.position.x = t.at<double>(0);
        camera_pose.pose.position.y = t.at<double>(1);
        camera_pose.pose.position.z = t.at<double>(2);
    }
    
    // Update trajectory and publish the pose
    camera_trajectory_.push_back(camera_pose);
    camera_pose_publisher_.publish(camera_pose);
    
    last_keypoints_ = std::move(keypoints);
    last_descriptors_ = std::move(feature_descriptors);
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
        std::cout<<match.front().distance<<","<<match.back().distance<<std::endl;
            
        if(match.size()==2 && match.front().distance/match.back().distance < 0.6){
            good_matches.push_back(match.front());
        }
    }
}

void SimpleNode::RotationMatToQuaternion(cv::Mat& R, geometry_msgs::Quaternion& quaternion){
    
    quaternion.w = sqrt(1 + R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2))/2;
    quaternion.x = (R.at<double>(2,1) - R.at<double>(1,2))/(4*quaternion.w);
    quaternion.y = (R.at<double>(0,2) - R.at<double>(2,0))/(4*quaternion.w);
    quaternion.x = (R.at<double>(1,0) - R.at<double>(0,1))/(4*quaternion.w);
    
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
