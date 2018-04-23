
#include <opencv2/xfeatures2d.h>

#include "localization_node.h"

namespace CameraLocalization {
    
// Get paramters from rosparam server and pass them to parameters_
void CameraLocalizationNode::GetParameters(){
    if(node_handle_.getParam("/max_feature_num",parameter_.max_feature_num)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
    if(node_handle_.getParam("/update_feature_num",parameter_.update_feature_num)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
    if(node_handle_.getParam("/cx",parameter_.cx)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
    if(node_handle_.getParam("/cy",parameter_.cy)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
    if(node_handle_.getParam("/fx",parameter_.fx)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
    if(node_handle_.getParam("/fy",parameter_.fy)){
        std::cout<<"Fail to get parameter"<<std::endl;
    }
}
    
// Constructor
CameraLocalizationNode::CameraLocalizationNode() : initialized_(false){
    GetParameters();
    // ROS API
    raw_image_subscriber_ = node_handle_.subscribe<sensor_msgs::Image>
    ("/usb_cam/image_raw", 10, &CameraLocalizationNode::ImageCallback, this);
    camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/pose", 10);
    
    // Feature detector (SURF)
    feature_detector_ = xfeatures2d::SURF::create();
}
    
// Receive image message, transform into cv image and callback
void CameraLocalizationNode::ImageCallback(const ::sensor_msgs::Image::ConstPtr& raw_image){
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

// Main function to address the comming image
void CameraLocalizationNode::AddressImage(cv_bridge::CvImageConstPtr cv_ptr){
    // Calculate key points
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat feature_descriptors;
    feature_detector_->detect(cv_ptr->image, keypoints);
    
    // sort keypoints by response and only keep the keeps part ot them
    std::sort(keypoints.begin(), keypoints.end(),[](const cv::KeyPoint& a, const cv::KeyPoint& b){
        return a.response > b.response;});
    
    // compute feature descriptors
    feature_detector_->compute(cv_ptr->image, keypoints, feature_descriptors);
    
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header = cv_ptr->header;
    // Initialize if not
    if(!initialized_){
        // identity position
        camera_pose.pose.position.x = 0.0;
        camera_pose.pose.position.y = 0.0;
        camera_pose.pose.position.z = 0.0;
        camera_pose.pose.orientation.x = 0.0;
        camera_pose.pose.orientation.y = 0.0;
        camera_pose.pose.orientation.z = 0.0;
        camera_pose.pose.orientation.w = 1.0;
        initialized_ = true;
    } else{
        // match and calculate camera pose
        cv::FlannBasedMatcher descriptor_matcher_;
        std::vector<cv::DMatch> matches;
        descriptor_matcher_.match(feature_descriptors, feature_map_.descriptors, matches);
        
        // filter the matches
        double min_match_distance = DBL_MAX;
        for(auto match:matches) min_match_distance = min(min_match_distance, match.distance);
        const double match_distance_threshold = 2 * min_match_distance;
        
        // optimize reprojection error to solve pose
        // for good_match, need some math and codes for optimization
        // Rememmber to update matched times
    }
    
    // Update trajectory and publish the pose
    camera_trajectory_.push_back(camera_pose);
    camera_pose_publisher_.publish(camera_pose);
    
    // Updateing existing Feature Map History
    for(auto time:feature_map_.history_time) time++;
    
    // Update Feature Map
    const int camera_pose_index = camera_trajectory_.size() - 1;
    if(parameter_.update_feature_num + feature_map_.points.size() <= parameter_.max_feature_num){
        for(int i=0;i<keypoints.size();++i){
            feature_map_.points.push_back(keypoints[i]);
            feature_map_.descriptors.push_back(feature_descriptors[i]);
            feature_map_.history_time.push_back(0);
            feature_map_.matched_times.push_back(0);
            feature_map_.image_index.push_back(camera_pose_index);
        }
    } else{
        // score = score(history, matched_time)
        vector<int> scores;
        for(int i=0;i<keypoints.size();++i){
            scores.push_back(feature_map_.matched_time[i]*100000-history_time);
        }
        sort(scores.begin(), scores.end());
        const int scores_threshold = scores[parameters_.update_feature_num-1];
        int i=0, count = 0;
        while(i<feature_map_.keypoints.size()){
            int score = feature_map_.matched_time[i]*100000-history_time;
            if(score <= scores_threshold){
                feature_map_.points[i] = keypoints[count];
                feature_map_.descriptors[i] = feature_descriptors[i];
                feature_map_.history_time[i] = 0;
                feature_map_.matched_time[i] = 0;
                feature_map_.image_index[i] = camera_pose_index;
                ++count;
            }
            ++i;
        }
    }
    
}

void CameraLocalizationNode::UpdateFeatureMap(){
    // Mainly maintain the keypoints reprojects into the first image
    // correspond descriptors and some history image
    // which image it belongs (mainly we need the position related to first image)
}

    
} // namespace CameraLocalizationNode

