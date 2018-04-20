
#include "localization_node.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "localization_node");
    CameraLocalization::CameraLocalizationNode node(10);
    ros::spin();
    return 0;
}
