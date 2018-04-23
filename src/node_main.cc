
//#include "localization_node.h"
#include "simple_node.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "localization_node");
    //CameraLocalization::CameraLocalizationNode node(10);
    CameraLocalization::SimpleNode node;
    ros::spin();
    return 0;
}
