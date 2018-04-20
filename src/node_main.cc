#include "localization_node.h"


int main(int argc, char **argv){
    ::ros::init(argc, argv, "localization_node");
    ::LocalizationNode::LocalizationNode node;
    ::ros::spin();
    return 0;
}
