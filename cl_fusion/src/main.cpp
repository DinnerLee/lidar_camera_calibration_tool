#include "cl_fusion.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FUSION");
    ros::NodeHandle node;
    Fusion CL(node);

    ros::spin();

    return 0;
}
