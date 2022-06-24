
#include "../include/cogniteam_person_follower.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cogniteam_person_follower_node");
    CogniteamPersonFollower cogniteamPersonFollower;
    cogniteamPersonFollower.run();
    
    return 0;
}
