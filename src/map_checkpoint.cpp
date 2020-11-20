#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_processor/process.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_checkpoint");
    ros::NodeHandle n;
    ros::ServiceClient checkpoint_client = n.serviceClient<map_processor::process>("map_checkpoint", 1000);

    std::stringstream ss;
    ss << argv[1];
    map_processor::process p;
    p.request.msg = ss.str();
    checkpoint_client.call(p);
    ROS_INFO("%s", p.request.msg.data());


    return 0;
}