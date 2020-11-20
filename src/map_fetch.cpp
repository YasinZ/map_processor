#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_processor/fetch.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_fetch");
    ros::NodeHandle n;
    ros::ServiceClient fetch_client = n.serviceClient<map_processor::fetch>("map_fetch", 1000);

    std::stringstream ss;
    ss << argv[1];
    map_processor::fetch p;
    fetch_client.call(p);

    for(auto s: p.response.results){
        ROS_INFO("%s", s.c_str());
    }

    return 0;
}