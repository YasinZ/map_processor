#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_processor/save.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_save");
    ros::NodeHandle n;
    ros::ServiceClient save_client = n.serviceClient<map_processor::save>("map_save", 1000);

    map_processor::save p;
    save_client.call(p);

    return 0;
}