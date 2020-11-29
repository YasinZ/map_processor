#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_processor/fetch.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_fetch");
    ros::NodeHandle n;
    ros::ServiceClient fetch_client = n.serviceClient<map_processor::fetch>("map_fetch", 1000);

    map_processor::fetch p;
    fetch_client.call(p);

    for(unsigned int i = 0; i < p.response.results.size(); ++i) {
        std::cout << p.response.results[i] << ' ' << p.response.xs[i] << ' ' << p.response.ys[i] << '\n';
    }

    std::cout << std::endl;

    return 0;
}