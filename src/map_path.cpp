#include "ros/ros.h"
#include "std_msgs/String.h"
#include "map_processor/path.h"

#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_path");
    ros::NodeHandle n;
    ros::ServiceClient path_client = n.serviceClient<map_processor::path>("map_path", 1000);

    std::stringstream ss;
    ss << argv[1];
    map_processor::path p;
    p.request.request = ss.str();
    path_client.call(p);

    if(p.response.x[0] == -1 && p.response.y[0] == -1 && p.response.x.size() == 1 && p.response.y.size() == 1) {
        std::cout << "Failed to find path" << std::endl;
        return 1;
    }

    for(unsigned int i = 0; i < p.response.x.size(); ++i) {
        std::cout << p.response.x[i] << ' ' << p.response.y[i] << '\n';
    }

    std::cout << std::endl;

    return 0;
}