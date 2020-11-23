
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include "map_processor/process.h"
#include "map_processor/fetch.h"
#include <sstream>
#include <queue>
#include <fstream>
#include <cmath>

// AMCL
nav_msgs::OccupancyGrid::ConstPtr _grid;
geometry_msgs::PoseStamped::ConstPtr _ps;

struct Checkpoint { std::string name; geometry_msgs::PoseStamped::ConstPtr ps; };
struct Point { int x, y; };
struct Node { Point pt; int dist; std::vector<Point> path; };
std::vector<Checkpoint> checkpoints;

bool IsValid(int row, int col) {
  return (row >= 0) && (row < _grid->info.width) &&
         (col >= 0) && (col < _grid->info.height);
}

int rowNum[] = {-1, 0, 0, 1};
int colNum[] = {0, -1, 1, 0};

Node BFS(Point src, Point dest);

void slam_out_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& ps) {
  _ps = ps;
  ROS_INFO("PS %f %f %f", ps->pose.position.x, ps->pose.position.y, ps->pose.position.z);

  std::ostringstream oss;
  int height = _grid->info.height;
  int width = _grid->info.width;
  ROS_INFO("Width [%d] Height [%d] size [%zu]", width, height, _grid->data.size());
  ROS_INFO("Origin x [%f] y [%f] resolution [%f]", _grid->info.origin.position.x, _grid->info.origin.position.y, _grid->info.resolution);

  unsigned int grid_x = (unsigned int) (ps->pose.position.x - _grid->info.origin.position.x) / _grid->info.resolution;
  unsigned int grid_y = (unsigned int) (ps->pose.position.y - _grid->info.origin.position.y) / _grid->info.resolution;

  // index = floor((x - x_origin)/resolution) + floor((y - y_origin)/resolution)*map_width
  // x = (index % width) * resolution + x_origin
  // y = ((index - (index % width)) / resolution) + y_origin
  // unsigned int index = (unsigned int) (floor((ps->pose.position.x - _grid->info.origin.position.x) / _grid->info.resolution)) +
  //                      (unsigned int) (floor((ps->pose.position.y - _grid->info.origin.position.y) / _grid->info.resolution)) * _grid->info.width;

  // unsigned int _x = (unsigned int) ((index % _grid->info.width) * _grid->info.resolution + _grid->info.origin.position.x);
  // unsigned int _y = (unsigned int) (((index - (index % _grid->info.width)) / _grid->info.resolution) + _grid->info.origin.position.y);

  ROS_INFO("Grid_X x [%d] Grid_Y y [%d]", grid_x, grid_y);
  // ROS_INFO("index [%d] _x [%d] _y [%d]", index, _x, _y);

  // Node last = BFS({(int)grid_x + 10, (int)grid_y + 15}, {(int)grid_x - 60, (int)grid_y + 50});
  // ROS_INFO("DISTANCE: %d", last.dist);
  // ROS_INFO("LAST x: %d, y: %d", last.pt.x, last.pt.y);
  // do {
  //   ROS_INFO("LAST x: %d, y: %d", last->pt.x, last->pt.y);
  //   last = last->prev;
  // }while(last->prev != nullptr);
  // for(Point pt: last.path) {
  //   ROS_INFO("PATH: x %d y %d", pt.x, pt.y);
  // }

  for(int i = 0; i < width; i++) {
    bool hasAny = false;

    for(int j = 0; j < height; j++) {
      int countIgnore = 0;
      if(int(_grid->data[i * width + j]) != -1) {
        hasAny = true;
      }
      if(int(_grid->data[i * width + j]) == -1 && !hasAny) {
        countIgnore++;
      }
    }

    if(hasAny) {
      for(int j = 0; j < height; j++) {
        if(i == grid_x && j == grid_y) {
            oss << "A" << ' ';
        }

        if(i == grid_x + 10 && j == grid_y + 15) {
            oss << "DDD" << ' ';
        }

        if(i == grid_x - 60 && j == grid_y + 50) {
            oss << "EEE" << ' ';
        }

        // for(Point pt: last.path) {
        //   if(pt.x == i && pt.y == j)
        //     oss << "!" << ' ';
        // }
        oss << int(_grid->data[i * width + j]) << ' ';
      }
    }

    if(hasAny)
      oss << "\n";  
  }
  
  std::ofstream o("src/map_processor/src/map.txt");
  o << oss.str().c_str();
  o.close();
  // ROS_INFO("MAP:\n%s", oss.str().c_str());
  ROS_INFO("END\n");
}

Node BFS(Point src, Point dest) {
  // ASSUME DESTINATION AND SOURCE ARE VALID
  bool **visited = new bool *[_grid->info.height];
  for(int i = 0; i < _grid->info.height; i++) {
    visited[i] = new bool[_grid->info.height];
    for(int j = 0; j < _grid->info.height; j++)
      visited[i][j] = false;
  }

  visited[src.x][src.y] = true;
  std::queue<Node> q;

  Node s = {src, 0};
  q.push(s);

  while (!q.empty()) {
    Node current = q.front();
    Point point = current.pt;

    if(point.x == dest.x && point.y == dest.y) {
      return current;
    }
    
    q.pop();

    for(int i = 0; i < 4; i++) {
      int row = point.x + rowNum[i];
      int col = point.y + colNum[i];

      if(IsValid(row, col) && _grid->data[row * _grid->info.height + col] == 0 && !visited[row][col]) 
      {
        visited[row][col] = true;
        std::vector<Point> newPath = current.path;
        newPath.push_back({row, col});
        Node adj = { {row, col}, current.dist + 1, newPath};
        q.push(adj);
      }
    }
  }

  return Node{ {-1, -1}, -1};
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
  _grid = grid;
  {
  // std::ostringstream oss;
  // int height = grid->info.height;
  // int width = grid->info.width;
  // ROS_INFO("Width [%d] Height [%d] size [%d]", width, height, grid->data.size());
  // ROS_INFO("Origin x [%f] y [%f] resolution [%f]", grid->info.origin.position.x, grid->info.origin.position.y, grid->info.resolution);

  // for(int i = 0; i < width; i++) {
  //   bool hasAny = false;

  //   for(int j = 0; j < height; j++) {
  //     if(int(grid->data[i * width + j]) != -1) {
  //       hasAny = true;
  //     }
  //   }

  //   if(hasAny) {
  //     bool printedCharacter = false;
  //     for(int j = 0; j < height; j++) {
  //       if(i == 988 && j == 1036) {
  //           oss << "ASDF" << ' ';
  //       }
  //       if(int(grid->data[i * width + j]) != -1) {
  //           printedCharacter = true;
  //           oss << int(grid->data[i * width + j]) << ' ';
  //       }
  //       // else {
  //       //   oss << ' ';
  //       // }
  //       // if(!printedCharacter && j > 150)
  //         // oss << ' ';
  //     }
  //   }

  //   if(hasAny)
  //     oss << "\n";  
  // }
  // ROS_INFO("MAP:\n%s", oss.str().c_str());
  // ROS_INFO("END\n");
  }
}

bool mapCheckpointCallback(map_processor::process::Request& req, 
                           map_processor::process::Response& resp) {
  checkpoints.push_back( Checkpoint{req.msg, _ps} );

  for(auto checkpoint: checkpoints) {
    ROS_INFO("Checkpoint name: %s", checkpoint.name.c_str());
    resp.results.push_back(checkpoint.name);
  }

  return true;
}

bool mapFetchCallback(map_processor::fetch::Request& req, 
                           map_processor::fetch::Response& resp) {
  for(auto checkpoint: checkpoints) {
    resp.results.push_back(checkpoint.name);
    resp.xs.push_back(checkpoint.ps->pose.position.x);
    resp.ys.push_back(checkpoint.ps->pose.position.y);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_processor");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 1000, mapCallback);
  ros::Subscriber sub2 = n.subscribe("slam_out_pose", 1000, slam_out_poseCallback);
  ros::ServiceServer checkpoint_server = n.advertiseService("map_checkpoint", mapCheckpointCallback);
  ros::ServiceServer fetch_server = n.advertiseService("map_fetch", mapFetchCallback);

  ROS_INFO("STARTED");
  ros::spin();

  return 0;
}

// catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1