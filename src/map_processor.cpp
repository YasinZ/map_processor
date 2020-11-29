
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Polygon.h"
#include "map_processor/process.h"
#include "map_processor/fetch.h"
#include <tf/transform_datatypes.h>
#include "map_processor/path.h"
#include <sstream>
#include <queue>
#include <fstream>
#include <cmath>

// AMCL
nav_msgs::OccupancyGrid::ConstPtr _grid;
geometry_msgs::PoseStamped::ConstPtr _ps;
bool showPath = false;

struct Checkpoint { std::string name; geometry_msgs::PoseStamped::ConstPtr ps; };
struct Point { int x, y; };
struct Node { Point pt; int dist; std::vector<Point> path; };
std::vector<Checkpoint> checkpoints;
Node last;

typedef int16_t coord_t;

tf::Transform mapToWorld (const nav_msgs::MapMetaData& info)
{
  tf::Transform world_to_map;
  tf::poseMsgToTF (info.origin, world_to_map);
  return world_to_map;
}

tf::Transform worldToMap (const nav_msgs::MapMetaData& info)
{
  return mapToWorld(info).inverse();
}
 
struct Cell
{
   Cell(const coord_t x=0, const coord_t y=0): x(x), y(y) {}
   coord_t x;
   coord_t y;
 
   bool operator== (const Cell& c) const;
   bool operator< (const Cell& c) const;
};

bool IsValid(int row, int col) {
  return (row >= 0) && (row < _grid->info.width) &&
         (col >= 0) && (col < _grid->info.height);
}

int rowNum[] = {-1, 0, 0, 1};
int colNum[] = {0, -1, 1, 0};

Node BFS(Point src, Point dest);

Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
   tf::Point pt;
   tf::pointMsgToTF(p, pt);
   tf::Point p2 = worldToMap(info)*pt;
   return Cell(floor(p2.x()/info.resolution), floor(p2.y()/info.resolution));
}

void slam_out_poseCallback(const geometry_msgs::PoseStamped::ConstPtr& ps) {
  _ps = ps;
  ROS_INFO("PS %f %f %f", ps->pose.position.x, ps->pose.position.y, ps->pose.position.z);

  std::ostringstream oss;
  int height = _grid->info.height;
  int width = _grid->info.width;
  ROS_INFO("Width [%d] Height [%d] size [%zu]", width, height, _grid->data.size());
  ROS_INFO("Origin x [%f] y [%f] resolution [%f]", _grid->info.origin.position.x, _grid->info.origin.position.y, _grid->info.resolution);

  Cell position = pointCell(_grid->info, _ps->pose.position);

  ROS_INFO("Test x [%d] Test y [%d]\n", position.x, position.y);

  for(int i = 0; i < height; i++) {
    bool hasAny = false;

    for(int j = 0; j < width; j++) {
      if(int(_grid->data[j * width + i]) != -1) {
        hasAny = true;
      }
    }

    if(hasAny) {
      oss << i << ' ';
      for(int j = 0; j < width; j++) {
        if(i == position.x && j == position.y) {
            oss << "B" << ' ';
        }

        if(showPath) {
          for(Point pt: last.path) {
            if(pt.x == i && pt.y == j)
              oss << "P" << ' ';
          }
        }  

        int value = int(_grid->data[j * width + i]);
        oss << (value == 100 ? 1 : (value == -1 ? 2 : value)) << ' ';

      }
    }

    if(hasAny)
      oss << "\n";  
  }
  
  std::ofstream o("src/map_processor/src/map.txt");
  o << oss.str().c_str();
  o.close();
  // ROS_INFO("MAP:\n%s", oss.str().c_str());
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
}

bool mapCheckpointCallback(map_processor::process::Request& req, 
                           map_processor::process::Response& resp) {
  checkpoints.push_back( Checkpoint{req.msg, _ps} );

  for(auto checkpoint: checkpoints) {
    // ROS_INFO("Checkpoint name: %s", checkpoint.name.c_str());
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

bool mapPathCallback(map_processor::path::Request& req,
                     map_processor::path::Response& resp) {

  for(auto checkpoint: checkpoints) {
    if(checkpoint.name == req.request) {
      Cell src = pointCell(_grid->info, _ps->pose.position);
      Cell dest = pointCell(_grid->info, checkpoint.ps->pose.position);
      
      Node node = BFS(Point{src.x, src.y}, Point{dest.x, dest.y});
      if(node.dist == -1) {
        resp.x.push_back(-1);
        resp.y.push_back(-1);
        break;
      }

      last = node;
      showPath = true;
      for(auto p: node.path) {
        resp.x.push_back(p.x);
        resp.y.push_back(p.y);
      }
      break;
    }
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
  ros::ServiceServer path_server = n.advertiseService("map_path", mapPathCallback);

  ros::spin();

  return 0;
}

// catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1