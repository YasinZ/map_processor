#ifndef __MAP_PROCESSOR_H__
#define __MAP_PROCESSOR_H__

#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include "nav_msgs/OccupancyGrid.h"
#include "map_processor/process.h"
#include "map_processor/fetch.h"
#include "map_processor/path.h"
#include <queue>

struct Checkpoint { std::string name; geometry_msgs::PoseStamped::ConstPtr ps; };
struct Point { int x, y; };
struct Node { Point pt; int dist; std::vector<Point> path; };

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

Cell pointCell (const nav_msgs::MapMetaData& info, const geometry_msgs::Point& p)
{
   tf::Point pt;
   tf::pointMsgToTF(p, pt);
   tf::Point p2 = worldToMap(info)*pt;
   return Cell(floor(p2.x()/info.resolution), floor(p2.y()/info.resolution));
}

const int rowNum[] = {-1, 0, 0, 1};
const int colNum[] = {0, -1, 1, 0};

nav_msgs::OccupancyGrid::ConstPtr _grid;
geometry_msgs::PoseStamped::ConstPtr _ps;
bool showPath = false;

std::vector<Checkpoint> checkpoints;
Node last;

bool IsValid(int row, int col) {
  return (row >= 0) && (row < _grid->info.width) &&
         (col >= 0) && (col < _grid->info.height);
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

      if(IsValid(row, col) && _grid->data[col * _grid->info.height + row] == 0 && !visited[row][col]) 
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

// Map saver
void save();

#endif
