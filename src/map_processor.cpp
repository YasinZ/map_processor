
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/tfMessage.h"
#include <sstream>
#include <fstream>
#include <cmath>
#include "map_processor/save.h"
#include "map_processor.h"

bool mapSaveCallback(map_processor::save::Request& req,
                     map_processor::save::Response& resp)
{
  save();
  return true;
}

void save() {
  std::string mapdatafile = "map.ppm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P6\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
          _grid->info.resolution, _grid->info.width, _grid->info.height);
  for(unsigned int y = 0; y < _grid->info.height; y++) {
    for(unsigned int x = 0; x < _grid->info.width; x++) {
      unsigned int i = x + (_grid->info.height - y - 1) * _grid->info.width;
      if (_grid->data[i] >= 0 && _grid->data[i] <= 0) { // [0,free)
        Cell position = pointCell(_grid->info, _ps->pose.position);
        if(position.x == x && position.y == _grid->info.height -y - 1) {
          fputc(0, out);
          fputc(0, out);
          fputc(255, out);
          continue;
        } else {
          fputc(254, out);
          fputc(254, out);
          fputc(254, out);
        }

        // if(showPath) {
        //   bool found = false;
        //   for(Point pt: last.path) {
        //     if(pt.x == x && pt.y == _grid->info.height - y - 1) {
        //       fputc(111, out);
        //       fputc(222, out);
        //       fputc(055, out);
        //       found = true;
        //       break;
        //     }
        //   }

        //   if(!found) {
        //     fputc(254, out);
        //     fputc(254, out);
        //     fputc(254, out);
        //   }
        // } else {
        //   fputc(111, out);
        //   fputc(111, out);
        //   fputc(111, out);
        // }
      } else if (_grid->data[i] >= 100) { // (occ,255]
        fputc(000, out);
        fputc(000, out);
        fputc(000, out);
      } else { //occ [0.25,0.65]
        fputc(205, out);
        fputc(205, out);
        fputc(205, out);
      }
    }
  }

  fclose(out);
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
  // save();
  o << oss.str().c_str();
  o.close();
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
  ros::ServiceServer save_server = n.advertiseService("map_save", mapSaveCallback);

  ros::spin();

  return 0;
}

// catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1