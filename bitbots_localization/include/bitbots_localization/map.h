//
// Created by judith on 08.03.19.
//

#ifndef BITBOTS_LOCALIZATION_MAP_H
#define BITBOTS_LOCALIZATION_MAP_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <bitbots_localization/RobotState.h>
#include <bitbots_localization/LocalizationConfig.h>
#include <geometry_msgs/Point.h>
#include <bitbots_localization/tools.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>

namespace gm = geometry_msgs;
namespace bl = bitbots_localization;


/**
* @class Map
* @brief Stores a map for a messurement class (e.g. a map of the lines)  
*/
class Map {
 public:

  /**
   * @param name of the environment. (E.g. webots)
   * @param type of the map. (E.g. lines)
   * @param config Config of the localization.
   */ 
  explicit Map(const std::string& file_path, const bl::LocalizationConfig &config);
  explicit Map(const std::string& name, const std::string& type, const bl::LocalizationConfig &config);

  cv::Mat map;

  std::vector<double> provideRating(const RobotState &state,
                                    const std::vector<std::pair<double, double>> &observations);

  double get_occupancy(double x, double y);

  std::pair<double, double> observationRelative(std::pair<double, double> observation,
                                                double stateX,
                                                double stateY,
                                                double stateT);

 private:
     bl::LocalizationConfig config_;

};

#endif //BITBOTS_LOCALIZATION_MAP_H
