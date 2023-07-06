//
// Created by judith on 09.03.19.
//


#include <bitbots_localization/ObservationModel.h>

namespace bitbots_localization {

RobotPoseObservationModel::RobotPoseObservationModel(std::shared_ptr<Map> map_lines,
                                                     std::shared_ptr<Map> map_goals,
                                                     std::shared_ptr<Map> map_field_boundary,
                                                     std::shared_ptr<bl::Config> config)
    : particle_filter::ObservationModel<RobotState>() {
  map_lines_ = map_lines;
  map_goals_ = map_goals;
  map_field_boundary_ = map_field_boundary;
  config_ = config;
  particle_filter::ObservationModel<RobotState>::accumulate_weights_ = true;
}

double RobotPoseObservationModel::calculate_particle_line_mask_weight(
    const RobotState &state,
    const cv::Mat &last_measurement,
    std::shared_ptr<Map> map,
    double element_weight) const {
  cv::Point2f points[4];
  points[0] = cv::Point2f(0, 0);
  points[1] = cv::Point2f(1100, 0);
  points[2] = cv::Point2f(1100, 800);
  points[3] = cv::Point2f(0, 800);
  
  cv::Point2f transformed_points[4];
  cv::getPerspectiveTransform(points, transformed_points);
  return 0;
}

double RobotPoseObservationModel::calculate_weight_for_class(
    const RobotState &state,
    const std::vector<std::pair<double, double>> &last_measurement,
    std::shared_ptr<Map> map,
    double element_weight) const {
  double particle_weight_for_class = 1;
  if (!last_measurement.empty()) {
    std::vector<double> ratings = map->Map::provideRating(state, last_measurement);
    for (double rating : ratings) {
      particle_weight_for_class *= (1 - element_weight) + element_weight * (rating/100);
    }
  } else {
    particle_weight_for_class = 0;
  }
  return particle_weight_for_class;
}

double RobotPoseObservationModel::measure(const RobotState &state) const {
  double particle_weight_lines = calculate_weight_for_class(
    state,
    last_measurement_lines_,
     map_lines_,
     config_->line_element_confidence);
  double particle_weight_line_mask = calculate_particle_line_mask_weight(
    state,
    last_measurement_line_mask_,
    map_lines_,
    config_->line_mask_element_confidence);
  double particle_weight_goal = calculate_weight_for_class(
    state,
    last_measurement_goal_,
    map_goals_,
    config_->goal_element_confidence);
  double particle_weight_field_boundary = calculate_weight_for_class(
    state,
    last_measurement_field_boundary_,
    map_field_boundary_,
    config_->field_boundary_element_confidence);

  double weight = (
      ((1 - config_->lines_factor) + config_->lines_factor * particle_weight_lines) *
      ((1 - config_->line_mask_factor) + config_->line_mask_factor * particle_weight_line_mask) *
      ((1 - config_->goals_factor) + config_->goals_factor * particle_weight_goal) *
      ((1 - config_->field_boundary_factor) + config_->field_boundary_factor * particle_weight_field_boundary)
  );

  if (weight < min_weight_) {
    weight = min_weight_;
  }


  // reduce weight if particle is too far outside of the field:
  float range = config_->out_of_field_range;
  if ( state.getXPos() > (config_->field_x + config_->field_padding)/2 + range
    || state.getXPos() < -(config_->field_x + config_->field_padding)/2 - range
    || state.getYPos() > (config_->field_y + config_->field_padding)/2 + range
    || state.getYPos() < -(config_->field_y + config_->field_padding)/2 - range){
    weight = weight - config_->out_of_field_weight_decrease;
  }

  return weight; //exponential?
}

void RobotPoseObservationModel::set_measurement_lines_pc(sm::msg::PointCloud2 measurement){
  for (sm::PointCloud2ConstIterator<float> iter_xyz(measurement, "x"); iter_xyz != iter_xyz.end(); ++iter_xyz)
  {
    std::pair<double, double> linePolar = cartesianToPolar(iter_xyz[0], iter_xyz[1]);
    last_measurement_lines_.push_back(linePolar);
  }
}

void RobotPoseObservationModel::set_measurement_line_mask(sm::msg::Image measurement) {
  last_measurement_line_mask_ = cv_bridge::toCvCopy(measurement, "mono8")->image;
}

void RobotPoseObservationModel::set_measurement_goalposts(sv3dm::msg::GoalpostArray measurement) {
  // convert to polar
  for (sv3dm::msg::Goalpost &post : measurement.posts) {
    std::pair<double, double> postPolar = cartesianToPolar(post.bb.center.position.x, post.bb.center.position.y);
    last_measurement_goal_.push_back(postPolar);
  }
}

void RobotPoseObservationModel::set_measurement_field_boundary(sv3dm::msg::FieldBoundary measurement) {
  // convert to polar
  for (gm::msg::Point &point : measurement.points) {
    std::pair<double, double> fieldBoundaryPointPolar = cartesianToPolar(point.x, point.y);
    last_measurement_field_boundary_.push_back(fieldBoundaryPointPolar);
  }
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_lines() const {
  return last_measurement_lines_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_goals() const {
  return last_measurement_goal_;
}

std::vector<std::pair<double, double>> RobotPoseObservationModel::get_measurement_field_boundary() const {
  return last_measurement_field_boundary_;
}

void RobotPoseObservationModel::set_min_weight(double min_weight) {
  min_weight_ = min_weight;
}

double RobotPoseObservationModel::get_min_weight() const {
  return min_weight_;
}

void RobotPoseObservationModel::set_cof_transform(geometry_msgs::msg::Transform transform) {
  cof_transform_ = transform;
}

void RobotPoseObservationModel::clear_measurement() {
  last_measurement_lines_.clear();
  last_measurement_goal_.clear();
  last_measurement_field_boundary_.clear();
}

bool RobotPoseObservationModel::measurements_available() {
  bool available = false;
  available |= !last_measurement_lines_.empty();
  available |= !last_measurement_goal_.empty();
  available |= !last_measurement_field_boundary_.empty();
  return available;
}
}
