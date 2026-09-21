/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSystem.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/PointCloud.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/dataset_reader.h"
#include "utils/opencv_yaml_parse.h"
#include <memory>

#if ROS_AVAILABLE == 2
#include "core/ROS2Helper.h"
#include "core/ROS2Publisher.h"
#include "sim/Sim2Visualizer.h"
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#elif ROS_AVAILABLE == 1
#include "core/ROSHelper.h"
#include "core/ROSPublisher.h"
#include "sim/SimVisualizer.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#endif

using namespace std;
using namespace mins;

#if ROS_AVAILABLE == 2
// Message types used below, so the playback logic reads the same for ROS1 and ROS2
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::PointCloud2;

namespace mins {
// ROS2 counterparts of the ROS1 classes used below (SimVisualizer is forward-declared in Simulator.h, so it cannot be aliased)
using ROSHelper = ROS2Helper;
using ROSPublisher = ROS2Publisher;

// rosbag2 renamed SerializedBagMessage::time_stamp to recv_timestamp (Jazzy). Pick whichever exists.
template <typename M> auto bag_stamp(const M &m, int) -> decltype(m.recv_timestamp) { return m.recv_timestamp; }
template <typename M> auto bag_stamp(const M &m, long) -> decltype(m.time_stamp) { return m.time_stamp; }

/**
 * @brief Minimal stand-in for rosbag::MessageInstance.
 * Holds the serialized rosbag2 message and deserializes it on request, so the ROS1 playback logic below works unchanged.
 * The bytes are copied out of the rosbag2 message: the storage plugin that allocated it gets unloaded by a class_loader
 * static destructor before this file's globals are destroyed, so keeping plugin-owned objects alive would crash on exit.
 */
class MessageInstance {
public:
  MessageInstance(const rosbag2_storage::SerializedBagMessage &msg, std::string type)
      : topic(msg.topic_name), type(std::move(type)), time(bag_stamp(msg, 0)), serialized(std::make_shared<rclcpp::SerializedMessage>(*msg.serialized_data)) {}

  const std::string &getTopic() const { return topic; }

  rclcpp::Time getTime() const { return time; }

  /// Deserialize as T. Returns nullptr if the topic type is not T (same contract as rosbag::MessageInstance::instantiate)
  template <typename T> std::shared_ptr<T> instantiate() const {
    if (type != rosidl_generator_traits::name<T>())
      return nullptr;
    auto data = std::make_shared<T>();
    rclcpp::Serialization<T>().deserialize_message(serialized.get(), data.get());
    return data;
  }

private:
  std::string topic, type;
  rclcpp::Time time;
  std::shared_ptr<const rclcpp::SerializedMessage> serialized; // shared between 'view' and 'msgs' copies
};
} // namespace mins

// Header stamp helpers (builtin_interfaces::msg::Time has no toSec())
inline double stamp_to_sec(const builtin_interfaces::msg::Time &t) { return t.sec + t.nanosec * 1e-9; }
inline bool stamp_is_zero(const builtin_interfaces::msg::Time &t) { return t.sec == 0 && t.nanosec == 0; }
#elif ROS_AVAILABLE == 1
using geometry_msgs::PoseStamped;
using nav_msgs::Odometry;
using sensor_msgs::CompressedImage;
using sensor_msgs::Image;
using sensor_msgs::Imu;
using sensor_msgs::JointState;
using sensor_msgs::NavSatFix;
using sensor_msgs::PointCloud2;

inline double stamp_to_sec(const ros::Time &t) { return t.toSec(); }
inline bool stamp_is_zero(const ros::Time &t) { return t == ros::Time(0); }
#endif

shared_ptr<SystemManager> sys;
shared_ptr<ROSPublisher> pub;
#if ROS_AVAILABLE == 2
shared_ptr<Sim2Visualizer> sim_viz;
#elif ROS_AVAILABLE == 1
shared_ptr<SimVisualizer> sim_viz;
#endif
shared_ptr<Options> op;
shared_ptr<State_Logger> save;

#if ROS_AVAILABLE == 2
vector<MessageInstance> view; // messages on the topics we use, from all bags, in time order
vector<MessageInstance> msgs;
rclcpp::Time time_init, time_finish;
rclcpp::Time bag_begin, bag_end; // time span of all bags (all topics), as rosbag::View reports it
#elif ROS_AVAILABLE == 1
rosbag::View view;
vector<rosbag::Bag> bags; // Changed to vector to support multiple bags
vector<rosbag::MessageInstance> msgs;
ros::Time time_init, time_finish;
#endif
vector<map<double, int>> cam_map; // {cam[i], {img_time, idx in vec}}
vector<int> used_index;

// read parameters, init system, build map('cam_map') for cam msg ptr
void system_setup(int argc, char **argv);
// find the closest stereo pair's image msg ptr based on map('cam_map')
bool find_stereo_pair(double meas_t, int cam_id, int &idx);
// feed the camera measurement to the system. automatically find stereo pair measurement if it is
bool feed_camera(int cam_id, int idx);

// Main function
int main(int argc, char **argv) {

  // Load parameters
  system_setup(argc, argv);

  // process measurements
  for (int i = 0; i < (int)msgs.size(); i++) {
#if ROS_AVAILABLE == 2
    if (!rclcpp::ok() || msgs.at(i).getTime() > time_finish)
#elif ROS_AVAILABLE == 1
    if (!ros::ok() || msgs.at(i).getTime() > time_finish)
#endif
      break;

    // skip until start time
    if (msgs.at(i).getTime() < time_init)
      continue;

    // skip used measurements. Usually stereo img
    if (find(used_index.begin(), used_index.end(), i) != used_index.end()) {
      used_index.erase(std::remove(used_index.begin(), used_index.end(), i), used_index.end());
      continue;
    }

    // ===================== IMU =====================
    if (msgs.at(i).getTopic() == op->est->imu->topic) {
      ov_core::ImuData imu = ROSHelper::Imu2Data(msgs.at(i).instantiate<Imu>());
      PRINT1(GREEN "[BAG] IMU measurement: %.3f" RESET, imu.timestamp);
      PRINT1(GREEN "|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
      bool visualize = sys->feed_measurement_imu(imu);
      pub->publish_imu();
      if (visualize) {
        pub->visualize();
        sim_viz->publish_groundtruth();
        op->sys->save_trajectory ? save->save_trajectory_to_file(sys) : void();
      }
      continue;
    }

    // ===================== CAM =====================
    if (op->est->cam->enabled) {
      for (int cam_id = 0; cam_id < op->est->cam->max_n; cam_id++) {
        if (msgs.at(i).getTopic() == op->est->cam->topic.at(cam_id)) {
          if (feed_camera(cam_id, i)) // this also adds index to [used_index] if stereo
            pub->publish_cam_images({cam_id, i});
          continue;
        }
      }
    }

    // ===================== WHEEL =====================
    if (op->est->wheel->enabled && msgs.at(i).getTopic() == op->est->wheel->topic) {
      auto wheel_j = msgs.at(i).instantiate<JointState>();
      auto wheel_o = msgs.at(i).instantiate<Odometry>();

      WheelData data;
      if (wheel_j != nullptr) {
        data = ROSHelper::JointState2Data(wheel_j);
      } else {
        data = ROSHelper::Odometry2Data(wheel_o);
      }

      PRINT1(MAGENTA "[BAG] WHL measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
      sys->feed_measurement_wheel(data);
      continue;
    }

    // ===================== GPS =====================
    if (op->est->gps->enabled) {
      for (int gps_id = 0; gps_id < op->est->gps->max_n; gps_id++) {
        if (msgs.at(i).getTopic() == op->est->gps->topic.at(gps_id)) {
          auto ptr_fix = msgs.at(i).instantiate<NavSatFix>();
          auto ptr_geo = msgs.at(i).instantiate<PoseStamped>();
          GPSData data = (ptr_fix != nullptr ? ROSHelper::NavSatFix2Data(ptr_fix, gps_id) : ROSHelper::PoseStamped2Data(ptr_geo, gps_id, op->est->gps->noise));
          // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
          data.noise(0) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(0) = op->est->gps->noise : double();
          data.noise(1) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(1) = op->est->gps->noise : double();
          data.noise(2) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(2) = op->est->gps->noise * 2 : double();
          PRINT1(CYAN "[BAG] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
          PRINT1(CYAN "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
          sys->feed_measurement_gps(data, ptr_fix != nullptr);
          pub->publish_gps(data, ptr_fix != nullptr);
          continue;
        }
      }
    }

    // ==================== LiDAR ====================
    if (op->est->lidar->enabled) {
      for (int lidar_id = 0; lidar_id < op->est->lidar->max_n; lidar_id++) {
        if (msgs.at(i).getTopic() == op->est->lidar->topic.at(lidar_id)) {
          std::shared_ptr<mins::PointCloud<mins::PointXYZ>> data = ROSHelper::rosPC2pclPC(msgs.at(i).instantiate<PointCloud2>(), lidar_id);
          PRINT1("[BAG] LDR measurement: %.3f|%d|%d\n", (double)data->header.stamp / 1000, lidar_id, data->points.size());
          sys->feed_measurement_lidar(data);
          pub->publish_lidar_cloud(data);
          continue;
        }
      }
    }

    // ==================== VICON ====================
    if (op->est->vicon->enabled) {
      for (int vicon_id = 0; vicon_id < op->est->vicon->max_n; vicon_id++) {
        if (msgs.at(i).getTopic() == op->est->vicon->topic.at(vicon_id)) {
          ViconData data = ROSHelper::PoseStamped2Data(msgs.at(i).instantiate<PoseStamped>(), vicon_id);
          PRINT1("[BAG] VCN measurement: %.3f|%d|", data.time, data.id);
          PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", data.pose(0), data.pose(1), data.pose(2), data.pose(3), data.pose(4), data.pose(5));
          sys->feed_measurement_vicon(data);
          pub->publish_vicon(data);
          continue;
        }
      }
    }
  }

  // Final visualization
  sys->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();

#if ROS_AVAILABLE == 2
  rclcpp::shutdown();
#elif ROS_AVAILABLE == 1
  ros::shutdown();
#endif

  // Done!
  return EXIT_SUCCESS;
}

bool feed_camera(int cam_id, int idx) {
  auto img_c = msgs.at(idx).instantiate<CompressedImage>();
  auto img_i = msgs.at(idx).instantiate<Image>();
  // In case the image does not have timestamp (then 0), overwrite it with message time.
  if (img_c != nullptr && stamp_is_zero(img_c->header.stamp))
    img_c->header.stamp = msgs.at(idx).getTime();
  if (img_i != nullptr && stamp_is_zero(img_i->header.stamp))
    img_i->header.stamp = msgs.at(idx).getTime();
  ov_core::CameraData cam;

  // MONO
  if (op->est->cam->stereo_pairs.find(cam_id) == op->est->cam->stereo_pairs.end()) {
    if (img_c != nullptr) {
      if (ROSHelper::Image2Data(img_c, cam_id, cam, op->est->cam)) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d\n" RESET, stamp_to_sec(img_c->header.stamp), cam_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images(cam_id);
      } else {
        PRINT3(YELLOW "Failed to convert image to a proper format.\n" RESET);
      }
    } else {
      if (ROSHelper::Image2Data(img_i, cam_id, cam, op->est->cam)) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d\n" RESET, stamp_to_sec(img_i->header.stamp), cam_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images(cam_id);
      } else {
        PRINT3(YELLOW "Failed to convert image to a proper format.\n" RESET);
      }
    }
    return true;
  }

  // STEREO - find stereo measurement
  int msgs_id; // this is index position in msgs vector
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);
  double meas_t = img_c != nullptr ? stamp_to_sec(img_c->header.stamp) : stamp_to_sec(img_i->header.stamp);
  if (find_stereo_pair(meas_t, cam_id, msgs_id)) {

    // should be always in the future
    if (msgs_id < idx)
      return false;

    // record this index
    used_index.push_back(msgs_id);

    // get stereo pair image
    if (img_c != nullptr) {
      bool success0 = ROSHelper::Image2Data(img_c, cam_id, cam, op->est->cam);
      bool success1 = ROSHelper::Image2Data(msgs.at(msgs_id).instantiate<CompressedImage>(), stereo_id, cam, op->est->cam);
      if (success0 && success1) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, stamp_to_sec(img_c->header.stamp), cam_id, stereo_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images({cam_id, stereo_id});
      }
    } else {
      bool success0 = ROSHelper::Image2Data(img_i, cam_id, cam, op->est->cam);
      bool success1 = ROSHelper::Image2Data(msgs.at(msgs_id).instantiate<Image>(), stereo_id, cam, op->est->cam);
      if (success0 && success1) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, stamp_to_sec(img_i->header.stamp), cam_id, stereo_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images({cam_id, stereo_id});
      }
    }
  } else {
    PRINT4(RED "Cannot find proper stereo pair of CAM%d!\n" RESET, cam_id);
    return false;
  }
  return true;
}

bool find_stereo_pair(double meas_t, int cam_id, int &idx) {
  // Get the stereo pair's cam id
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);

  // get lower bound of stereo pair's message
  assert(!cam_map.at(stereo_id).empty());
  auto pair_lb = cam_map.at(stereo_id).lower_bound(meas_t);

  // check one previous message
  if (distance(cam_map.at(stereo_id).begin(), pair_lb) > 0) {
    // get lb and its prev measurement info
    double t_1 = pair_lb->first;
    int idx_1 = pair_lb->second;
    pair_lb--;
    double t_0 = pair_lb->first;
    int idx_0 = pair_lb->second;

    // get the closest index
    idx = abs(meas_t - t_0) < abs(meas_t - t_1) ? idx_0 : idx_1;
  } else {
    // this is the closest measurement you can get
    idx = pair_lb->second;
  }

  // filter too large time gap
  auto stereo_img_c = msgs.at(idx).instantiate<CompressedImage>();
  auto stereo_img_i = msgs.at(idx).instantiate<Image>();
  if (stereo_img_c != nullptr)
    return abs(stamp_to_sec(stereo_img_c->header.stamp) - meas_t) < 0.01;
  else if (stereo_img_i != nullptr)
    return abs(stamp_to_sec(stereo_img_i->header.stamp) - meas_t) < 0.01;
  else {
    PRINT4(RED "Image topic has unmatched message types!. Exiting.\n" RESET);
    exit(EXIT_FAILURE);
  }
}

void system_setup(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  string config_path = "unset_path_to_config.yaml";
  argc > 1 ? config_path = argv[1] : string();

  // Launch our ros node
#if ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("mins_bag", options);
  node->get_parameter<std::string>("config_path", config_path);
#elif ROS_AVAILABLE == 1
  ros::init(argc, argv, "mins_bag");
  auto nh = make_shared<ros::NodeHandle>("~");
  nh->param<string>("config_path", config_path, config_path);
#endif

  // Init options
  auto parser = make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 2
  parser->set_node(node);
#elif ROS_AVAILABLE == 1
  parser->set_node_handler(nh);
#endif
  op = make_shared<Options>();
  op->load_print(parser);
  op->sys->save_prints ? mins::Print_Logger::open_file(op->sys->path_state, true) : void();

  sys = make_shared<SystemManager>(op->est);
#if ROS_AVAILABLE == 2
  pub = make_shared<ROSPublisher>(node, sys, op);
  sim_viz = make_shared<Sim2Visualizer>(node, sys);
  // Release the publishers before static destruction (also on the exit() error paths below). The image_transport
  // publishers are pluginlib objects, and class_loader unloads their plugin from a function-local static whose destructor
  // is registered when the first plugin loads (inside the ROSPublisher constructor), i.e. after this file's globals were
  // constructed, so it would run before them and their destructors would crash. Registered after 'pub', it runs first.
  std::atexit([]() {
    pub.reset();
    sim_viz.reset();
  });
#elif ROS_AVAILABLE == 1
  pub = make_shared<ROSPublisher>(nh, sys, op);
  sim_viz = make_shared<SimVisualizer>(nh, sys);
#endif

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Load rosbag here, and find messages we can play
  // Parse multiple bag file paths separated by ":"
  PRINT2("[BAG] Parsing bag file paths\n");
  vector<string> bag_paths;
  string path_string = op->sys->path_bag;
  size_t path_start = 0;
  size_t path_end = path_string.find(":");
  
  // Split the string by ":" and add each path (except the last one)
  while (path_end != string::npos) {
    string path = path_string.substr(path_start, path_end - path_start);
    if (!path.empty()) {
      bag_paths.push_back(path);
      PRINT2("[BAG] Found bag path: %s\n", path.c_str());
    }
    path_start = path_end + 1;
    path_end = path_string.find(":", path_start);
  }
  
  // Add the last (or only) path
  string last_path = path_string.substr(path_start);
  if (!last_path.empty()) {
    bag_paths.push_back(last_path);
    PRINT2("[BAG] Found bag path: %s\n", last_path.c_str());
  }
  
  PRINT2("[BAG] Total bag files to process: %d\n", (int)bag_paths.size());
  
  // Open all bag files and add them to the view, which handles synchronization and merging messages from multiple bags
  PRINT2("[BAG] Reading bag files       ");
#if ROS_AVAILABLE == 2
  // rosbag2 has no lazy multi-bag view: read the messages of the topics we use into memory and merge them by time.
  // Only the topics we use are kept so we do not hold the whole bag in memory.
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(op->est->imu->topic);
  if (op->est->cam->enabled)
    filter.topics.insert(filter.topics.end(), op->est->cam->topic.begin(), op->est->cam->topic.end());
  if (op->est->wheel->enabled)
    filter.topics.push_back(op->est->wheel->topic);
  if (op->est->gps->enabled)
    filter.topics.insert(filter.topics.end(), op->est->gps->topic.begin(), op->est->gps->topic.end());
  if (op->est->lidar->enabled)
    filter.topics.insert(filter.topics.end(), op->est->lidar->topic.begin(), op->est->lidar->topic.end());
  if (op->est->vicon->enabled)
    filter.topics.insert(filter.topics.end(), op->est->vicon->topic.begin(), op->est->vicon->topic.end());

  int64_t begin_ns = numeric_limits<int64_t>::max(), end_ns = numeric_limits<int64_t>::min();
  for (size_t i = 0; i < bag_paths.size(); i++) {
    PRINT2("[BAG] Opening bag %d/%d: %s\n", (int)(i+1), (int)bag_paths.size(), bag_paths[i].c_str());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_paths[i];
    rosbag2_cpp::Reader reader;
    reader.open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    // time span of the whole bag (all topics), like rosbag::View::getBeginTime()/getEndTime()
    const rosbag2_storage::BagMetadata &meta = reader.get_metadata();
    int64_t bag_begin_ns = chrono::duration_cast<chrono::nanoseconds>(meta.starting_time.time_since_epoch()).count();
    begin_ns = min(begin_ns, bag_begin_ns);
    end_ns = max(end_ns, bag_begin_ns + (int64_t)meta.duration.count());

    // topic -> message type, needed to emulate rosbag::MessageInstance::instantiate
    map<string, string> topic_types;
    for (const rosbag2_storage::TopicMetadata &topic : reader.get_all_topics_and_types())
      topic_types[topic.name] = topic.type;

    reader.set_filter(filter);
    while (reader.has_next()) {
      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
      view.emplace_back(*msg, topic_types.at(msg->topic_name));
    }
  }
  PRINT2("[BAG] Successfully opened %d bag file(s)\n", (int)bag_paths.size());

  // merge the bags in time order (rosbag::View does this for us in ROS1)
  stable_sort(view.begin(), view.end(), [](const MessageInstance &a, const MessageInstance &b) { return a.getTime() < b.getTime(); });
  bag_begin = rclcpp::Time(begin_ns);
  bag_end = rclcpp::Time(end_ns);
#elif ROS_AVAILABLE == 1
  bags.resize(bag_paths.size());
  for (size_t i = 0; i < bag_paths.size(); i++) {
    PRINT2("[BAG] Opening bag %d/%d: %s\n", (int)(i+1), (int)bag_paths.size(), bag_paths[i].c_str());
    bags[i].open(bag_paths[i], rosbag::bagmode::Read);
    view.addQuery(bags[i]);
  }
  PRINT2("[BAG] Successfully opened %d bag file(s)\n", (int)bags.size());
#endif

  // Check to make sure we have data to play
  if (view.size() == 0) {
    PRINT4(RED "\nNo messages to play on specified topics. Exiting.\n" RESET);
#if ROS_AVAILABLE == 2
    rclcpp::shutdown();
#elif ROS_AVAILABLE == 1
    ros::shutdown();
#endif
    exit(EXIT_FAILURE);
  }
  PRINT2("[BAG] Total messages in view: %d\n", (int)view.size());

  // load rosbag msg itr
  msgs.reserve(view.size());
  cam_map = vector<map<double, int>>(op->est->cam->max_n);
  int cnt = 0;
  for (const auto &msg : view) {
#if ROS_AVAILABLE == 2
    if (!rclcpp::ok())
#elif ROS_AVAILABLE == 1
    if (!ros::ok())
#endif
      break;
    PRINT2("\b\b\b%02d%%", (int)((cnt++ * 100) / view.size()));

    // check IMU ========================================
    if (msg.getTopic() == op->est->imu->topic) {
      assert(msg.instantiate<Imu>() != nullptr);
      msgs.push_back(msg);
      continue;
    }

    // check CAM ========================================
    if (op->est->cam->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->cam->max_n; id++) {
        if (msg.getTopic() == op->est->cam->topic.at(id)) {
          auto img_c = msg.instantiate<CompressedImage>();
          auto img_i = msg.instantiate<Image>();
          if (img_c != nullptr)
            cam_map.at(id).insert({stamp_to_sec(img_c->header.stamp), msgs.size()});
          else if (img_i != nullptr)
            cam_map.at(id).insert({stamp_to_sec(img_i->header.stamp), msgs.size()});
          else {
            PRINT4(RED "\nImage topic has unmatched message types!. Exiting.\n" RESET);
            exit(EXIT_FAILURE);
          }
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check WHEEL ========================================
    if (op->est->wheel->enabled && msg.getTopic() == op->est->wheel->topic) {
      auto wheel_j = msg.instantiate<JointState>();
      auto wheel_o = msg.instantiate<Odometry>();
      assert(wheel_j != nullptr || wheel_o != nullptr);
      if (wheel_j != nullptr && op->est->wheel->sub_topics.at(0) == wheel_j->name.at(0))
        msgs.push_back(msg);
      else if (wheel_o != nullptr)
        msgs.push_back(msg);
      else {
        PRINT4(RED "Wheel type setting is wrong!\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      continue;
    }

    // check LiDAR ========================================
    if (op->est->lidar->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->lidar->max_n; id++) {
        if (msg.getTopic() == op->est->lidar->topic.at(id)) {
          assert(msg.instantiate<PointCloud2>() != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check GPS ========================================
    if (op->est->gps->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->gps->max_n; id++) {
        if (msg.getTopic() == op->est->gps->topic.at(id)) {
          auto ptr_fix = msg.instantiate<NavSatFix>();
          auto ptr_geo = msg.instantiate<PoseStamped>();
          assert(ptr_fix != nullptr || ptr_geo != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check VICON ========================================
    if (op->est->vicon->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->vicon->max_n; id++) {
        if (msg.getTopic() == op->est->vicon->topic.at(id)) {
          assert(msg.instantiate<PoseStamped>() != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }
  }
  PRINT2("\n");

  // bag run times
#if ROS_AVAILABLE == 2
  time_init = bag_begin + rclcpp::Duration::from_seconds(op->sys->bag_start);
  op->sys->bag_durr = bag_end.seconds() < op->sys->bag_durr ? bag_end.seconds() : op->sys->bag_durr;
  time_finish = (op->sys->bag_durr < 0) ? bag_end : time_init + rclcpp::Duration::from_seconds(op->sys->bag_durr);
#elif ROS_AVAILABLE == 1
  time_init = view.getBeginTime() + ros::Duration(op->sys->bag_start);
  op->sys->bag_durr = view.getEndTime().toSec() < op->sys->bag_durr ? view.getEndTime().toSec() : op->sys->bag_durr;
  time_finish = (op->sys->bag_durr < 0) ? view.getEndTime() : time_init + ros::Duration(op->sys->bag_durr);
#endif

  // Create state log files
  save = make_shared<State_Logger>(op);
}
