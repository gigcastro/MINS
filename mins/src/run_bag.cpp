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
#include <deque>
#include <memory>
#include <set>

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
  std::shared_ptr<const rclcpp::SerializedMessage> serialized; // shared between window copies
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
using Msg = mins::MessageInstance;
/// One rosbag2 reader per bag. 'next' holds its upcoming message; nullptr = exhausted.
struct BagSource {
  unique_ptr<rosbag2_cpp::Reader> reader;
  map<string, string> topic_types; // topic -> message type, to emulate rosbag::MessageInstance::instantiate
  rosbag2_storage::SerializedBagMessageSharedPtr next;
};
vector<BagSource> sources;
rclcpp::Time time_init, time_finish;
rclcpp::Time bag_begin, bag_end; // time span of all bags (all topics), as the metadata reports it
#elif ROS_AVAILABLE == 1
using Msg = rosbag::MessageInstance;
rosbag::View view; // lazy, merged and time-ordered over all bags
rosbag::View::iterator view_it;
vector<rosbag::Bag> bags;
ros::Time time_init, time_finish;
#endif

// The bags are streamed instead of loaded: the sources above are merged by time on the
// fly and only a small sliding lookahead window is buffered, so bags that do not fit in
// memory can be processed. The window exists solely because a stereo image's pair can
// arrive slightly later in the stream; it must stay larger than the 10 ms pairing
// tolerance of find_stereo_pair (each source must yield messages in time order, which
// indexed rosbag2 storage and rosbag::View guarantee).
const double WINDOW_SEC = 1.0;
deque<Msg> window;                     // buffered lookahead, front is next to process
uint64_t seq_front = 0;                // global sequence number of window.front()
uint64_t seq_next = 0;                 // sequence number of the next buffered message
set<uint64_t> used_seq;                // messages already consumed as stereo pairs
vector<map<double, uint64_t>> cam_map; // {cam[i], {img_time, global seq}}

// read parameters, init system, open the bags and prime the merged stream
void system_setup(int argc, char **argv);
// buffer the next message of the merged stream into the window (false when exhausted)
bool buffer_next_message();
// seconds of data currently buffered in the window
double window_span();
// find the closest stereo pair (global seq) of an image time, based on cam_map
bool find_stereo_pair(double meas_t, int cam_id, uint64_t &pair_seq);
// feed the camera measurement to the system. automatically finds the stereo pair if any
bool feed_camera(int cam_id, const Msg &msg);

// Main function
int main(int argc, char **argv) {

  // Load parameters
  system_setup(argc, argv);

  // process the stream: keep WINDOW_SEC of lookahead buffered, consume from the front
  bool stream_done = false;
#if ROS_AVAILABLE == 2
  while (rclcpp::ok()) {
#elif ROS_AVAILABLE == 1
  while (ros::ok()) {
#endif
    while (!stream_done && (window.empty() || window_span() < WINDOW_SEC))
      stream_done = !buffer_next_message();
    if (window.empty())
      break;

    Msg msg = window.front(); // cheap copy, the payload is shared
    window.pop_front();
    uint64_t seq = seq_front++;

    if (msg.getTime() > time_finish)
      break;

    // skip until start time
    if (msg.getTime() < time_init)
      continue;

    // skip used measurements. Usually stereo img
    if (used_seq.erase(seq) > 0)
      continue;

    // ===================== IMU =====================
    if (msg.getTopic() == op->est->imu->topic) {
      auto imu_msg = msg.instantiate<Imu>();
      if (imu_msg == nullptr) {
        PRINT4(RED "IMU topic has unmatched message type!. Exiting.\n" RESET);
        exit(EXIT_FAILURE);
      }
      ov_core::ImuData imu = ROSHelper::Imu2Data(imu_msg);
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
      bool is_cam = false;
      for (int cam_id = 0; cam_id < op->est->cam->max_n && !is_cam; cam_id++) {
        if (msg.getTopic() == op->est->cam->topic.at(cam_id)) {
          is_cam = true;
          if (feed_camera(cam_id, msg))
            pub->publish_cam_images({cam_id});
        }
      }
      if (is_cam)
        continue;
    }

    // ===================== WHEEL =====================
    if (op->est->wheel->enabled && msg.getTopic() == op->est->wheel->topic) {
      auto wheel_j = msg.instantiate<JointState>();
      auto wheel_o = msg.instantiate<Odometry>();
      if ((wheel_j == nullptr && wheel_o == nullptr) || (wheel_j != nullptr && op->est->wheel->sub_topics.at(0) != wheel_j->name.at(0))) {
        PRINT4(RED "Wheel type setting is wrong!\n" RESET);
        exit(EXIT_FAILURE);
      }

      WheelData data = wheel_j != nullptr ? ROSHelper::JointState2Data(wheel_j) : ROSHelper::Odometry2Data(wheel_o);
      PRINT1(MAGENTA "[BAG] WHL measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
      sys->feed_measurement_wheel(data);
      continue;
    }

    // ===================== GPS =====================
    if (op->est->gps->enabled) {
      bool is_gps = false;
      for (int gps_id = 0; gps_id < op->est->gps->max_n && !is_gps; gps_id++) {
        if (msg.getTopic() == op->est->gps->topic.at(gps_id)) {
          is_gps = true;
          auto ptr_fix = msg.instantiate<NavSatFix>();
          auto ptr_geo = msg.instantiate<PoseStamped>();
          if (ptr_fix == nullptr && ptr_geo == nullptr) {
            PRINT4(RED "GPS topic has unmatched message type!. Exiting.\n" RESET);
            exit(EXIT_FAILURE);
          }
          GPSData data = (ptr_fix != nullptr ? ROSHelper::NavSatFix2Data(ptr_fix, gps_id) : ROSHelper::PoseStamped2Data(ptr_geo, gps_id, op->est->gps->noise));
          // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
          data.noise(0) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(0) = op->est->gps->noise : double();
          data.noise(1) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(1) = op->est->gps->noise : double();
          data.noise(2) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(2) = op->est->gps->noise * 2 : double();
          PRINT1(CYAN "[BAG] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
          PRINT1(CYAN "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
          sys->feed_measurement_gps(data, ptr_fix != nullptr);
          pub->publish_gps(data, ptr_fix != nullptr);
        }
      }
      if (is_gps)
        continue;
    }

    // ==================== LiDAR ====================
    if (op->est->lidar->enabled) {
      bool is_lidar = false;
      for (int lidar_id = 0; lidar_id < op->est->lidar->max_n && !is_lidar; lidar_id++) {
        if (msg.getTopic() == op->est->lidar->topic.at(lidar_id)) {
          is_lidar = true;
          auto pc2 = msg.instantiate<PointCloud2>();
          if (pc2 == nullptr) {
            PRINT4(RED "LiDAR topic has unmatched message type!. Exiting.\n" RESET);
            exit(EXIT_FAILURE);
          }
          std::shared_ptr<mins::PointCloud<mins::PointXYZ>> data = ROSHelper::rosPC2pclPC(pc2, lidar_id);
          PRINT1("[BAG] LDR measurement: %.3f|%d|%d\n", (double)data->header.stamp / 1000, lidar_id, data->points.size());
          sys->feed_measurement_lidar(data);
          pub->publish_lidar_cloud(data);
        }
      }
      if (is_lidar)
        continue;
    }

    // ==================== VICON ====================
    if (op->est->vicon->enabled) {
      bool is_vicon = false;
      for (int vicon_id = 0; vicon_id < op->est->vicon->max_n && !is_vicon; vicon_id++) {
        if (msg.getTopic() == op->est->vicon->topic.at(vicon_id)) {
          is_vicon = true;
          auto pose = msg.instantiate<PoseStamped>();
          if (pose == nullptr) {
            PRINT4(RED "VICON topic has unmatched message type!. Exiting.\n" RESET);
            exit(EXIT_FAILURE);
          }
          ViconData data = ROSHelper::PoseStamped2Data(pose, vicon_id);
          PRINT1("[BAG] VCN measurement: %.3f|%d|", data.time, data.id);
          PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", data.pose(0), data.pose(1), data.pose(2), data.pose(3), data.pose(4), data.pose(5));
          sys->feed_measurement_vicon(data);
          pub->publish_vicon(data);
        }
      }
      if (is_vicon)
        continue;
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

double window_span() {
  if (window.size() < 2)
    return 0;
#if ROS_AVAILABLE == 2
  return (window.back().getTime() - window.front().getTime()).seconds();
#elif ROS_AVAILABLE == 1
  return (window.back().getTime() - window.front().getTime()).toSec();
#endif
}

bool buffer_next_message() {
#if ROS_AVAILABLE == 2
  // k-way merge: take the earliest upcoming message over all bags
  int best = -1;
  for (int i = 0; i < (int)sources.size(); i++) {
    if (sources.at(i).next == nullptr)
      continue;
    if (best < 0 || bag_stamp(*sources.at(i).next, 0) < bag_stamp(*sources.at(best).next, 0))
      best = i;
  }
  if (best < 0)
    return false;
  BagSource &src = sources.at(best);
  window.emplace_back(*src.next, src.topic_types.at(src.next->topic_name));
  src.next = src.reader->has_next() ? src.reader->read_next() : nullptr;
#elif ROS_AVAILABLE == 1
  if (view_it == view.end())
    return false;
  window.push_back(*view_it);
  ++view_it;
#endif

  // register camera stamps, so stereo pairs can be found by time
  const Msg &msg = window.back();
  if (op->est->cam->enabled) {
    for (int id = 0; id < op->est->cam->max_n; id++) {
      if (msg.getTopic() != op->est->cam->topic.at(id))
        continue;
      auto img_c = msg.instantiate<CompressedImage>();
      auto img_i = msg.instantiate<Image>();
      if (img_c != nullptr)
        cam_map.at(id).insert({stamp_to_sec(img_c->header.stamp), seq_next});
      else if (img_i != nullptr)
        cam_map.at(id).insert({stamp_to_sec(img_i->header.stamp), seq_next});
      else {
        PRINT4(RED "Image topic has unmatched message types!. Exiting.\n" RESET);
        exit(EXIT_FAILURE);
      }
      break;
    }
  }
  seq_next++;
  return true;
}

bool feed_camera(int cam_id, const Msg &msg) {
  auto img_c = msg.instantiate<CompressedImage>();
  auto img_i = msg.instantiate<Image>();
  // In case the image does not have timestamp (then 0), overwrite it with message time.
  if (img_c != nullptr && stamp_is_zero(img_c->header.stamp))
    img_c->header.stamp = msg.getTime();
  if (img_i != nullptr && stamp_is_zero(img_i->header.stamp))
    img_i->header.stamp = msg.getTime();
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

  // STEREO - find stereo measurement in the lookahead window
  uint64_t pair_seq;
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);
  double meas_t = img_c != nullptr ? stamp_to_sec(img_c->header.stamp) : stamp_to_sec(img_i->header.stamp);
  if (!find_stereo_pair(meas_t, cam_id, pair_seq)) {
    PRINT4(RED "Cannot find proper stereo pair of CAM%d!\n" RESET, cam_id);
    return false;
  }

  // consume the pair message: skip it when it reaches the front of the window
  used_seq.insert(pair_seq);
  const Msg &pair = window.at(pair_seq - seq_front);

  // get stereo pair image
  if (img_c != nullptr) {
    bool success0 = ROSHelper::Image2Data(img_c, cam_id, cam, op->est->cam);
    bool success1 = ROSHelper::Image2Data(pair.instantiate<CompressedImage>(), stereo_id, cam, op->est->cam);
    if (success0 && success1) {
      PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, stamp_to_sec(img_c->header.stamp), cam_id, stereo_id);
      sys->feed_measurement_camera(cam);
      pub->publish_cam_images({cam_id, stereo_id});
    }
  } else {
    bool success0 = ROSHelper::Image2Data(img_i, cam_id, cam, op->est->cam);
    bool success1 = ROSHelper::Image2Data(pair.instantiate<Image>(), stereo_id, cam, op->est->cam);
    if (success0 && success1) {
      PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, stamp_to_sec(img_i->header.stamp), cam_id, stereo_id);
      sys->feed_measurement_camera(cam);
      pub->publish_cam_images({cam_id, stereo_id});
    }
  }
  return true;
}

bool find_stereo_pair(double meas_t, int cam_id, uint64_t &pair_seq) {
  // Get the stereo pair's cam id
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);
  auto &pair_map = cam_map.at(stereo_id);

  // drop entries that already left the window (their messages were processed)
  while (!pair_map.empty() && pair_map.begin()->second < seq_front)
    pair_map.erase(pair_map.begin());
  if (pair_map.empty())
    return false;

  // get the entry closest in time to the measurement
  auto pair_lb = pair_map.lower_bound(meas_t);
  if (pair_lb == pair_map.end())
    pair_lb--;
  else if (pair_lb != pair_map.begin()) {
    auto prev_it = prev(pair_lb);
    pair_lb = abs(meas_t - prev_it->first) < abs(meas_t - pair_lb->first) ? prev_it : pair_lb;
  }

  // the pair must still be in the window (not processed yet)
  if (pair_lb->second < seq_front)
    return false;

  // filter too large time gap
  pair_seq = pair_lb->second;
  return abs(pair_lb->first - meas_t) < 0.01;
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

  // Only the topics we use are read from the bags
  vector<string> topics;
  topics.push_back(op->est->imu->topic);
  if (op->est->cam->enabled)
    topics.insert(topics.end(), op->est->cam->topic.begin(), op->est->cam->topic.end());
  if (op->est->wheel->enabled)
    topics.push_back(op->est->wheel->topic);
  if (op->est->gps->enabled)
    topics.insert(topics.end(), op->est->gps->topic.begin(), op->est->gps->topic.end());
  if (op->est->lidar->enabled)
    topics.insert(topics.end(), op->est->lidar->topic.begin(), op->est->lidar->topic.end());
  if (op->est->vicon->enabled)
    topics.insert(topics.end(), op->est->vicon->topic.begin(), op->est->vicon->topic.end());

  // Open all bags. They are merged by time and streamed while processing, so they do not
  // need to fit in memory.
  size_t n_msgs = 0;
#if ROS_AVAILABLE == 2
  rosbag2_storage::StorageFilter filter;
  filter.topics = topics;

  int64_t begin_ns = numeric_limits<int64_t>::max(), end_ns = numeric_limits<int64_t>::min();
  sources.resize(bag_paths.size());
  for (size_t i = 0; i < bag_paths.size(); i++) {
    PRINT2("[BAG] Opening bag %d/%d: %s\n", (int)(i + 1), (int)bag_paths.size(), bag_paths[i].c_str());
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_paths[i];
    BagSource &src = sources[i];
    src.reader = make_unique<rosbag2_cpp::Reader>();
    src.reader->open(storage_options, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    // time span of the whole bag (all topics), like rosbag::View::getBeginTime()/getEndTime()
    const rosbag2_storage::BagMetadata &meta = src.reader->get_metadata();
    int64_t bag_begin_ns = chrono::duration_cast<chrono::nanoseconds>(meta.starting_time.time_since_epoch()).count();
    begin_ns = min(begin_ns, bag_begin_ns);
    end_ns = max(end_ns, bag_begin_ns + (int64_t)meta.duration.count());

    // topic -> message type, needed to emulate rosbag::MessageInstance::instantiate
    for (const rosbag2_storage::TopicMetadata &topic : src.reader->get_all_topics_and_types())
      src.topic_types[topic.name] = topic.type;
    for (const auto &info : meta.topics_with_message_count)
      if (find(topics.begin(), topics.end(), info.topic_metadata.name) != topics.end())
        n_msgs += info.message_count;

    src.reader->set_filter(filter);
  }
  // The readers hold storage plugin objects, so like 'pub' above they must be released before
  // class_loader unloads the plugins at exit. Registered after the readers opened, this runs
  // before both the unload and the 'pub' handler, on every exit path.
  std::atexit([]() { sources.clear(); });
  PRINT2("[BAG] Successfully opened %d bag file(s)\n", (int)bag_paths.size());
  bag_begin = rclcpp::Time(begin_ns);
  bag_end = rclcpp::Time(end_ns);

  // bag run times
  time_init = bag_begin + rclcpp::Duration::from_seconds(op->sys->bag_start);
  op->sys->bag_durr = bag_end.seconds() < op->sys->bag_durr ? bag_end.seconds() : op->sys->bag_durr;
  time_finish = (op->sys->bag_durr < 0) ? bag_end : time_init + rclcpp::Duration::from_seconds(op->sys->bag_durr);

  // skip straight to the start time using the bag index, then prime the merge
  for (BagSource &src : sources) {
    if (op->sys->bag_start > 0)
      src.reader->seek(time_init.nanoseconds());
    src.next = src.reader->has_next() ? src.reader->read_next() : nullptr;
  }

  bool have_msgs = any_of(sources.begin(), sources.end(), [](const BagSource &src) { return src.next != nullptr; });
#elif ROS_AVAILABLE == 1
  bags.resize(bag_paths.size());
  for (size_t i = 0; i < bag_paths.size(); i++) {
    PRINT2("[BAG] Opening bag %d/%d: %s\n", (int)(i + 1), (int)bag_paths.size(), bag_paths[i].c_str());
    bags[i].open(bag_paths[i], rosbag::bagmode::Read);
    view.addQuery(bags[i], rosbag::TopicQuery(topics));
  }
  PRINT2("[BAG] Successfully opened %d bag file(s)\n", (int)bags.size());
  n_msgs = view.size();

  // bag run times
  time_init = view.getBeginTime() + ros::Duration(op->sys->bag_start);
  op->sys->bag_durr = view.getEndTime().toSec() < op->sys->bag_durr ? view.getEndTime().toSec() : op->sys->bag_durr;
  time_finish = (op->sys->bag_durr < 0) ? view.getEndTime() : time_init + ros::Duration(op->sys->bag_durr);

  view_it = view.begin();
  bool have_msgs = view_it != view.end();
#endif

  // Check to make sure we have data to play
  if (!have_msgs) {
    PRINT4(RED "\nNo messages to play on specified topics. Exiting.\n" RESET);
#if ROS_AVAILABLE == 2
    rclcpp::shutdown();
#elif ROS_AVAILABLE == 1
    ros::shutdown();
#endif
    exit(EXIT_FAILURE);
  }
  PRINT2("[BAG] Total messages to process: %d\n", (int)n_msgs);

  cam_map = vector<map<double, uint64_t>>(op->est->cam->max_n);

  // Create state log files
  save = make_shared<State_Logger>(op);
}
