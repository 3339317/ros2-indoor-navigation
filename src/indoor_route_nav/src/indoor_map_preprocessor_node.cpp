#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

namespace indoor_route_nav
{

namespace
{

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

struct FloorRange
{
  std::string id;
  double z_min{};
  double z_max{};
  double z_center{};
  std::size_t point_count{};
};

struct Config
{
  std::string input_path;
  std::string output_path{"/tmp/indoor_nav_preprocessed_map.json"};
  std::string map_name{"indoor_map"};
  std::string frame_id{"map"};
  int sample_step{3};
  double voxel_leaf_size{0.03};
  std::string floor_mode{"auto"};
  double histogram_bin_size{0.10};
  int min_peak_points{800};
  double min_floor_gap{1.20};
  double floor_thickness{0.45};
  bool generate_drivable_area{true};
  double boundary_margin{0.20};
  double percentile_low{2.0};
  double percentile_high{98.0};
  int min_points_per_area{1000};
  double robot_radius{0.25};
  double grid_resolution{0.05};
  bool stair_generate{true};
  double stair_grid_resolution{0.20};
  int stair_min_cell_points{1};
  int stair_min_component_cells{8};
  double stair_floor_exclusion{0.25};
  double stair_min_vertical_span{0.60};
  double stair_min_floor_coverage{0.40};
  double stair_min_length{0.80};
  double stair_max_width{4.00};
  int stair_polyline_points{8};
  std::vector<FloorRange> manual_ranges;
};

template<typename T>
T yamlValue(const YAML::Node & node, const std::string & key, const T & fallback)
{
  if (!node || !node[key]) {
    return fallback;
  }
  return node[key].as<T>();
}

Config loadConfig(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  Config cfg;

  cfg.input_path = yamlValue<std::string>(root, "input_path", cfg.input_path);
  cfg.output_path = yamlValue<std::string>(root, "output_path", cfg.output_path);

  const auto map = root["map"];
  cfg.map_name = yamlValue<std::string>(map, "name", cfg.map_name);
  cfg.frame_id = yamlValue<std::string>(map, "frame_id", cfg.frame_id);

  const auto pc = root["point_cloud"];
  cfg.sample_step = std::max(1, yamlValue<int>(pc, "sample_step", cfg.sample_step));
  cfg.voxel_leaf_size = std::max(0.0, yamlValue<double>(pc, "voxel_leaf_size", cfg.voxel_leaf_size));

  const auto floors = root["floors"];
  cfg.floor_mode = yamlValue<std::string>(floors, "mode", cfg.floor_mode);
  cfg.histogram_bin_size = std::max(0.01, yamlValue<double>(floors, "histogram_bin_size", cfg.histogram_bin_size));
  cfg.min_peak_points = std::max(1, yamlValue<int>(floors, "min_peak_points", cfg.min_peak_points));
  cfg.min_floor_gap = std::max(0.05, yamlValue<double>(floors, "min_floor_gap", cfg.min_floor_gap));
  cfg.floor_thickness = std::max(0.05, yamlValue<double>(floors, "floor_thickness", cfg.floor_thickness));

  if (floors && floors["manual_ranges"] && floors["manual_ranges"].IsSequence()) {
    int i = 0;
    for (const auto & item : floors["manual_ranges"]) {
      FloorRange f;
      f.id = yamlValue<std::string>(item, "id", "floor_" + std::to_string(++i));
      f.z_min = yamlValue<double>(item, "z_min", 0.0);
      f.z_max = yamlValue<double>(item, "z_max", 0.0);
      if (f.z_min > f.z_max) {
        std::swap(f.z_min, f.z_max);
      }
      f.z_center = (f.z_min + f.z_max) * 0.5;
      cfg.manual_ranges.push_back(f);
    }
  }

  const auto area = root["drivable_area"];
  cfg.generate_drivable_area = yamlValue<bool>(area, "generate", cfg.generate_drivable_area);
  cfg.boundary_margin = std::max(0.0, yamlValue<double>(area, "boundary_margin", cfg.boundary_margin));
  cfg.percentile_low = yamlValue<double>(area, "percentile_low", cfg.percentile_low);
  cfg.percentile_high = yamlValue<double>(area, "percentile_high", cfg.percentile_high);
  cfg.min_points_per_area = std::max(3, yamlValue<int>(area, "min_points_per_area", cfg.min_points_per_area));

  const auto meta = root["metadata"];
  cfg.robot_radius = std::max(0.0, yamlValue<double>(meta, "robot_radius", cfg.robot_radius));
  cfg.grid_resolution = std::max(0.01, yamlValue<double>(meta, "grid_resolution", cfg.grid_resolution));

  const auto stairs = root["stairs"];
  cfg.stair_generate = yamlValue<bool>(stairs, "generate", cfg.stair_generate);
  cfg.stair_grid_resolution = std::max(0.05, yamlValue<double>(stairs, "grid_resolution", cfg.stair_grid_resolution));
  cfg.stair_min_cell_points = std::max(1, yamlValue<int>(stairs, "min_cell_points", cfg.stair_min_cell_points));
  cfg.stair_min_component_cells = std::max(2, yamlValue<int>(stairs, "min_component_cells", cfg.stair_min_component_cells));
  cfg.stair_floor_exclusion = std::max(0.0, yamlValue<double>(stairs, "floor_exclusion", cfg.stair_floor_exclusion));
  cfg.stair_min_vertical_span = std::max(0.05, yamlValue<double>(stairs, "min_vertical_span", cfg.stair_min_vertical_span));
  cfg.stair_min_floor_coverage = std::max(0.05, yamlValue<double>(stairs, "min_floor_coverage", cfg.stair_min_floor_coverage));
  cfg.stair_min_length = std::max(0.05, yamlValue<double>(stairs, "min_length", cfg.stair_min_length));
  cfg.stair_max_width = std::max(0.10, yamlValue<double>(stairs, "max_width", cfg.stair_max_width));
  cfg.stair_polyline_points = std::max(2, yamlValue<int>(stairs, "polyline_points", cfg.stair_polyline_points));

  if (cfg.percentile_low < 0.0) {
    cfg.percentile_low = 0.0;
  }
  if (cfg.percentile_high > 100.0) {
    cfg.percentile_high = 100.0;
  }
  if (cfg.percentile_low >= cfg.percentile_high) {
    cfg.percentile_low = 2.0;
    cfg.percentile_high = 98.0;
  }

  return cfg;
}

std::string lowerExt(const std::string & path)
{
  const auto pos = path.find_last_of('.');
  if (pos == std::string::npos) {
    return "";
  }
  std::string ext = path.substr(pos);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return ext;
}

CloudT::Ptr loadXyzLike(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("无法打开点云文件: " + path);
  }

  auto cloud = std::make_shared<CloudT>();
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  while (in >> x >> y >> z) {
    PointT p;
    p.x = static_cast<float>(x);
    p.y = static_cast<float>(y);
    p.z = static_cast<float>(z);
    p.r = 255;
    p.g = 255;
    p.b = 255;
    cloud->push_back(p);
    std::string rest;
    std::getline(in, rest);
  }
  return cloud;
}

CloudT::Ptr loadCloud(const std::string & path)
{
  auto cloud = std::make_shared<CloudT>();
  const auto ext = lowerExt(path);
  int rc = -1;
  if (ext == ".pcd") {
    rc = pcl::io::loadPCDFile<PointT>(path, *cloud);
  } else if (ext == ".ply") {
    rc = pcl::io::loadPLYFile<PointT>(path, *cloud);
  } else if (ext == ".xyz" || ext == ".xyzn" || ext == ".xyzrgb") {
    return loadXyzLike(path);
  } else {
    throw std::runtime_error("不支持的点云格式: " + ext);
  }

  if (rc != 0 || cloud->empty()) {
    throw std::runtime_error("点云读取失败或为空: " + path);
  }
  return cloud;
}

CloudT::Ptr filterFinite(const CloudT::Ptr & input)
{
  auto out = std::make_shared<CloudT>();
  out->reserve(input->size());
  for (const auto & p : input->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      out->push_back(p);
    }
  }
  return out;
}

CloudT::Ptr voxelDownsample(const CloudT::Ptr & input, double leaf)
{
  if (leaf <= 1e-9) {
    return input;
  }
  auto out = std::make_shared<CloudT>();
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud(input);
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  voxel.filter(*out);
  return out;
}

std::pair<double, double> zBounds(const CloudT::Ptr & cloud)
{
  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  for (const auto & p : cloud->points) {
    lo = std::min(lo, static_cast<double>(p.z));
    hi = std::max(hi, static_cast<double>(p.z));
  }
  return {lo, hi};
}

std::vector<FloorRange> detectFloorsAuto(const CloudT::Ptr & cloud, const Config & cfg)
{
  const auto [z_min, z_max] = zBounds(cloud);
  const int bins = std::max(1, static_cast<int>(std::ceil((z_max - z_min) / cfg.histogram_bin_size)) + 1);
  std::vector<int> hist(static_cast<std::size_t>(bins), 0);
  for (const auto & p : cloud->points) {
    const int idx = std::clamp(
      static_cast<int>(std::floor((p.z - z_min) / cfg.histogram_bin_size)),
      0,
      bins - 1);
    hist[static_cast<std::size_t>(idx)]++;
  }

  std::vector<double> centers;
  for (int i = 0; i < bins; ++i) {
    const int prev = i > 0 ? hist[static_cast<std::size_t>(i - 1)] : 0;
    const int next = i + 1 < bins ? hist[static_cast<std::size_t>(i + 1)] : 0;
    const int current = hist[static_cast<std::size_t>(i)];
    if (current >= cfg.min_peak_points && current >= prev && current >= next) {
      const double z = z_min + (static_cast<double>(i) + 0.5) * cfg.histogram_bin_size;
      if (centers.empty() || std::abs(z - centers.back()) >= cfg.min_floor_gap) {
        centers.push_back(z);
      } else {
        const double prev_z = centers.back();
        const int prev_idx = static_cast<int>(std::floor((prev_z - z_min) / cfg.histogram_bin_size));
        if (prev_idx >= 0 && prev_idx < bins && current > hist[static_cast<std::size_t>(prev_idx)]) {
          centers.back() = z;
        }
      }
    }
  }

  if (centers.size() < 2) {
    std::vector<int> order(bins);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return hist[static_cast<std::size_t>(a)] > hist[static_cast<std::size_t>(b)];
      });
    for (const int idx : order) {
      if (hist[static_cast<std::size_t>(idx)] <= 0) {
        continue;
      }
      const double z = z_min + (static_cast<double>(idx) + 0.5) * cfg.histogram_bin_size;
      bool separated = true;
      for (const double existing : centers) {
        if (std::abs(existing - z) < cfg.min_floor_gap) {
          separated = false;
          break;
        }
      }
      if (separated) {
        centers.push_back(z);
      }
      if (centers.size() >= 4) {
        break;
      }
    }
    std::sort(centers.begin(), centers.end());
  }

  if (centers.empty()) {
    std::vector<double> zs;
    zs.reserve(cloud->size());
    for (const auto & p : cloud->points) {
      zs.push_back(p.z);
    }
    std::sort(zs.begin(), zs.end());

    std::vector<std::pair<double, double>> clusters;
    double cluster_min = zs.front();
    double cluster_max = zs.front();
    for (std::size_t i = 1; i < zs.size(); ++i) {
      if (zs[i] - cluster_max >= cfg.min_floor_gap) {
        clusters.push_back({cluster_min, cluster_max});
        cluster_min = zs[i];
      }
      cluster_max = zs[i];
    }
    clusters.push_back({cluster_min, cluster_max});

    for (const auto & c : clusters) {
      centers.push_back((c.first + c.second) * 0.5);
    }
  }

  std::vector<FloorRange> floors;
  floors.reserve(centers.size());
  for (std::size_t i = 0; i < centers.size(); ++i) {
    FloorRange f;
    f.id = "floor_" + std::to_string(i + 1);
    f.z_center = centers[i];
    f.z_min = centers[i] - cfg.floor_thickness * 0.5;
    f.z_max = centers[i] + cfg.floor_thickness * 0.5;
    floors.push_back(f);
  }
  return floors;
}

std::vector<FloorRange> buildFloors(const CloudT::Ptr & cloud, const Config & cfg)
{
  std::vector<FloorRange> floors =
    cfg.floor_mode == "manual" && !cfg.manual_ranges.empty() ?
    cfg.manual_ranges :
    detectFloorsAuto(cloud, cfg);

  for (auto & floor : floors) {
    floor.point_count = 0;
    for (const auto & p : cloud->points) {
      if (p.z >= floor.z_min && p.z <= floor.z_max) {
        floor.point_count++;
      }
    }
  }

  floors.erase(
    std::remove_if(
      floors.begin(), floors.end(),
      [](const FloorRange & f) {return f.point_count == 0;}),
    floors.end());
  return floors;
}

double percentile(std::vector<double> values, double pct)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double pos = (pct / 100.0) * static_cast<double>(values.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(pos));
  const auto hi = static_cast<std::size_t>(std::ceil(pos));
  if (lo == hi) {
    return values[lo];
  }
  const double t = pos - static_cast<double>(lo);
  return values[lo] * (1.0 - t) + values[hi] * t;
}

nlohmann::json makeDrivableArea(const CloudT::Ptr & cloud, const FloorRange & floor, const Config & cfg)
{
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(floor.point_count);
  ys.reserve(floor.point_count);
  for (const auto & p : cloud->points) {
    if (p.z >= floor.z_min && p.z <= floor.z_max) {
      xs.push_back(p.x);
      ys.push_back(p.y);
    }
  }
  if (xs.size() < static_cast<std::size_t>(cfg.min_points_per_area)) {
    return nullptr;
  }

  const double min_x = percentile(xs, cfg.percentile_low) - cfg.boundary_margin;
  const double max_x = percentile(xs, cfg.percentile_high) + cfg.boundary_margin;
  const double min_y = percentile(ys, cfg.percentile_low) - cfg.boundary_margin;
  const double max_y = percentile(ys, cfg.percentile_high) + cfg.boundary_margin;
  const double z = floor.z_center;

  nlohmann::json area;
  area["name"] = "auto_" + floor.id;
  area["z_min"] = floor.z_min;
  area["z_max"] = floor.z_max;
  area["points"] = nlohmann::json::array({
    {{"x", min_x}, {"y", min_y}, {"z", z}},
    {{"x", max_x}, {"y", min_y}, {"z", z}},
    {{"x", max_x}, {"y", max_y}, {"z", z}},
    {{"x", min_x}, {"y", max_y}, {"z", z}}
  });
  return area;
}

struct StairCell
{
  int ix{};
  int iy{};
  int count{};
  double sx{};
  double sy{};
  double sz{};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};

  double x() const {return sx / std::max(1, count);}
  double y() const {return sy / std::max(1, count);}
  double z() const {return sz / std::max(1, count);}
};

long long cellKey(int ix, int iy)
{
  return (static_cast<long long>(ix) << 32) ^ static_cast<unsigned int>(iy);
}

std::pair<int, int> splitCellKey(long long key)
{
  const int ix = static_cast<int>(key >> 32);
  const int iy = static_cast<int>(key & 0xffffffff);
  return {ix, iy};
}

std::vector<nlohmann::json> detectStairConnectors(
  const CloudT::Ptr & cloud,
  std::vector<FloorRange> floors,
  const Config & cfg)
{
  std::vector<nlohmann::json> connectors;
  if (!cfg.stair_generate || floors.size() < 2) {
    return connectors;
  }

  std::sort(floors.begin(), floors.end(), [](const FloorRange & a, const FloorRange & b) {
      return a.z_center < b.z_center;
    });

  for (std::size_t fi = 0; fi + 1 < floors.size(); ++fi) {
    for (std::size_t fj = fi + 1; fj < floors.size(); ++fj) {
    const auto & low = floors[fi];
    const auto & high = floors[fj];
    const double floor_delta = high.z_center - low.z_center;
    if (floor_delta < cfg.min_floor_gap * 0.5) {
      continue;
    }

    const double z_low = low.z_center + cfg.stair_floor_exclusion;
    const double z_high = high.z_center - cfg.stair_floor_exclusion;
    if (z_high <= z_low) {
      continue;
    }

    std::unordered_map<long long, StairCell> cells;
    for (const auto & p : cloud->points) {
      if (p.z < z_low || p.z > z_high) {
        continue;
      }
      const int ix = static_cast<int>(std::floor(p.x / cfg.stair_grid_resolution));
      const int iy = static_cast<int>(std::floor(p.y / cfg.stair_grid_resolution));
      auto & c = cells[cellKey(ix, iy)];
      c.ix = ix;
      c.iy = iy;
      c.count++;
      c.sx += p.x;
      c.sy += p.y;
      c.sz += p.z;
      c.min_z = std::min(c.min_z, static_cast<double>(p.z));
      c.max_z = std::max(c.max_z, static_cast<double>(p.z));
    }

    std::unordered_set<long long> valid;
    valid.reserve(cells.size());
    for (const auto & kv : cells) {
      if (kv.second.count >= cfg.stair_min_cell_points) {
        valid.insert(kv.first);
      }
    }

    std::unordered_set<long long> visited;
    int component_idx = 0;
    for (const auto & start_key : valid) {
      if (visited.count(start_key) != 0) {
        continue;
      }

      std::vector<StairCell> comp;
      std::queue<long long> q;
      q.push(start_key);
      visited.insert(start_key);

      while (!q.empty()) {
        const auto key = q.front();
        q.pop();
        comp.push_back(cells.at(key));
        const auto [ix, iy] = splitCellKey(key);
        for (int dx = -1; dx <= 1; ++dx) {
          for (int dy = -1; dy <= 1; ++dy) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const auto nk = cellKey(ix + dx, iy + dy);
            if (valid.count(nk) == 0 || visited.count(nk) != 0) {
              continue;
            }
            visited.insert(nk);
            q.push(nk);
          }
        }
      }

      if (comp.size() < static_cast<std::size_t>(cfg.stair_min_component_cells)) {
        continue;
      }

      double total_weight = 0.0;
      double mx = 0.0;
      double my = 0.0;
      double min_z = std::numeric_limits<double>::infinity();
      double max_z = -std::numeric_limits<double>::infinity();
      for (const auto & c : comp) {
        const double w = static_cast<double>(c.count);
        total_weight += w;
        mx += c.x() * w;
        my += c.y() * w;
        min_z = std::min(min_z, c.z());
        max_z = std::max(max_z, c.z());
      }
      mx /= std::max(1.0, total_weight);
      my /= std::max(1.0, total_weight);

      const double vertical_span = max_z - min_z;
      if (
        vertical_span < cfg.stair_min_vertical_span ||
        vertical_span < floor_delta * cfg.stair_min_floor_coverage)
      {
        continue;
      }

      double cxx = 0.0;
      double cxy = 0.0;
      double cyy = 0.0;
      for (const auto & c : comp) {
        const double w = static_cast<double>(c.count);
        const double dx = c.x() - mx;
        const double dy = c.y() - my;
        cxx += dx * dx * w;
        cxy += dx * dy * w;
        cyy += dy * dy * w;
      }

      const double theta = 0.5 * std::atan2(2.0 * cxy, cxx - cyy);
      double ax = std::cos(theta);
      double ay = std::sin(theta);

      std::vector<double> proj;
      std::vector<double> perp;
      proj.reserve(comp.size());
      perp.reserve(comp.size());
      for (const auto & c : comp) {
        const double dx = c.x() - mx;
        const double dy = c.y() - my;
        proj.push_back(dx * ax + dy * ay);
        perp.push_back(-dx * ay + dy * ax);
      }

      const auto [proj_min_it, proj_max_it] = std::minmax_element(proj.begin(), proj.end());
      const auto [perp_min_it, perp_max_it] = std::minmax_element(perp.begin(), perp.end());
      const double length = *proj_max_it - *proj_min_it;
      const double width = *perp_max_it - *perp_min_it;
      if (length < cfg.stair_min_length || width > cfg.stair_max_width) {
        continue;
      }

      const int samples = cfg.stair_polyline_points;
      std::vector<nlohmann::json> points;
      points.reserve(static_cast<std::size_t>(samples));
      for (int si = 0; si < samples; ++si) {
        const double t0 = *proj_min_it + length * static_cast<double>(si) / static_cast<double>(samples);
        const double t1 = *proj_min_it + length * static_cast<double>(si + 1) / static_cast<double>(samples);
        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        double sw = 0.0;
        for (std::size_t ci = 0; ci < comp.size(); ++ci) {
          const bool in_bin =
            (si == samples - 1) ? (proj[ci] >= t0 && proj[ci] <= t1) : (proj[ci] >= t0 && proj[ci] < t1);
          if (!in_bin) {
            continue;
          }
          const double w = static_cast<double>(comp[ci].count);
          sx += comp[ci].x() * w;
          sy += comp[ci].y() * w;
          sz += comp[ci].z() * w;
          sw += w;
        }
        if (sw > 0.0) {
          points.push_back({{"x", sx / sw}, {"y", sy / sw}, {"z", sz / sw}});
        }
      }

      if (points.size() < 2) {
        continue;
      }

      if (points.front()["z"].get<double>() > points.back()["z"].get<double>()) {
        std::reverse(points.begin(), points.end());
        ax = -ax;
        ay = -ay;
      }

      auto low_end = points.front();
      auto high_end = points.back();
      low_end["z"] = low.z_center;
      high_end["z"] = high.z_center;
      points.insert(points.begin(), low_end);
      points.push_back(high_end);

      nlohmann::json conn;
      conn["type"] = "stair";
      conn["name"] = "auto_stair_" + low.id + "_" + high.id + "_" + std::to_string(++component_idx);
      conn["from_floor"] = low.id;
      conn["to_floor"] = high.id;
      conn["z_min"] = low.z_min;
      conn["z_max"] = high.z_max;
      conn["points"] = points;
      conn["debug"] = {
        {"component_cells", comp.size()},
        {"vertical_span", vertical_span},
        {"length", length},
        {"width", width}
      };
      connectors.push_back(conn);
    }
    }
  }

  return connectors;
}

nlohmann::json buildWorkspaceJson(
  const CloudT::Ptr & display_cloud,
  const CloudT::Ptr & compute_cloud,
  const std::vector<FloorRange> & floors,
  const Config & cfg)
{
  nlohmann::json out;
  out["format"] = "indoor_route_nav_workspace";
  out["version"] = 2;
  out["source"] = "indoor_map_preprocessor_node";
  out["pcd_path"] = cfg.input_path;
  out["map_name"] = cfg.map_name;
  out["map_is_3d"] = true;
  out["metadata"] = {
    {"frame_id", cfg.frame_id},
    {"robot_radius", cfg.robot_radius},
    {"grid_resolution", cfg.grid_resolution},
    {"sample_step", cfg.sample_step},
    {"voxel_leaf_size", cfg.voxel_leaf_size}
  };

  out["points_xyz"] = nlohmann::json::array();
  out["points_rgb"] = nlohmann::json::array();
  for (std::size_t i = 0; i < display_cloud->size(); i += static_cast<std::size_t>(cfg.sample_step)) {
    const auto & p = display_cloud->points[i];
    out["points_xyz"].push_back({{"x", p.x}, {"y", p.y}, {"z", p.z}});
    out["points_rgb"].push_back({
      static_cast<double>(p.r) / 255.0,
      static_cast<double>(p.g) / 255.0,
      static_cast<double>(p.b) / 255.0
    });
  }

  out["floors"] = nlohmann::json::array();
  for (const auto & f : floors) {
    out["floors"].push_back({
      {"id", f.id},
      {"z_min", f.z_min},
      {"z_max", f.z_max},
      {"z_center", f.z_center},
      {"point_count", f.point_count}
    });
  }

  out["drivable_areas"] = nlohmann::json::array();
  if (cfg.generate_drivable_area) {
    for (const auto & f : floors) {
      auto area = makeDrivableArea(compute_cloud, f, cfg);
      if (!area.is_null()) {
        out["drivable_areas"].push_back(area);
      }
    }
  }

  out["connectors"] = nlohmann::json::array();
  for (const auto & connector : detectStairConnectors(compute_cloud, floors, cfg)) {
    out["connectors"].push_back(connector);
  }

  return out;
}

void writeJson(const std::string & path, const nlohmann::json & data)
{
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("无法写入输出文件: " + path);
  }
  out << data.dump(2) << "\n";
}

std::string defaultConfigPath()
{
  return ament_index_cpp::get_package_share_directory("indoor_route_nav") +
         "/config/map_preprocessor.yaml";
}

}  // namespace

class MapPreprocessorNode : public rclcpp::Node
{
public:
  explicit MapPreprocessorNode(const std::string & cli_config_path)
  : Node("indoor_map_preprocessor_node")
  {
    std::string fallback_config_path = cli_config_path;
    if (fallback_config_path.empty()) {
      try {
        fallback_config_path = defaultConfigPath();
      } catch (const std::exception &) {
        fallback_config_path = "";
      }
    }

    const auto config_path = declare_parameter<std::string>("config_path", fallback_config_path);
    if (config_path.empty()) {
      throw std::runtime_error("请通过 --config 或 ROS 参数 config_path 设置预处理配置文件");
    }
    auto cfg = loadConfig(config_path);

    const auto input_param = declare_parameter<std::string>("input_path", "");
    const auto output_param = declare_parameter<std::string>("output_path", "");
    if (!input_param.empty()) {
      cfg.input_path = input_param;
    }
    if (!output_param.empty()) {
      cfg.output_path = output_param;
    }
    if (cfg.input_path.empty()) {
      throw std::runtime_error("请在配置文件或 ROS 参数中设置 input_path");
    }

    RCLCPP_INFO(get_logger(), "map preprocess config: %s", config_path.c_str());
    RCLCPP_INFO(get_logger(), "loading point cloud: %s", cfg.input_path.c_str());

    auto raw = filterFinite(loadCloud(cfg.input_path));
    if (raw->empty()) {
      throw std::runtime_error("有效点云为空");
    }
    auto compute = voxelDownsample(raw, cfg.voxel_leaf_size);
    auto floors = buildFloors(compute, cfg);
    auto workspace = buildWorkspaceJson(raw, compute, floors, cfg);
    writeJson(cfg.output_path, workspace);

    RCLCPP_INFO(
      get_logger(),
      "preprocess done: raw=%zu compute=%zu floors=%zu areas=%zu output=%s",
      raw->size(),
      compute->size(),
      floors.size(),
      workspace["drivable_areas"].size(),
      cfg.output_path.c_str());
  }
};

}  // namespace indoor_route_nav

int main(int argc, char ** argv)
{
  std::string cli_config_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if ((arg == "--config" || arg == "-c" || arg == "--config_path") && i + 1 < argc) {
      cli_config_path = argv[++i];
      continue;
    }
    const std::string prefix = "--config=";
    if (arg.rfind(prefix, 0) == 0) {
      cli_config_path = arg.substr(prefix.size());
    }
  }

  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<indoor_route_nav::MapPreprocessorNode>(cli_config_path);
    (void)node;
  } catch (const std::exception & e) {
    std::cerr << "indoor_map_preprocessor_node failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
