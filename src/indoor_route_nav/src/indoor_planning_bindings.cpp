#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

namespace py = pybind11;

namespace
{

struct Point
{
  double x{};
  double y{};
  double z{};
};

using Polygon = std::vector<Point>;

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

bool pointInPolygon(double x, double y, const Polygon & polygon)
{
  bool inside = false;
  if (polygon.size() < 3) {
    return false;
  }

  std::size_t j = polygon.size() - 1;
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    if ((pi.y > y) != (pj.y > y)) {
      const double x_cross = (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x;
      if (x < x_cross) {
        inside = !inside;
      }
    }
    j = i;
  }
  return inside;
}

double distPointToSegment(double px, double py, const Point & a, const Point & b)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < 1e-12) {
    return std::hypot(px - a.x, py - a.y);
  }
  const double t = clamp(((px - a.x) * abx + (py - a.y) * aby) / ab2, 0.0, 1.0);
  const double qx = a.x + t * abx;
  const double qy = a.y + t * aby;
  return std::hypot(px - qx, py - qy);
}

double minDistToPolygonEdges(double x, double y, const Polygon & polygon)
{
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    best = std::min(best, distPointToSegment(x, y, polygon[i], polygon[(i + 1) % polygon.size()]));
  }
  return best;
}

bool pointInPolygonOrNearEdge(double x, double y, const Polygon & polygon, double tolerance)
{
  if (pointInPolygon(x, y, polygon)) {
    return true;
  }
  return minDistToPolygonEdges(x, y, polygon) <= tolerance;
}

std::string formatPoint2D(const Point & p)
{
  return "x=" + std::to_string(p.x) + ", y=" + std::to_string(p.y);
}

std::pair<double, double> interpolatedZOnSegment(double x, double y, const Point & a, const Point & b)
{
  const double abx = b.x - a.x;
  const double aby = b.y - a.y;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < 1e-12) {
    return {a.z, std::hypot(x - a.x, y - a.y)};
  }

  const double t = clamp(((x - a.x) * abx + (y - a.y) * aby) / ab2, 0.0, 1.0);
  const double qx = a.x + t * abx;
  const double qy = a.y + t * aby;
  return {a.z + (b.z - a.z) * t, std::hypot(x - qx, y - qy)};
}

double estimateSurfaceZ(double x, double y, const std::vector<Polygon> & polys, double fallback_z)
{
  bool found = false;
  double best_z = fallback_z;
  double best_dist = std::numeric_limits<double>::infinity();

  for (const auto & poly : polys) {
    if (!pointInPolygon(x, y, poly)) {
      continue;
    }
    for (std::size_t i = 0; i < poly.size(); ++i) {
      const auto [z, dist] = interpolatedZOnSegment(x, y, poly[i], poly[(i + 1) % poly.size()]);
      if (dist < best_dist) {
        found = true;
        best_z = z;
        best_dist = dist;
      }
    }
  }

  if (found) {
    return best_z;
  }

  double weighted = 0.0;
  double weight_sum = 0.0;
  for (const auto & poly : polys) {
    for (const auto & p : poly) {
      const double d = std::hypot(x - p.x, y - p.y);
      const double w = 1.0 / std::max(d, 0.05);
      weighted += p.z * w;
      weight_sum += w;
    }
  }

  if (weight_sum > 0.0) {
    return weighted / weight_sum;
  }
  return fallback_z;
}

std::vector<Polygon> parseAreas(const py::list & areas_list)
{
  std::vector<Polygon> polys;
  for (const auto & area_handle : areas_list) {
    py::dict area = area_handle.cast<py::dict>();
    if (!area.contains("points")) {
      continue;
    }
    py::list points_list = area["points"].cast<py::list>();

    Polygon poly;
    for (const auto & item_handle : points_list) {
      py::dict item = item_handle.cast<py::dict>();
      Point p;
      p.x = item["x"].cast<double>();
      p.y = item["y"].cast<double>();
      p.z = item.contains("z") ? item["z"].cast<double>() : 0.0;
      poly.push_back(p);
    }
    if (poly.size() >= 3) {
      polys.push_back(std::move(poly));
    }
  }

  return polys;
}

std::vector<Point> roundPathCorners(
  const std::vector<Point> & points,
  const std::function<bool(double, double)> & is_xy_allowed,
  const std::function<double(double, double)> & z_lookup,
  double radius,
  int samples)
{
  if (points.size() < 3) {
    return points;
  }

  radius = std::max(0.05, radius);
  samples = std::max(2, samples);
  std::vector<Point> out;
  out.reserve(points.size() * static_cast<std::size_t>(samples));
  out.push_back(points.front());

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const auto & prev = points[i - 1];
    const auto & curr = points[i];
    const auto & next = points[i + 1];

    const double v1x = prev.x - curr.x;
    const double v1y = prev.y - curr.y;
    const double v2x = next.x - curr.x;
    const double v2y = next.y - curr.y;
    const double l1 = std::hypot(v1x, v1y);
    const double l2 = std::hypot(v2x, v2y);
    if (l1 < 1e-6 || l2 < 1e-6) {
      out.push_back(curr);
      continue;
    }

    const double u1x = v1x / l1;
    const double u1y = v1y / l1;
    const double u2x = v2x / l2;
    const double u2y = v2y / l2;
    const double dot = clamp(u1x * u2x + u1y * u2y, -1.0, 1.0);
    const double angle = std::acos(dot);
    if (angle < 20.0 * M_PI / 180.0 || angle > 170.0 * M_PI / 180.0) {
      out.push_back(curr);
      continue;
    }

    const double cut = std::min({radius, l1 * 0.45, l2 * 0.45});
    const Point p_start{curr.x + u1x * cut, curr.y + u1y * cut, curr.z};
    const Point p_end{curr.x + u2x * cut, curr.y + u2y * cut, curr.z};

    std::vector<Point> arc;
    arc.reserve(static_cast<std::size_t>(samples + 1));
    bool valid = true;
    for (int s = 0; s <= samples; ++s) {
      const double t = static_cast<double>(s) / static_cast<double>(samples);
      const double omt = 1.0 - t;
      const double x = omt * omt * p_start.x + 2.0 * omt * t * curr.x + t * t * p_end.x;
      const double y = omt * omt * p_start.y + 2.0 * omt * t * curr.y + t * t * p_end.y;
      if (!is_xy_allowed(x, y)) {
        valid = false;
        break;
      }
      arc.push_back(Point{x, y, z_lookup(x, y)});
    }

    if (valid) {
      out.insert(out.end(), arc.begin(), arc.end());
    } else {
      out.push_back(curr);
    }
  }

  out.push_back(points.back());
  return out;
}

py::list planPath(
  py::dict start_dict,
  py::dict goal_dict,
  py::list areas_list,
  double resolution,
  double robot_radius,
  py::list blocked_zones = py::list())
{
  Point start{
    start_dict["x"].cast<double>(),
    start_dict["y"].cast<double>(),
    start_dict.contains("z") ? start_dict["z"].cast<double>() : 0.0
  };
  Point goal{
    goal_dict["x"].cast<double>(),
    goal_dict["y"].cast<double>(),
    goal_dict.contains("z") ? goal_dict["z"].cast<double>() : 0.0
  };

  const auto polys = parseAreas(areas_list);

  if (polys.empty()) {
    throw std::runtime_error("当前高度层没有可行驶区域");
  }

  std::vector<std::array<double, 4>> blocked;
  for (const auto & bz : blocked_zones) {
    py::dict d = bz.cast<py::dict>();
    blocked.push_back({{
      d["x_min"].cast<double>(),
      d["y_min"].cast<double>(),
      d["x_max"].cast<double>(),
      d["y_max"].cast<double>()
    }});
  }

  resolution = std::max(0.05, resolution);
  robot_radius = std::max(0.0, robot_radius);

  double min_x = std::min(start.x, goal.x);
  double max_x = std::max(start.x, goal.x);
  double min_y = std::min(start.y, goal.y);
  double max_y = std::max(start.y, goal.y);
  for (const auto & poly : polys) {
    for (const auto & p : poly) {
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
    }
  }

  const double margin = std::max(1.0, robot_radius + resolution * 4.0);
  min_x -= margin;
  max_x += margin;
  min_y -= margin;
  max_y += margin;

  const int cols = static_cast<int>(std::ceil((max_x - min_x) / resolution)) + 1;
  const int rows = static_cast<int>(std::ceil((max_y - min_y) / resolution)) + 1;
  if (cols <= 1 || rows <= 1) {
    throw std::runtime_error("规划区域范围过小");
  }
  if (static_cast<long long>(cols) * static_cast<long long>(rows) > 260000LL) {
    throw std::runtime_error(
      "规划网格过大(" + std::to_string(cols) + "x" + std::to_string(rows) +
      ")，请增大分辨率或缩小可行驶区域");
  }

  const auto toIndex = [cols](int c, int r) {return r * cols + c;};
  const auto fromIndex = [cols](int idx) {return std::pair<int, int>{idx % cols, idx / cols};};
  const auto worldToCell = [&](const Point & p) {
      int c = static_cast<int>(std::round((p.x - min_x) / resolution));
      int r = static_cast<int>(std::round((p.y - min_y) / resolution));
      c = std::max(0, std::min(cols - 1, c));
      r = std::max(0, std::min(rows - 1, r));
      return std::pair<int, int>{c, r};
    };
  const auto cellToWorld = [&](int c, int r) {
      return std::pair<double, double>{min_x + c * resolution, min_y + r * resolution};
    };

  const auto isWorldWalkable = [&](double x, double y) {
      for (const auto & bz : blocked) {
        double bx_margin = std::max(0.05, robot_radius * 0.8);
        if (x >= bz[0] - bx_margin && x <= bz[2] + bx_margin &&
            y >= bz[1] - bx_margin && y <= bz[3] + bx_margin) {
          return false;
        }
      }
      for (const auto & poly : polys) {
        if (!pointInPolygon(x, y, poly)) {
          continue;
        }
        if (robot_radius > 1e-6 && minDistToPolygonEdges(x, y, poly) < robot_radius) {
          continue;
        }
        return true;
      }
      return false;
    };

  const auto isWorldInsideAnyArea = [&](double x, double y) {
      const double tolerance = std::max(0.03, resolution * 0.5);
      for (const auto & poly : polys) {
        if (pointInPolygonOrNearEdge(x, y, poly, tolerance)) {
          return true;
        }
      }
      return false;
    };

  std::vector<int8_t> walkable_cache(static_cast<std::size_t>(cols * rows), -1);
  const auto isWalkable = [&](int c, int r) {
      if (c < 0 || c >= cols || r < 0 || r >= rows) {
        return false;
      }
      const int idx = toIndex(c, r);
      if (walkable_cache[static_cast<std::size_t>(idx)] >= 0) {
        return walkable_cache[static_cast<std::size_t>(idx)] == 1;
      }
      const auto [x, y] = cellToWorld(c, r);
      const bool ok = isWorldWalkable(x, y);
      walkable_cache[static_cast<std::size_t>(idx)] = ok ? 1 : 0;
      return ok;
    };

  if (!isWorldInsideAnyArea(start.x, start.y)) {
    throw std::runtime_error("当前位置不在可行驶区域内(" + formatPoint2D(start) + ")");
  }
  if (!isWorldInsideAnyArea(goal.x, goal.y)) {
    throw std::runtime_error("目标点不在可行驶区域内(" + formatPoint2D(goal) + ")");
  }

  const auto findNearestWalkable = [&](std::pair<int, int> cell, int max_radius) {
      if (isWalkable(cell.first, cell.second)) {
        return toIndex(cell.first, cell.second);
      }
      for (int rad = 1; rad <= max_radius; ++rad) {
        for (int dc = -rad; dc <= rad; ++dc) {
          for (int dr : {-rad, rad}) {
            const int c = cell.first + dc;
            const int r = cell.second + dr;
            if (isWalkable(c, r)) {
              return toIndex(c, r);
            }
          }
        }
        for (int dr = -rad + 1; dr < rad; ++dr) {
          for (int dc : {-rad, rad}) {
            const int c = cell.first + dc;
            const int r = cell.second + dr;
            if (isWalkable(c, r)) {
              return toIndex(c, r);
            }
          }
        }
      }
      return -1;
    };

  const int snap_radius = std::max(4, static_cast<int>(std::ceil((robot_radius + resolution * 2.0) / resolution)));
  const int start_idx = findNearestWalkable(worldToCell(start), snap_radius);
  const int goal_idx = findNearestWalkable(worldToCell(goal), snap_radius);
  if (start_idx < 0) {
    throw std::runtime_error(
      "当前位置在区域内，但附近没有满足机器人半径的可用网格，请减小机器人半径或把区域边界画宽一些(" +
      formatPoint2D(start) + ", robot_radius=" + std::to_string(robot_radius) + ")");
  }
  if (goal_idx < 0) {
    throw std::runtime_error(
      "目标点在区域内，但附近没有满足机器人半径的可用网格，请减小机器人半径或把区域边界画宽一些(" +
      formatPoint2D(goal) + ", robot_radius=" + std::to_string(robot_radius) + ")");
  }

  const auto heuristic = [&](int a, int b) {
      const auto [ac, ar] = fromIndex(a);
      const auto [bc, br] = fromIndex(b);
      return std::hypot(static_cast<double>(ac - bc), static_cast<double>(ar - br));
    };

  struct QueueItem
  {
    double f;
    double g;
    int idx;
    bool operator>(const QueueItem & other) const {return f > other.f;}
  };

  const int total = cols * rows;
  std::vector<int> came_from(static_cast<std::size_t>(total), -1);
  std::vector<double> g_score(static_cast<std::size_t>(total), std::numeric_limits<double>::infinity());
  std::vector<uint8_t> visited(static_cast<std::size_t>(total), 0);
  std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;

  g_score[static_cast<std::size_t>(start_idx)] = 0.0;
  open.push(QueueItem{heuristic(start_idx, goal_idx), 0.0, start_idx});

  const std::vector<std::tuple<int, int, double>> neighbors = {
    {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
    {-1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)},
    {1, -1, std::sqrt(2.0)}, {1, 1, std::sqrt(2.0)}
  };

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (visited[static_cast<std::size_t>(current.idx)]) {
      continue;
    }
    visited[static_cast<std::size_t>(current.idx)] = 1;
    if (current.idx == goal_idx) {
      break;
    }

    const auto [cc, cr] = fromIndex(current.idx);
    for (const auto & [dc, dr, cost] : neighbors) {
      const int nc = cc + dc;
      const int nr = cr + dr;
      if (!isWalkable(nc, nr)) {
        continue;
      }
      const int next_idx = toIndex(nc, nr);
      const double ng = current.g + cost;
      if (ng < g_score[static_cast<std::size_t>(next_idx)]) {
        g_score[static_cast<std::size_t>(next_idx)] = ng;
        came_from[static_cast<std::size_t>(next_idx)] = current.idx;
        open.push(QueueItem{ng + heuristic(next_idx, goal_idx), ng, next_idx});
      }
    }
  }

  if (goal_idx != start_idx && came_from[static_cast<std::size_t>(goal_idx)] < 0) {
    throw std::runtime_error("可行驶区域内未找到可达路径");
  }

  std::vector<int> cells;
  cells.push_back(goal_idx);
  while (cells.back() != start_idx) {
    cells.push_back(came_from[static_cast<std::size_t>(cells.back())]);
  }
  std::reverse(cells.begin(), cells.end());

  const auto lineIsWalkable = [&](int a, int b) {
      const auto [ac, ar] = fromIndex(a);
      const auto [bc, br] = fromIndex(b);
      const int steps = static_cast<int>(std::max(std::abs(bc - ac), std::abs(br - ar)));
      if (steps <= 1) {
        return true;
      }
      for (int i = 1; i < steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const int c = static_cast<int>(std::round(ac + (bc - ac) * t));
        const int r = static_cast<int>(std::round(ar + (br - ar) * t));
        if (!isWalkable(c, r)) {
          return false;
        }
      }
      return true;
    };

  std::vector<int> smoothed;
  smoothed.push_back(cells.front());
  std::size_t i = 0;
  while (i + 1 < cells.size()) {
    std::size_t j = cells.size() - 1;
    while (j > i + 1 && !lineIsWalkable(cells[i], cells[j])) {
      --j;
    }
    smoothed.push_back(cells[j]);
    i = j;
  }

  std::vector<Point> points;
  points.reserve(smoothed.size());
  for (const int idx : smoothed) {
    const auto [c, r] = fromIndex(idx);
    const auto [x, y] = cellToWorld(c, r);
    points.push_back(Point{x, y, estimateSurfaceZ(x, y, polys, start.z)});
  }
  if (points.empty()) {
    throw std::runtime_error("规划结果为空");
  }

  points.front() = start;
  points.back() = goal;
  points = roundPathCorners(
    points,
    isWorldWalkable,
    [&](double x, double y) {return estimateSurfaceZ(x, y, polys, start.z);},
    std::max(0.25, resolution * 3.0),
    6);
  points.front() = start;
  points.back() = goal;

  py::list result;
  for (const auto & p : points) {
    py::dict point;
    point["x"] = p.x;
    point["y"] = p.y;
    point["z"] = p.z;
    result.append(point);
  }
  return result;
}

}  // namespace

PYBIND11_MODULE(_planning_core, m)
{
  using namespace pybind11::literals;
  m.doc() = "C++ indoor path planning module (A* + smoothing + corner rounding)";
  m.def("plan_path", &planPath,
    py::arg("start"),
    py::arg("goal"),
    py::arg("areas"),
    py::arg("resolution") = 0.20,
    py::arg("robot_radius") = 0.15,
    py::arg("blocked_zones") = py::list(),
    R"(Run A* path planning in given drivable areas.

Args:
    start: {"x": float, "y": float, "z": float}
    goal: {"x": float, "y": float, "z": float}
    areas: list of {"points": [{"x":float, "y":float, "z":float}, ...], ...}
    resolution: grid resolution in meters
    robot_radius: robot radius for collision margin
    blocked_zones: list of {"x_min": float, "y_min": float, "x_max": float, "y_max": float} bounding boxes

Returns:
    list of {"x": float, "y": float, "z": float} waypoints
)");
}
