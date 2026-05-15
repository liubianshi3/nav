#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

struct Options {
  std::string input_path;
  std::string output_dir;
  std::string pcd_output_path;
  double resolution = 0.05;
  double ground_threshold = 0.12;
  double robot_height = 1.2;
  int min_obstacle_points = 1;
  double border_padding = 1.0;
};

void usage(const char *argv0) {
  std::cerr
      << "Usage: " << argv0 << " <map.bt|map.ot> --output <dir> [options]\n"
      << "Options:\n"
      << "  --resolution <m>              Output grid resolution (default: 0.05)\n"
      << "  --ground-threshold <m>        Occupied voxels below this z are free ground (default: 0.12)\n"
      << "  --robot-height <m>            Occupied voxels above this z are ignored (default: 1.2)\n"
      << "  --min-obstacle-points <n>     Occupied voxel count needed per 2D cell (default: 1)\n"
      << "  --pcd-output <path>           Optional PCD export of occupied voxels\n"
      << "  --border-padding <m>          Padding around octree bounds (default: 1.0)\n";
}

double parse_double(const std::string &s, const std::string &name) {
  try {
    size_t idx = 0;
    double value = std::stod(s, &idx);
    if (idx != s.size() || !std::isfinite(value)) {
      throw std::invalid_argument("invalid");
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid " + name + ": " + s);
  }
}

int parse_int(const std::string &s, const std::string &name) {
  try {
    size_t idx = 0;
    int value = std::stoi(s, &idx);
    if (idx != s.size()) {
      throw std::invalid_argument("invalid");
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid " + name + ": " + s);
  }
}

Options parse_args(int argc, char **argv) {
  if (argc < 2) {
    usage(argv[0]);
    throw std::runtime_error("missing octomap path");
  }

  Options opts;
  opts.input_path = argv[1];

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];
    auto need_value = [&](const std::string &name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--output" || arg == "-o") {
      opts.output_dir = need_value(arg);
    } else if (arg == "--resolution") {
      opts.resolution = parse_double(need_value(arg), arg);
    } else if (arg == "--ground-threshold") {
      opts.ground_threshold = parse_double(need_value(arg), arg);
    } else if (arg == "--robot-height") {
      opts.robot_height = parse_double(need_value(arg), arg);
    } else if (arg == "--min-obstacle-points") {
      opts.min_obstacle_points = parse_int(need_value(arg), arg);
    } else if (arg == "--pcd-output") {
      opts.pcd_output_path = need_value(arg);
    } else if (arg == "--border-padding") {
      opts.border_padding = parse_double(need_value(arg), arg);
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (opts.output_dir.empty()) {
    opts.output_dir = fs::path(opts.input_path).parent_path().string();
    if (opts.output_dir.empty()) {
      opts.output_dir = ".";
    }
  }
  if (opts.resolution <= 0.0) {
    throw std::runtime_error("--resolution must be > 0");
  }
  if (opts.robot_height <= opts.ground_threshold) {
    throw std::runtime_error("--robot-height must be greater than --ground-threshold");
  }
  if (opts.min_obstacle_points < 1) {
    throw std::runtime_error("--min-obstacle-points must be >= 1");
  }
  return opts;
}

struct Grid {
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.05;
  int width = 0;
  int height = 0;
  std::vector<int> obstacle_count;
  std::vector<int> free_count;

  int index(int row, int col) const { return row * width + col; }
};

void fill_cell_range(
    Grid &grid,
    double x,
    double y,
    double size,
    bool occupied,
    bool free_ground) {
  const double half = size * 0.5;
  int min_col = static_cast<int>(std::floor((x - half - grid.origin_x) / grid.resolution));
  int max_col = static_cast<int>(std::floor((x + half - grid.origin_x) / grid.resolution));
  int min_row = static_cast<int>(std::floor((y - half - grid.origin_y) / grid.resolution));
  int max_row = static_cast<int>(std::floor((y + half - grid.origin_y) / grid.resolution));

  min_col = std::clamp(min_col, 0, grid.width - 1);
  max_col = std::clamp(max_col, 0, grid.width - 1);
  min_row = std::clamp(min_row, 0, grid.height - 1);
  max_row = std::clamp(max_row, 0, grid.height - 1);

  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      int idx = grid.index(row, col);
      if (occupied && !free_ground) {
        grid.obstacle_count[idx] += 1;
      } else {
        grid.free_count[idx] += 1;
      }
    }
  }
}

void write_outputs(const Grid &grid, const Options &opts) {
  fs::create_directories(opts.output_dir);
  fs::path pgm_path = fs::path(opts.output_dir) / "map.pgm";
  fs::path yaml_path = fs::path(opts.output_dir) / "map.yaml";

  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    throw std::runtime_error("failed to open " + pgm_path.string());
  }
  pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";

  int occ = 0;
  int free = 0;
  int unknown = 0;
  for (int row = grid.height - 1; row >= 0; --row) {
    for (int col = 0; col < grid.width; ++col) {
      int idx = grid.index(row, col);
      int pixel = 205;
      if (grid.obstacle_count[idx] >= opts.min_obstacle_points) {
        pixel = 0;
        occ += 1;
      } else if (grid.free_count[idx] > 0) {
        pixel = 254;
        free += 1;
      } else {
        unknown += 1;
      }
      auto byte = static_cast<unsigned char>(pixel);
      pgm.write(reinterpret_cast<const char *>(&byte), 1);
    }
  }

  std::ofstream yaml(yaml_path);
  if (!yaml) {
    throw std::runtime_error("failed to open " + yaml_path.string());
  }
  yaml << "image: map.pgm\n"
       << "resolution: " << grid.resolution << "\n"
       << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
       << "negate: 0\n"
       << "occupied_thresh: 0.65\n"
       << "free_thresh: 0.25\n"
       << "mode: trinary\n";

  const double total = std::max(1, grid.width * grid.height);
  std::cout << "[octomap_to_2d_grid] Wrote " << pgm_path << " (" << grid.width << "x"
            << grid.height << ", res=" << grid.resolution << "m)\n";
  std::cout << "[octomap_to_2d_grid] Wrote " << yaml_path << "\n";
  std::cout << "[octomap_to_2d_grid] Grid bounds: x=[" << grid.origin_x << ", "
            << grid.origin_x + grid.width * grid.resolution << "], y=[" << grid.origin_y
            << ", " << grid.origin_y + grid.height * grid.resolution << "]\n";
  std::cout << "[octomap_to_2d_grid] Cells: " << (100.0 * occ / total)
            << "% occupied, " << (100.0 * free / total) << "% free, "
            << (100.0 * unknown / total) << "% unknown\n";
}

void write_ascii_pcd(
    const std::vector<std::array<double, 3>> &points,
    const fs::path &pcd_path) {
  std::ofstream handle(pcd_path);
  if (!handle) {
    throw std::runtime_error("failed to open " + pcd_path.string());
  }

  handle << "# .PCD v0.7 - Point Cloud Data file format\n";
  handle << "VERSION 0.7\n";
  handle << "FIELDS x y z\n";
  handle << "SIZE 4 4 4\n";
  handle << "TYPE F F F\n";
  handle << "COUNT 1 1 1\n";
  handle << "WIDTH " << points.size() << "\n";
  handle << "HEIGHT 1\n";
  handle << "VIEWPOINT 0 0 0 1 0 0 0\n";
  handle << "POINTS " << points.size() << "\n";
  handle << "DATA ascii\n";
  handle.setf(std::ios::fixed);
  handle.precision(6);
  for (const auto &point : points) {
    handle << point[0] << " " << point[1] << " " << point[2] << "\n";
  }
}

int main(int argc, char **argv) {
  try {
    const Options opts = parse_args(argc, argv);
    std::unique_ptr<octomap::OcTree> binary_tree;
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree;
    octomap::OcTree *tree = nullptr;

    if (fs::path(opts.input_path).extension() == ".bt") {
      binary_tree = std::make_unique<octomap::OcTree>(opts.resolution);
      if (!binary_tree->readBinary(opts.input_path)) {
        throw std::runtime_error("failed to read binary octomap: " + opts.input_path);
      }
      tree = binary_tree.get();
    } else {
      abstract_tree.reset(octomap::AbstractOcTree::read(opts.input_path));
      if (!abstract_tree) {
        throw std::runtime_error("failed to read octomap: " + opts.input_path);
      }
      tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
      if (!tree) {
        throw std::runtime_error("unsupported octree type: " + abstract_tree->getTreeType());
      }
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    int leaf_count = 0;
    int occupied_leaf_count = 0;
    int free_leaf_count = 0;
    std::vector<std::array<double, 3>> occupied_points;

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      const double x = it.getX();
      const double y = it.getY();
      const double half = it.getSize() * 0.5;
      min_x = std::min(min_x, x - half);
      min_y = std::min(min_y, y - half);
      max_x = std::max(max_x, x + half);
      max_y = std::max(max_y, y + half);
      leaf_count += 1;
      if (tree->isNodeOccupied(*it)) {
        occupied_leaf_count += 1;
        if (!opts.pcd_output_path.empty()) {
          occupied_points.push_back({x, y, it.getZ()});
        }
      } else {
        free_leaf_count += 1;
      }
    }

    if (leaf_count == 0) {
      throw std::runtime_error("octomap has no leaf nodes");
    }

    Grid grid;
    grid.resolution = opts.resolution;
    grid.origin_x = std::floor((min_x - opts.border_padding) / opts.resolution) * opts.resolution;
    grid.origin_y = std::floor((min_y - opts.border_padding) / opts.resolution) * opts.resolution;
    grid.width = static_cast<int>(
        std::ceil((max_x + opts.border_padding - grid.origin_x) / opts.resolution)) + 1;
    grid.height = static_cast<int>(
        std::ceil((max_y + opts.border_padding - grid.origin_y) / opts.resolution)) + 1;
    grid.obstacle_count.assign(grid.width * grid.height, 0);
    grid.free_count.assign(grid.width * grid.height, 0);

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      const bool occupied = tree->isNodeOccupied(*it);
      const double z = it.getZ();
      if (occupied && z > opts.robot_height) {
        continue;
      }
      const bool free_ground = occupied && z < opts.ground_threshold;
      fill_cell_range(grid, it.getX(), it.getY(), it.getSize(), occupied, free_ground);
    }

    std::cout << "[octomap_to_2d_grid] Read " << leaf_count << " leaves from " << opts.input_path
              << " (" << occupied_leaf_count << " occupied, " << free_leaf_count << " free)\n";
    write_outputs(grid, opts);
    if (!opts.pcd_output_path.empty()) {
      write_ascii_pcd(occupied_points, opts.pcd_output_path);
      std::cout << "[octomap_to_2d_grid] Wrote occupied-voxel PCD " << opts.pcd_output_path
                << " (" << occupied_points.size() << " points)\n";
    }
  } catch (const std::exception &exc) {
    std::cerr << "ERROR: " << exc.what() << "\n";
    return 1;
  }
  return 0;
}
