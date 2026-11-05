#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include "nav_stack/planners.hpp"

using namespace nav_stack;

static bool readPgm(const std::string& file, int& w, int& h, std::vector<uint8_t>& px) {
  std::ifstream f(file, std::ios::binary);
  std::string magic;
  int maxv;
  f >> magic >> w >> h >> maxv;
  f.get();
  if (magic != "P5") return false;
  px.resize(w * h);
  f.read(reinterpret_cast<char*>(px.data()), w * h);
  return static_cast<bool>(f);
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: bench_planners cases.txt [rrt_runs]\n";
    return 1;
  }
  const int rrt_runs = argc > 2 ? std::stoi(argv[2]) : 5;
  std::ifstream list(argv[1]);
  std::string line;
  std::cout << "case,planner,run,success,time_ms,length,expansions\n";
  int case_id = 0;
  while (std::getline(list, line)) {
    std::istringstream ss(line);
    std::string file;
    double res;
    Point s, g;
    if (!(ss >> file >> res >> s.x >> s.y >> g.x >> g.y)) continue;
    int w, h;
    std::vector<uint8_t> px;
    if (!readPgm(file, w, h, px)) {
      std::cerr << "bad map " << file << "\n";
      continue;
    }
    GridInfo info;
    info.width = w;
    info.height = h;
    info.resolution = res;
    std::vector<int8_t> occ(w * h);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) occ[y * w + x] = px[(h - 1 - y) * w + x] < 128 ? 100 : 0;
    CostMap cm(info, occ, CostMap::Params{});

    auto out = [&](const char* name, int run, const PlanResult& r) {
      std::cout << case_id << "," << name << "," << run << "," << r.success << "," << r.time_ms << ","
                << (r.success ? pathLength(r.path) : 0.0) << "," << r.expansions << "\n";
    };
    out("astar", 0, planAStar(cm, s, g));
    for (int run = 0; run < rrt_runs; ++run) {
      std::mt19937 rng(1000 * case_id + run);
      RrtStarParams p;
      p.time_budget_ms = 1e9;
      p.max_iterations = 3000;
      out("rrtstar_3000", run, planRrtStar(cm, s, g, p, rng));
      std::mt19937 rng2(1000 * case_id + run);
      p.stop_at_first_solution = true;
      p.max_iterations = 20000;
      out("rrt_first", run, planRrtStar(cm, s, g, p, rng2));
    }
    ++case_id;
  }
}