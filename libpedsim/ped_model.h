//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// Model coordinates a time step in a scenario: for each
// time step all agents need to be moved by one position if
// possible.
//
#ifndef _ped_model_h_
#define _ped_model_h_

#include "ped_agent.h"
#ifndef NOCUDA
#include <cuda_runtime.h>
#endif
#include <atomic>
#include <immintrin.h>
#include <map>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <set>
#include <vector>

namespace Ped {
class Tagent;

// The implementation modes for Assignment 1 + 2:
// chooses which implementation to use for tick()
enum IMPLEMENTATION {
  CUDA,
  VECTOR,
  OMP,
  PTHREAD,
  SEQ,
  VECTOR_OMP,
  CUDA_FULL,
  SEQ_REGION,
  OMP_REGION
};

class Model {
public:
  // Sets everything up
  void setup(std::vector<Tagent *> agentsInScenario,
             std::vector<Twaypoint *> destinationsInScenario,
             IMPLEMENTATION implementation, bool cuda_sync = true);

  // Coordinates a time step in the scenario: move all agents by one step (if
  // applicable).
  void tick();

  // Returns the agents of this scenario
  const std::vector<Tagent *> &getAgents() const { return agents; };

  // Thread work function for pthread
  void tick_thread(const int num_threads, int id);

  int find_region(int x, int y);

  // Adds an agent to the tree structure
  void placeAgent(const Ped::Tagent *a);

  // Cleans up the tree and restructures it. Worth calling every now and then.
  void cleanup();
  ~Model();

  // Returns the heatmap visualizing the density of agents
  int const *const *getHeatmap() const { return blurred_heatmap; };
  int getHeatmapSize() const;

private:
  // Denotes which implementation (sequential, parallel implementations..)
  // should be used for calculating the desired positions of
  // agents (Assignment 1)
  IMPLEMENTATION implementation;

  // Whether to use cudaDeviceSynchronize() after CUDA kernel launches
  bool cuda_sync;

  // The agents in this scenario
  std::vector<Tagent *> agents;

  // The waypoints in this scenario
  std::vector<Twaypoint *> destinations;

  // Arrays for SoA (Struct of Arrays) to support SIMD vectorization
  float *agentX;
  float *agentY;
  float *destX;
  float *destY;
  float *desiredX;
  float *desiredY;
  int num_agents;
  int n_padded;

  float *wpX, *wpY, *wpR;
  int *wpSequences, *wpSequencesLen, *currentWpIdx;
  int maxWpsPerAgent;

  int numRegions;

  struct Region {
    int id;
    int x_min, x_max, y_min, y_max;
    std::vector<Tagent *> agentsInRegion;
  };

  std::vector<Region> regions;
  std::vector<std::unique_ptr<std::mutex>> regionMutexes;

  // Moves an agent towards its next position
  void move(Region *region);
  void move(Ped::Tagent *agent);

  ////////////
  /// Everything below here won't be relevant until Assignment 3
  ///////////////////////////////////////////////

  // Returns the set of neighboring agents for the specified position
  set<const Ped::Tagent *> getNeighbors(int x, int y, int dist) const;

  ////////////
  /// Everything below here won't be relevant until Assignment 4
  ///////////////////////////////////////////////

#define SIZE 1024
#define CELLSIZE 5
#define SCALED_SIZE SIZE *CELLSIZE

  // The heatmap representing the density of agents
  int **heatmap;

  // The scaled heatmap that fits to the view
  int **scaled_heatmap;

  // The final heatmap: blurred and scaled to fit the view
  int **blurred_heatmap;

  void setupHeatmapSeq();
  void updateHeatmapSeq();

  // CUDA heatmap members
#ifndef NOCUDA
  int *d_heatmap;
  int *d_scaled_heatmap;
  int *d_blurred_heatmap;
  cudaStream_t heatmap_stream;
#endif
  void setupHeatmapCUDA();
};
} // namespace Ped
#endif
