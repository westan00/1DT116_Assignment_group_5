//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <pthread.h>
#include <stack>
#include <thread>
#include <vector>
#include <cuda_runtime.h>

extern "C" void launch_cuda_tick(float* agentX, float* agentY, float* destX, float* destY, float* desiredX, float* desiredY, int n);

#ifndef NOCUDA
#include "cuda_testkernel.h"
#endif

#include <stdlib.h>

void Ped::Model::setup(std::vector<Ped::Tagent *> agentsInScenario,
                       std::vector<Twaypoint *> destinationsInScenario,
                       IMPLEMENTATION implementation) {
#ifndef NOCUDA
  // Convenience test: does CUDA work on this machine?
  cuda_test();
#else
  std::cout << "Not compiled for CUDA" << std::endl;
#endif

  // Set
  agents = std::vector<Ped::Tagent *>(agentsInScenario.begin(),
                                      agentsInScenario.end());

  // Set up destinations
  destinations = std::vector<Ped::Twaypoint *>(destinationsInScenario.begin(),
                                               destinationsInScenario.end());

  // Sets the chosen implemenation. Standard in the given code is SEQ
  this->implementation = implementation;

  // Allocate and initialize SoA arrays for VECTOR implementation
  num_agents = agents.size();
  if (implementation == Ped::VECTOR or implementation == Ped::VECTOROMP) {
    n_padded = (num_agents + 15) / 16 * 16; // Pad to multiple of 16 for AVX-512
  } else {
    n_padded = num_agents;
  }

  if (implementation == Ped::CUDA) {
    cudaMallocManaged(&agentX, n_padded * sizeof(float));
    cudaMallocManaged(&agentY, n_padded * sizeof(float));
    cudaMallocManaged(&destX, n_padded * sizeof(float));
    cudaMallocManaged(&destY, n_padded * sizeof(float));
    cudaMallocManaged(&desiredX, n_padded * sizeof(float));
    cudaMallocManaged(&desiredY, n_padded * sizeof(float));
  } else {
    posix_memalign((void **)&agentX, 64, n_padded * sizeof(float));
    posix_memalign((void **)&agentY, 64, n_padded * sizeof(float));
    posix_memalign((void **)&destX, 64, n_padded * sizeof(float));
    posix_memalign((void **)&destY, 64, n_padded * sizeof(float));
    posix_memalign((void **)&desiredX, 64, n_padded * sizeof(float));
    posix_memalign((void **)&desiredY, 64, n_padded * sizeof(float));
  }

  for (int i = 0; i < n_padded; ++i) {
    if (i < num_agents) {
      agents[i]->setSoAPointers(&agentX[i], &agentY[i], &destX[i], &destY[i],
                                &desiredX[i], &desiredY[i]);
    } else {
      // Padding
      agentX[i] = 0;
      agentY[i] = 0;
      destX[i] = 0;
      destY[i] = 0;
      desiredX[i] = 0;
      desiredY[i] = 0;
    }
  }

  // Set up heatmap (relevant for Assignment 4)
  setupHeatmapSeq();
}

/////////////////////////
/// CHUNK IMPLEMENTATION FOR PTHREAD
//////////////////
void Ped::Model::tick_thread(const int num_threads, int id) {
  int n_agents = agents.size();
  int chunk_size = n_agents / num_threads;
  int remainder = n_agents % num_threads;

  int start = id * chunk_size + (id < remainder ? id : remainder);
  int end = start + chunk_size + (id < remainder ? 1 : 0);

  for (int i = start; i < end; ++i) {
    auto *agent = agents[i];
    agent->computeNextDesiredPosition();
    agent->setX(agent->getDesiredX());
    agent->setY(agent->getDesiredY());
  }
}

struct BarrierData {
  pthread_barrier_t start_barrier;
  pthread_barrier_t done_barrier;
  Ped::Model *model;
  int num_threads;
  bool running;
};

static BarrierData bd;

void *barrier_worker(void *arg) {
  long id = (long)arg;
  BarrierData *data = &bd;

  while (data->running) {
    pthread_barrier_wait(&data->start_barrier);
    data->model->tick_thread(data->num_threads, id);
    pthread_barrier_wait(&data->done_barrier);
  }
  return NULL;
}

__global__ void Ped::Model::cuda_tick() {
#pragma omp parallel for
  for (int i = 0; i < num_agents; ++i) {
    agents[i]->updateWaypoint();
  }
  // Parallelized Vectorized calculation (OMP + AVX-512)
#pragma omp parallel for
  for (int i = 0; i < n_padded; i += 16) {
    __m512 ax = _mm512_load_ps(&agentX[i]);
    __m512 ay = _mm512_load_ps(&agentY[i]);
    __m512 dx = _mm512_load_ps(&destX[i]);
    __m512 dy = _mm512_load_ps(&destY[i]);

    __m512 diffX = _mm512_sub_ps(dx, ax);
    __m512 diffY = _mm512_sub_ps(dy, ay);

    __m512 lenSq =
        _mm512_add_ps(_mm512_mul_ps(diffX, diffX), _mm512_mul_ps(diffY, diffY));
    __m512 len = _mm512_sqrt_ps(lenSq);

    __m512 stepX = _mm512_div_ps(diffX, len);
    __m512 stepY = _mm512_div_ps(diffY, len);

    __m512 desX = _mm512_add_ps(ax, stepX);
    __m512 desY = _mm512_add_ps(ay, stepY);

    _mm512_store_ps(&desiredX[i], desX);
    _mm512_store_ps(&desiredY[i], desY);

    _mm512_store_ps(&agentX[i], desX);
    _mm512_store_ps(&agentY[i], desY);
  }
}

void Ped::Model::tick() {
  // EDIT HERE FOR ASSIGNMENT 1
  switch (this->implementation) {
  case Ped::SEQ: {
    for (Ped::Tagent *agent : agents) {
      agent->computeNextDesiredPosition();
      agent->setX(agent->getDesiredX());
      agent->setY(agent->getDesiredY());
    }
    break;
  }
  case Ped::OMP: {
#pragma omp parallel for default(none) shared(agents)
    for (int i = 0; i < agents.size(); ++i) {
      agents[i]->computeNextDesiredPosition();
      agents[i]->setX(agents[i]->getDesiredX());
      agents[i]->setY(agents[i]->getDesiredY());
    }
    break;
  }
  case Ped::PTHREAD: {
    static bool initialized = false;

    if (!initialized) {
      char *env = getenv("PTHREAD_NUM_THREADS");
      bd.num_threads = env ? atoi(env) : 8;
      cout << "Number of threads: " << bd.num_threads;
      cout << "\n";
      bd.model = this;
      bd.running = true;

      pthread_barrier_init(&bd.start_barrier, NULL, bd.num_threads + 1);
      pthread_barrier_init(&bd.done_barrier, NULL, bd.num_threads + 1);

      pthread_t t;
      for (long i = 0; i < bd.num_threads; i++) {
        pthread_create(&t, NULL, barrier_worker, (void *)i);
      }
      initialized = true;
    }

    pthread_barrier_wait(&bd.start_barrier);
    pthread_barrier_wait(&bd.done_barrier);

    break;
  }
  case Ped::VECTOR: {
    for (int i = 0; i < num_agents; ++i) {
      agents[i]->updateWaypoint();
    }
    for (int i = 0; i < n_padded; i += 16) {
      __m512 ax = _mm512_load_ps(&agentX[i]);
      __m512 ay = _mm512_load_ps(&agentY[i]);
      __m512 dx = _mm512_load_ps(&destX[i]);
      __m512 dy = _mm512_load_ps(&destY[i]);

      __m512 diffX = _mm512_sub_ps(dx, ax);
      __m512 diffY = _mm512_sub_ps(dy, ay);

      __m512 lenSq = _mm512_add_ps(_mm512_mul_ps(diffX, diffX),
                                   _mm512_mul_ps(diffY, diffY));
      __m512 len = _mm512_sqrt_ps(lenSq);

      __m512 stepX = _mm512_div_ps(diffX, len);
      __m512 stepY = _mm512_div_ps(diffY, len);

      __m512 desX = _mm512_add_ps(ax, stepX);
      __m512 desY = _mm512_add_ps(ay, stepY);

      _mm512_store_ps(&desiredX[i], desX);
      _mm512_store_ps(&desiredY[i], desY);

      _mm512_store_ps(&agentX[i], desX);
      _mm512_store_ps(&agentY[i], desY);
    }
    break;
  }
  case Ped::VECTOROMP: {
#pragma omp parallel for
    for (int i = 0; i < num_agents; ++i) {
      agents[i]->updateWaypoint();
    }
    // Parallelized Vectorized calculation (OMP + AVX-512)
#pragma omp parallel for
    for (int i = 0; i < n_padded; i += 16) {
      __m512 ax = _mm512_load_ps(&agentX[i]);
      __m512 ay = _mm512_load_ps(&agentY[i]);
      __m512 dx = _mm512_load_ps(&destX[i]);
      __m512 dy = _mm512_load_ps(&destY[i]);

      __m512 diffX = _mm512_sub_ps(dx, ax);
      __m512 diffY = _mm512_sub_ps(dy, ay);

      __m512 lenSq = _mm512_add_ps(_mm512_mul_ps(diffX, diffX),
                                   _mm512_mul_ps(diffY, diffY));
      __m512 len = _mm512_sqrt_ps(lenSq);

      __m512 stepX = _mm512_div_ps(diffX, len);
      __m512 stepY = _mm512_div_ps(diffY, len);

      __m512 desX = _mm512_add_ps(ax, stepX);
      __m512 desY = _mm512_add_ps(ay, stepY);

      _mm512_store_ps(&desiredX[i], desX);
      _mm512_store_ps(&desiredY[i], desY);

      _mm512_store_ps(&agentX[i], desX);
      _mm512_store_ps(&agentY[i], desY);
    }
    break;
  }
  case Ped::CUDA: {
    cuda_tick<<<1, 1>>>();
    cudaDeviceSynchronize();
    break;
  }
  default: {
    for (Ped::Tagent *agent : agents) {
      agent->computeNextDesiredPosition();
      agent->setX(agent->getDesiredX());
      agent->setY(agent->getDesiredY());
    }
  }
  }
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
void Ped::Model::move(Ped::Tagent *agent) {
  // Search for neighboring agents
  set<const Ped::Tagent *> neighbors =
      getNeighbors(agent->getX(), agent->getY(), 2);

  // Retrieve their positions
  std::vector<std::pair<int, int>> takenPositions;
  for (std::set<const Ped::Tagent *>::iterator neighborIt = neighbors.begin();
       neighborIt != neighbors.end(); ++neighborIt) {
    std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
    takenPositions.push_back(position);
  }

  // Compute the three alternative positions that would bring the agent
  // closer to his desiredPosition, starting with the desiredPosition itself
  std::vector<std::pair<int, int>> prioritizedAlternatives;
  std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
  prioritizedAlternatives.push_back(pDesired);

  int diffX = pDesired.first - agent->getX();
  int diffY = pDesired.second - agent->getY();
  std::pair<int, int> p1, p2;
  if (diffX == 0 || diffY == 0) {
    // Agent wants to walk straight to North, South, West or East
    p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
    p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
  } else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, agent->getY());
    p2 = std::make_pair(agent->getX(), pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);

  // Find the first empty alternative position
  for (std::vector<pair<int, int>>::iterator it =
           prioritizedAlternatives.begin();
       it != prioritizedAlternatives.end(); ++it) {

    // If the current position is not yet taken by any neighbor
    if (std::find(takenPositions.begin(), takenPositions.end(), *it) ==
        takenPositions.end()) {

      // Set the agent's position
      agent->setX((*it).first);
      agent->setY((*it).second);

      break;
    }
  }
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents
/// (search field is a square in the current implementation)
set<const Ped::Tagent *> Ped::Model::getNeighbors(int x, int y,
                                                  int dist) const {

  // create the output list
  // ( It would be better to include only the agents close by, but this
  // programmer is lazy.)
  return set<const Ped::Tagent *>(agents.begin(), agents.end());
}

void Ped::Model::cleanup() {
  // Nothing to do here right now.
}

Ped::Model::~Model() {
  if (implementation == Ped::CUDA) {
    cudaFree(agentX);
    cudaFree(agentY);
    cudaFree(destX);
    cudaFree(destY);
    cudaFree(desiredX);
    cudaFree(desiredY);
  } else {
    free(agentX);
    free(agentY);
    free(destX);
    free(destY);
    free(desiredX);
    free(desiredY);
  }

  std::for_each(agents.begin(), agents.end(),
                [](Ped::Tagent *agent) { delete agent; });
  std::for_each(destinations.begin(), destinations.end(),
                [](Ped::Twaypoint *destination) { delete destination; });
}
