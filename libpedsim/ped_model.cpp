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

#ifndef NOCDUA
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

  // Sets the chosen implemenation. Standard in the given code is SEQ
  this->implementation = implementation;

  // Allocate and initialize SoA arrays for VECTOR implementation
  num_agents = agents.size();
  if (implementation == Ped::VECTOR or implementation == Ped::VECTOROMP) {
    n_padded = (num_agents + 15) / 16 * 16; // Pad to multiple of 16 for AVX-512
  } else {
    n_padded = num_agents;
  }

  posix_memalign((void **)&agentX, 64, n_padded * sizeof(float));
  posix_memalign((void **)&agentY, 64, n_padded * sizeof(float));
  posix_memalign((void **)&destX, 64, n_padded * sizeof(float));
  posix_memalign((void **)&destY, 64, n_padded * sizeof(float));
  posix_memalign((void **)&desiredX, 64, n_padded * sizeof(float));
  posix_memalign((void **)&desiredY, 64, n_padded * sizeof(float));

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

  int totalWaypoints = 0;
  for (auto agent : agents) {
    totalWaypoints += agent->getWaypointCount();
  }

  waypoints.allocate(totalWaypoints);

  posix_memalign((void **)&agentWaypointStart, 64, n_padded * sizeof(int));
  posix_memalign((void **)&agentWaypointCount, 64, n_padded * sizeof(int));
  posix_memalign((void **)&agentCurrentWpIdx, 64, n_padded * sizeof(int));
  posix_memalign((void **)&agentWaypointGlobalIdx, 64, n_padded * sizeof(int));

  int currentWpIdx = 0;
  for (int i = 0; i < num_agents; ++i) {
    agentWaypointStart[i] = currentWpIdx;
    agentWaypointCount[i] = agents[i]->getWaypointCount();
    agentCurrentWpIdx[i] = 0;

    for (auto wp : agents[i]->getWaypoints()) {
      int idx = waypoints.addWaypoint(wp->getx(), wp->gety(), wp->getr());
      currentWpIdx++;
    }
    agentWaypointGlobalIdx[i] = agentWaypointStart[i];
    destX[i] = waypoints.x[agentWaypointGlobalIdx[i]];
    destY[i] = waypoints.y[agentWaypointGlobalIdx[i]];
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
    for (int i = 0; i < n_padded; i += 16) {
      // Load agent positions
      __m512 ax = _mm512_load_ps(&agentX[i]);
      __m512 ay = _mm512_load_ps(&agentY[i]);

      // Load current destination indices
      __m512i wpGlobalIdx = _mm512_load_epi32(&agentWaypointGlobalIdx[i]);

      // Gather current destination coords using indices
      __m512 dx = _mm512_i32gather_ps(wpGlobalIdx, waypoints.x, 4);
      __m512 dy = _mm512_i32gather_ps(wpGlobalIdx, waypoints.y, 4);
      __m512 radii = _mm512_i32gather_ps(wpGlobalIdx, waypoints.radius, 4);

      // Check if reached destination (distance < radius)
      __m512 diffX = _mm512_sub_ps(dx, ax);
      __m512 diffY = _mm512_sub_ps(dy, ay);
      __m512 distSq =
          _mm512_fmadd_ps(diffX, diffX, _mm512_mul_ps(diffY, diffY));
      __m512 radiusSq = _mm512_mul_ps(radii, radii);
      __mmask16 reached = _mm512_cmp_ps_mask(distSq, radiusSq, _CMP_LT_OQ);

      // For agents that reached, advance to next waypoint
      if (reached) { // If ANY agent reached
        __m512i currentWpIdx = _mm512_load_epi32(&agentCurrentWpIdx[i]);
        __m512i wpCount = _mm512_load_epi32(&agentWaypointCount[i]);
        __m512i wpStart = _mm512_load_epi32(&agentWaypointStart[i]);

        // Increment current waypoint index (with wraparound)
        __m512i nextWpIdx = _mm512_mask_add_epi32(
            currentWpIdx, reached, currentWpIdx, _mm512_set1_epi32(1));

        // Wraparound: if nextWpIdx >= wpCount, set to 0
        __mmask16 needWrap =
            _mm512_cmp_epi32_mask(nextWpIdx, wpCount, _MM_CMPINT_GE);
        nextWpIdx = _mm512_mask_blend_epi32(needWrap, nextWpIdx,
                                            _mm512_setzero_epi32());

        // Store updated waypoint index
        _mm512_store_epi32(&agentCurrentWpIdx[i], nextWpIdx);

        // Calculate new global waypoint index
        __m512i newGlobalIdx = _mm512_add_epi32(wpStart, nextWpIdx);
        _mm512_store_epi32(&agentWaypointGlobalIdx[i], newGlobalIdx);

        // Gather new destination coords
        __m512 newDx = _mm512_i32gather_ps(newGlobalIdx, waypoints.x, 4);
        __m512 newDy = _mm512_i32gather_ps(newGlobalIdx, waypoints.y, 4);

        // Update destX/destY only for agents that reached
        dx = _mm512_mask_blend_ps(reached, dx, newDx);
        dy = _mm512_mask_blend_ps(reached, dy, newDy);
      }

      // Now compute movement (same as before)
      __m512 diffX = _mm512_sub_ps(dx, ax);
      __m512 diffY = _mm512_sub_ps(dy, ay);
      __m512 lenSq = _mm512_fmadd_ps(diffX, diffX, _mm512_mul_ps(diffY, diffY));
      __m512 len = _mm512_sqrt_ps(lenSq);

      __m512 zero = _mm512_setzero_ps();
      __mmask16 mask = _mm512_cmp_ps_mask(len, zero, _CMP_GT_OQ);

      __m512 stepX = _mm512_maskz_div_ps(mask, diffX, len);
      __m512 stepY = _mm512_maskz_div_ps(mask, diffY, len);

      __m512 newX = _mm512_add_ps(ax, stepX);
      __m512 newY = _mm512_add_ps(ay, stepY);

      __m512 roundedX = _mm512_roundscale_ps(newX, _MM_FROUND_TO_NEAREST_INT);
      __m512 roundedY = _mm512_roundscale_ps(newY, _MM_FROUND_TO_NEAREST_INT);

      _mm512_store_ps(&agentX[i], roundedX);
      _mm512_store_ps(&agentY[i], roundedY);
      _mm512_store_ps(&desiredX[i], roundedX);
      _mm512_store_ps(&desiredY[i], roundedY);
      _mm512_store_ps(&destX[i], dx); // Store updated destinations
      _mm512_store_ps(&destY[i], dy);
    }
  } break;
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
  free(agentX);
  free(agentY);
  free(destX);
  free(destY);
  free(desiredX);
  free(desiredY);

  std::for_each(agents.begin(), agents.end(),
                [](Ped::Tagent *agent) { delete agent; });
  std::for_each(destinations.begin(), destinations.end(),
                [](Ped::Twaypoint *destination) { delete destination; });
}
