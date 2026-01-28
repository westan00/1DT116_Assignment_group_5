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

  // Set up destinations
  destinations = std::vector<Ped::Twaypoint *>(destinationsInScenario.begin(),
                                               destinationsInScenario.end());
  // Set up heatmap (relevant for Assignment 4)
  setupHeatmapSeq();
}

void tick_thread(std::vector<Ped::Tagent *>::iterator start,
                 std::vector<Ped::Tagent *>::iterator end) {
  for (auto it = start; it != end; ++it) {
    (*it)->computeNextDesiredPosition();
    (*it)->setX((*it)->getDesiredX());
    (*it)->setY((*it)->getDesiredY());
  }
}

void Ped::Model::tick() {
  // EDIT HERE FOR ASSIGNMENT 1
  switch (this->implementation) {
  case SEQ: {
    for (Ped::Tagent *agent : agents) {
      agent->computeNextDesiredPosition();
      agent->setX(agent->getDesiredX());
      agent->setY(agent->getDesiredY());
    }
    break;
  }
  case OMP: {
#pragma omp parallel for default(none) shared(agents)
    for (Ped::Tagent *agent : agents) {
      agent->computeNextDesiredPosition();
      agent->setX(agent->getDesiredX());
      agent->setY(agent->getDesiredY());
    }
    break;
  }
  case PTHREAD: {
    struct ThreadArg {
      std::vector<Ped::Tagent *>::iterator start;
      std::vector<Ped::Tagent *>::iterator end;
    };
    unsigned int num_threads = std::thread::hardware_concurrency();
    if (num_threads == 0)
      num_threads = 4;
    int chunk_size = agents.size() / num_threads;
    std::vector<pthread_t> threads(num_threads - 1);
    std::vector<ThreadArg> thread_args(num_threads - 1);

    auto start = agents.begin();
    for (unsigned int i = 0; i < num_threads - 1; ++i) {
      auto end = start + chunk_size;
      thread_args[i].start = start;
      thread_args[i].end = end;
      pthread_create(
          &threads[i], NULL,
          [](void *arg) -> void * {
            ThreadArg *t_arg = (ThreadArg *)arg;
            tick_thread(t_arg->start, t_arg->end);
            return NULL;
          },
          &thread_args[i]);
      start = end;
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
  std::for_each(agents.begin(), agents.end(),
                [](Ped::Tagent *agent) { delete agent; });
  std::for_each(destinations.begin(), destinations.end(),
                [](Ped::Twaypoint *destination) { delete destination; });
}
