//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <cmath>

#include <cstddef>
#include <stdlib.h>

Ped::Tagent::Tagent(int posX, int posY) { Ped::Tagent::init(posX, posY); }

Ped::Tagent::Tagent(double posX, double posY) {
  Ped::Tagent::init((int)std::round(posX), (int)std::round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
  initial_x = (float)posX;
  initial_y = (float)posY;

  p_x = nullptr;
  p_y = nullptr;
  p_desiredPositionX = nullptr;
  p_desiredPositionY = nullptr;
  p_destX = nullptr;
  p_destY = nullptr;

  destination = NULL;
  lastDestination = NULL;
}

void Ped::Tagent::computeNextDesiredPosition() {
  destination = getNextDestination();
  if (destination == NULL) {
    // no destination, no need to
    // compute where to move to
    return;
  }

  double diffX = destination->getx() - *p_x;
  double diffY = destination->gety() - *p_y;
  double len = sqrt(diffX * diffX + diffY * diffY);
  *p_desiredPositionX = (float)round(*p_x + diffX / len);
  *p_desiredPositionY = (float)round(*p_y + diffY / len);
}

void Ped::Tagent::updateWaypoint() {
  destination = getNextDestination();
  if (destination != NULL) {
    *p_destX = (float)destination->getx();
    *p_destY = (float)destination->gety();
  }
}

void Ped::Tagent::setSoAPointers(float *x, float *y, float *dX, float *dY,
                                 float *desX, float *desY) {
  p_x = x;
  p_y = y;
  p_destX = dX;
  p_destY = dY;
  p_desiredPositionX = desX;
  p_desiredPositionY = desY;

  // Initialize SoA array with initaial values
  *p_x = initial_x;
  *p_y = initial_y;
  *p_desiredPositionX = initial_x;
  *p_desiredPositionY = initial_y;

  if (destination) {
    *p_destX = (float)destination->getx();
    *p_destY = (float)destination->gety();
  } else {
    *p_destX = initial_x;
    *p_destY = initial_y;
  }
}

const std::deque<Ped::Twaypoint *> &Ped::Tagent::getWaypoints() const {
  return waypoints;
}

void Ped::Tagent::addWaypoint(Twaypoint *wp) { waypoints.push_back(wp); }

Ped::Twaypoint *Ped::Tagent::getNextDestination() {
  Ped::Twaypoint *nextDestination = NULL;
  bool agentReachedDestination = false;

  if (destination != NULL) {
    // compute if agent reached its current destination
    double diffX = destination->getx() - *p_x;
    double diffY = destination->gety() - *p_y;
    double length = sqrt(diffX * diffX + diffY * diffY);
    agentReachedDestination = length < destination->getr();
  }

  if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) {
    // Case 1: agent has reached destination (or has no current destination);
    // get next destination if available
    waypoints.push_back(destination);
    nextDestination = waypoints.front();
    waypoints.pop_front();
  } else {
    // Case 2: agent has not yet reached destination, continue to move towards
    // current destination
    nextDestination = destination;
  }

  return nextDestination;
}
