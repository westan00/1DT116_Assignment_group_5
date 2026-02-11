//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <cstddef>
#include <math.h>

#include <stdlib.h>

Ped::Tagent::Tagent(int posX, int posY) { Ped::Tagent::init(posX, posY); }

Ped::Tagent::Tagent(double posX, double posY) {
  Ped::Tagent::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
  initX = (float)posX;
  initY = (float)posY;

  pX = nullptr;
  pY = nullptr;
  pDestX = nullptr;
  pDestY = nullptr;
  pDesiredPositionX = nullptr;
  pDesiredPositionY = nullptr;

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

  double diffX = destination->getx() - *pX;
  double diffY = destination->gety() - *pY;
  double len = sqrt(diffX * diffX + diffY * diffY);
  *pDesiredPositionX = (int)round(*pX + diffX / len);
  *pDesiredPositionY = (int)round(*pY + diffY / len);
}

void Ped::Tagent::updateWaypoint() {
  destination = getNextDestination() if (destination != NULL) {
    *pDestX = (float)destination->getx();
    *pDestY = (float)destination->gety();
  }
}

void Ped::Tagent::addWaypoint(Twaypoint *wp) { waypoints.push_back(wp); }

void Ped::Tagent::setPointers(float *x, float *y, float *dX, float *dY,
                              float *desX, float *desY) {
  pX = x;
  pY = y;
  pDestX = dX;
  pDestY = dY;
  pDesiredPosX = desX;
  pDesiredPosY = desY;

  *pX = initX;
  *pY = initY;
  *pDesiredPosX = initX;
  *pDesiredPosY = initY;

  if (destination) {
    *pDestX = (float)destination->getx();
    *pDestY = (float)destination->gety();
  } else {
    *pDestX = initX;
    *pDestY = initY;
  }
}

Ped::Twaypoint *Ped::Tagent::getNextDestination() {
  Ped::Twaypoint *nextDestination = NULL;
  bool agentReachedDestination = false;

  if (destination != NULL) {
    // compute if agent reached its current destination
    double diffX = destination->getx() - *pX;
    double diffY = destination->gety() - *pY;
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
