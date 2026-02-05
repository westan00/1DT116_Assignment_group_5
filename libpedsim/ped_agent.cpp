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

#include <stdlib.h>

Ped::Tagent::Tagent(int posX, int posY) {
	Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
	Ped::Tagent::init((int)std::round(posX), (int)std::round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
	staticX = (float)posX;
	staticY = (float)posY;
	staticDesiredPositionX = (float)posX;
	staticDesiredPositionY = (float)posY;
	staticDestX = 0;
	staticDestY = 0;
	staticDestR = 0;

	p_x = &staticX;
	p_y = &staticY;
	p_desiredPositionX = &staticDesiredPositionX;
	p_desiredPositionY = &staticDesiredPositionY;
	p_destX = &staticDestX;
	p_destY = &staticDestY;
	p_destR = &staticDestR;

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
	double len = std::sqrt(diffX * diffX + diffY * diffY);
	*p_desiredPositionX = (float)std::round(*p_x + diffX / len);
	*p_desiredPositionY = (float)std::round(*p_y + diffY / len);
}

void Ped::Tagent::updateWaypoint() {
	destination = getNextDestination();
	if (destination != NULL) {
		*p_destX = (float)destination->getx();
		*p_destY = (float)destination->gety();
		*p_destR = (float)destination->getr();
	}
}

void Ped::Tagent::setSoAPointers(float* x, float* y, float* dX, float* dY, float* dR, float* desX, float* desY) {
	p_x = x; p_y = y;
	p_destX = dX; p_destY = dY; p_destR = dR;
	p_desiredPositionX = desX; p_desiredPositionY = desY;
	// Initialize linked data with current values
	*p_x = staticX;
	*p_y = staticY;
	if (destination) {
		*p_destX = (float)destination->getx();
		*p_destY = (float)destination->gety();
		*p_destR = (float)destination->getr();
	}
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
	Ped::Twaypoint* nextDestination = NULL;
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
	}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		nextDestination = destination;
	}

	return nextDestination;
}
