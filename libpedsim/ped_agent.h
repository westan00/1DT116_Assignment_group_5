//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp. 
//

#ifndef _ped_agent_h_
#define _ped_agent_h_ 1

#include <vector>
#include <deque>

using namespace std;

namespace Ped {
	class Twaypoint;

	class Tagent {
	public:
		Tagent(int posX, int posY);
		Tagent(double posX, double posY);

		// Returns the coordinates of the desired position
		int getDesiredX() const { return (int)round(*p_desiredPositionX); }
		int getDesiredY() const { return (int)round(*p_desiredPositionY); }

		// Sets the agent's position
		void setX(int newX) { *p_x = (float)newX; }
		void setY(int newY) { *p_y = (float)newY; }

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPosition();

		// Update only the waypoint if reached
		void updateWaypoint();

		// Position of agent defined by x and y
		int getX() const { return (int)round(*p_x); };
		int getY() const { return (int)round(*p_y); };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);

		// Returns the current destination
		Twaypoint* getDestination() const { return destination; }

		// Link to SoA storage in Model
		void setSoAPointers(float* x, float* y, float* dX, float* dY, float* dR, float* desX, float* desY) {
			p_x = x; p_y = y;
			p_destX = dX; p_destY = dY; p_destR = dR;
			p_desiredPositionX = desX; p_desiredPositionY = desY;
			// Initialize linked data with current values
			*p_x = staticX;
			*p_y = staticY;
			if (destination) {
				*p_destX = destination->getx();
				*p_destY = destination->gety();
				*p_destR = destination->getr();
			}
		}

	private:
		Tagent() {};

		// The agent's position (statically stored if not linked to SoA)
		float staticX;
		float staticY;
		float staticDesiredPositionX;
		float staticDesiredPositionY;
		float staticDestX;
		float staticDestY;
		float staticDestR;

		// Pointers to the active data (either static or in Model's SoA)
		float *p_x;
		float *p_y;
		float *p_desiredPositionX;
		float *p_desiredPositionY;
		float *p_destX;
		float *p_destY;
		float *p_destR;

		// The current destination (may require several steps to reach)
		Twaypoint* destination;

		// The last destination
		Twaypoint* lastDestination;

		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> waypoints;

		// Internal init function 
		void init(int posX, int posY);

		// Returns the next destination to visit
		Twaypoint* getNextDestination();
	};
}

#endif