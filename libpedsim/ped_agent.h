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

#include <cmath>
#include <deque>
#include <vector>

using namespace std;

namespace Ped {
class Twaypoint;

class Tagent {
public:
  Tagent(int posX, int posY);
  Tagent(double posX, double posY);

  // Returns the coordinates of the desired position
  int getDesiredX() const { return *p_desiredPositionX; }
  int getDesiredY() const { return *p_desiredPositionY; }

  // Sets the agent's position
  void setX(int newX) { *p_x = newX; }
  void setY(int newY) { *p_y = newY; }

  // Update the position according to get closer
  // to the current destination
  void computeNextDesiredPosition();

  // Update only the waypoint if reached
  void updateWaypoint();

  // Position of agent defined by x and y
  int getX() const { return *p_x; };
  int getY() const { return *p_y; };

  int getInitialX() const { return initial_x; };
  int getInitialY() const { return initial_y; };

  // Adds a new waypoint to reach for this agent
  void addWaypoint(Twaypoint *wp);

  int getWaypointCount() const { return waypoints.size(); }
  const deque<Twaypoint *> &getWaypoints() const { return waypoints; }

  // Returns the current destination
  Twaypoint *getDestination() const { return destination; }

  // Returns the number of waypoints
  int getWaypointCount() const { return waypoints.size(); }

  // Returns the list of waypoints
  const std::deque<Twaypoint *> &getWaypoints() const { return waypoints; }

  // Link to SoA storage in Model
  void setSoAPointers(float *x, float *y, float *dX, float *dY, float *desX,
                      float *desY);

private:
  Tagent() {};

  // Pointers to the active data in Model's SoA
  float *p_x;
  float *p_y;
  float *p_desiredPositionX;
  float *p_desiredPositionY;
  float *p_destX;
  float *p_destY;

  float initial_x;
  float initial_y;

  Twaypoint *destination;
  Twaypoint *lastDestination;
  deque<Twaypoint *> waypoints;

  int waypointListStart;
  int waypointListCount;
  int currentWaypointIdx;

  void addWaypoint(int wpIdx);

  // Internal init function
  void init(int posX, int posY);

  // Returns the next destination to visit
  Twaypoint *getNextDestination();
};
} // namespace Ped

#endif
