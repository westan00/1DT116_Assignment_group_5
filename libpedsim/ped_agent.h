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
  int getDesiredX() const { return round(*pDesiredPositionX); }
  int getDesiredY() const { return round(*pDesiredPositionY); }

  // Sets the agent's position
  void setX(int newX) { *pX = (float)newX; }
  void setY(int newY) { *pY = (float)newY; }

  // Update the position according to get closer
  // to the current destination
  void computeNextDesiredPosition();

  void updateWaypoint();

  // Position of agent defined by x and y
  int getX() const { return round(*pX); };
  int getY() const { return round(*pY); };

  getInitialX() const { return initX; };
  getInitialY() const { return initY; };

  // Adds a new waypoint to reach for this agent
  void addWaypoint(Twaypoint *wp);

  Twaypoint *getDestination() const { return destination; };

  void setPointers(float *x, float *y, float *dX, float *dY, float *desX,
                   float *desY);

private:
  Tagent() {};

  // The agent's current position
  float *pX;
  float *pY;
  float *pDestX;
  float *pDestY;
  float *pDesiredPositionX;
  float *pDesiredPositionY;

  // The agent's desired next position
  float initX;
  float initY;

  // The current destination (may require several steps to reach)
  Twaypoint *destination;

  // The last destination
  Twaypoint *lastDestination;

  // The queue of all destinations that this agent still has to visit
  deque<Twaypoint *> waypoints;

  // Internal init function
  void init(int posX, int posY);

  // Returns the next destination to visit
  Twaypoint *getNextDestination();
};
} // namespace Ped

#endif
