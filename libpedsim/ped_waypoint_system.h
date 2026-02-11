#ifndef _ped_waypoint_system_h_
#define _ped_waypoint_system_h_

#include <stdlib.h>

struct WaypointSystem {
  float *x;
  float *y;
  float *radius;
  int capacity;
  int count;

  void allocate(int maxWaypoints) {
    capacity = (maxWaypoints + 15) / 16 * 16; // Pad to 16
    posix_memalign((void **)&x, 64, capacity * sizeof(float));
    posix_memalign((void **)&y, 64, capacity * sizeof(float));
    posix_memalign((void **)&radius, 64, capacity * sizeof(float));
    count = 0;
  }

  int addWaypoint(float wx, float wy, float wr) {
    int idx = count++;
    x[idx] = wx;
    y[idx] = wy;
    radius[idx] = wr;
    return idx;
  }
};

#endif
