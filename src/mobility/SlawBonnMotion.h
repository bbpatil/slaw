#ifndef SLAW_BONNMOTION_H
#define SLAW_BONNMOTION_H

#include <fstream>
#include <string>
#include <unordered_map>

#include <omnetpp.h>
#include "BonnMotionMobility.h"
#include "../common/HashCoordinate.h"

class SlawBonnMotion : public inet::BonnMotionMobility {
private:
  /** @brief The number of trips a walker has completed */
  unsigned counter;
  /** @brief The last waypoint */
  inet::Coord lastWaypoint;
  /** @brief If true, then walker is in a waypoint, else walker is moving */
  bool pause, initial_waypoint;
  /** @brief Unordered map containing waypoints */
  std::unordered_map<inet::Coord, bool> waypointMap;
  /** @brief Signal informing node flight lengths */
  static omnetpp::simsignal_t flight;
  /** @brief Signal carrying the number of trips a walker has completed */
  static omnetpp::simsignal_t trip_counter;
private:
  /** @brief Overridden from BonnMotionMobility. */
  void setTargetPosition() override;
  /** @brief Overridden from BonnMotionMobility. */
  void setInitialPosition() override;
  /** @brief Initializes mobility model parameters. */
  void initialize(int stage) override;
  /** @brief Loads waypoints from a file an store them in waypointMap*/
  bool loadMap(const std::string&&);
public:
  SlawBonnMotion();
};

#endif