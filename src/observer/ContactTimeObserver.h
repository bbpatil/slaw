#if !defined(CONTACT_TIME_OBSERVER_H)
#define CONTACT_TIME_OBSERVER_H 

#include <vector>
#include <list>
#include <unordered_map>
#include <cmath>
#include <utility>

#include <omnetpp.h>
#include "PositionObserver.h"

class ContactTimeObserver : public PositionObserver {
protected:
  /** @brief The link lifetime table (llt) is a data structure keeping the time 
   * from which a relationship of neighbors begins. The llt is mapped in such a 
   * way that the indices of the vector correspond to node IDs and the value of 
   * the vector is an unordered map where the key represents the ID of a 
   * neighbor node and the value the time from which the relationship of 
   * neighbors begins. The intercontact time table (ictt) follows the same 
   * organization as the llt, but the ictt stores the time from which a 
   * neighbor node y leaves a neighborhood N(x), that is, y is not in N(x) */
  std::vector < std::unordered_map <unsigned, std::pair<omnetpp::simtime_t, bool > > > llt_t;
  /** @brief These are signal carrying the statistics asociated to its namely */
  static omnetpp::simsignal_t link_life_time;
  omnetpp::simtime_t llt_min;
  unsigned network_size;
public:
  /** @brief initializes data structures and the value of radius */
  virtual void initialize() override;
  /** @brief Receives the quadrant of a module and updates its one-hop 
   * neighborhood */
  virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t,  
    omnetpp::cObject*, omnetpp::cObject*) override;
};

#endif // CONTACT_TIME_OBSERVER_H 
