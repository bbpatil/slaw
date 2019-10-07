#if !defined(CHURN_OBSERVER_H)
#define CHURN_OBSERVER_H

#include <vector>

#include <omnetpp.h>
#include "PositionObserver.h"

class ChurnObserver : public PositionObserver {
protected:
  /** @brief The ID of the observed area. This parameter could be set by the 
   *  user. If no value is given, then this parameters gets the 
   *  ID of the most popular area of the self-similar map used in the 
   *  simulation */
  unsigned area_id;
  /** The duration of an observation window*/
  double window_size;
  /** @brief The number of walkers in a simulation*/
  unsigned walker_num;
  /** @brief The number of walkers into the observed area per time window */ 
  unsigned membership_counter;
  /** @brief The number of departures per window per time window */
  unsigned departure_counter;
  /** @brief The number of arrivals per window per time window */
  unsigned arrival_counter;
  /** @brief The time at which a walker arrives to the observed area */

};

#endif // CHURN_OBSERVER_H
