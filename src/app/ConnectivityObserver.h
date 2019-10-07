#if !defined(CONNECTIVITY_OBSERVER)
#define CONNECTIVITY_OBSERVER

#include <vector>

#include <omnetpp.h>
#include "PositionObserver.h"

class ConnectivityObserver : public PositionObserver {
  /** @brief A data structure to track links between neighbors */
  std::vector< std::vector<bool> > adjacency_matrix;

};

#endif // CONNECTIVITY_OBSERVER
