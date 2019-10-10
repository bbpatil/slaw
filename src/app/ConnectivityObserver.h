#if !defined(CONNECTIVITY_OBSERVER)
#define CONNECTIVITY_OBSERVER

#include <vector>
#include <tuple>
#include <omnetpp.h>
#include "PositionObserver.h"

class ConnectivityObserver : public PositionObserver {
protected:
  /** @brief triplet of kind <link_duration, start, status> */
  typedef std::tuple<omnetpp::simtime_t, omnetpp::simtime_t, bool> cell;
  /** @brief A data structure to track links between neighbors. 
   *  The matrix keeps triples of kind <link_duration, start, status>. The 
   *  link_duration element is the overall time a link between two nodes
   *  was established. The start element is the time a link is established. The 
   *  status element is a boolean value indicating whether a link is up */
  std::vector< std::vector<cell> > adjacency_matrix;
  /** @brief The name of the file where the adjacency matrix is written */
  const char* filename;
  /** @brief The minimum time a link must last to consider a contact occurs. If 
   *  an application is used over a MANET, then this time could depend on the 
   *  frame transmision time */
  omnetpp::simtime_t min_contact_time;
protected:

public:
  /** @brief Default constructor */
  ConnectivityObserver();
  /** @brief Initializes the attributes of this class */
  virtual void initialize(int stage) override;
  /** @brief Receives the quadrant of a module and updates its one-hop 
   * neighborhood */
  virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t,  
    omnetpp::cObject*, omnetpp::cObject*) override;
  /** @brief Write the adjacency matrix in a file*/
  virtual void finish() override;
};

#endif // CONNECTIVITY_OBSERVER
