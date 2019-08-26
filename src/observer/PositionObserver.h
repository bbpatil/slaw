#if !defined(POSITION_OBSERVER_H)
#define POSITION_OBSERVER_H

#include <vector>
#include <list>
#include <algorithm>
#include <cmath>
#include <unordered_map>

#include "../contract/IObserver.h"
#include "Coord.h"
#include "MovingMobilityBase.h"

class PositionObserver: public IObserver{
protected:
  /** @brief The number of observations to be gathered to finish simulations */
  int counter;
  /** @brief  The name of the observer module */
  const char* observer_type;
  //TODO add statistic to measure the quadrant distribution
  /** @brief Data structure storing the position of nodes in a 
   * square position system, indices correspond to the square number and values 
   * are lists of the node IDs located in a square */
  std::vector< std::list<unsigned> > nodeMap;
  /**@ brief Data structures storing the square number of a node with index i */
  std::vector<inet::Coord> nodePosition;
  /** @brief the signal ID carriying the quadrant where a emitting node is 
   * located */
  static omnetpp::simsignal_t position;
  /** @brief The coverage radius of walkers in meters. It equals the length of 
   * the side of a square in a square position system. */
  double radius, x_length, y_length;
  /** @brief the number of nodes in the simulation */
  unsigned network_size;
  /** @brief the number of squares on an axis */
  unsigned x_num, y_num;
  /** @brief the Id of the node that emitted a signal */
  unsigned nodeId;
protected:
  /** @brief Returns the number of square given a x, y coordinate */
  unsigned computeSquare(const inet::Coord&);
  /** Returns the nine neighboring squares corresponding to a given square */
  std::list<unsigned> computeNeighboringSquares(unsigned);
  /** @brief Computes the one hop neighborhood of a node whose ID is passed as
   * an argument. Neighborhoods are computed using the nodeMap. This member function member also updates the ictt when receiving a signal */
  virtual std::unordered_map<unsigned, omnetpp::simtime_t>
  computeOneHopNeighborhood(unsigned);
public:
  /** Subscribes to signal quadrant and initializes the class attributes*/
  PositionObserver();
  /** Unsubscribes to signal quadrant */
  virtual ~PositionObserver();
  /** Initializes the attributes of this class */
  virtual void initialize() override;
  /** This module does not receive messages */
  virtual void handleMessage(omnetpp::cMessage*);
  /** Receives the quadrant of a module and updates its one-hop neighborhod*/
  virtual void receiveSignal(omnetpp::cComponent*, omnetpp::simsignal_t,  
    omnetpp::cObject*, omnetpp::cObject*);
};


#endif // POSITION_OBSERVER_H
