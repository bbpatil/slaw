#include "ContactTimeObserver.h"

Define_Module(ContactTimeObserver);

omnetpp::simsignal_t ContactTimeObserver::link_life_time = 
  registerSignal("linkLifeTime");

void ContactTimeObserver::initialize() {
  PositionObserver::initialize();
  sample_size = par("sampleSize");
  network_size = par("nimOfNodes");
  llt_min = par("minLLT");
  llt_t.resize(numOfNodes);
  WATCH(counter);
}

void ContactTimeObserver::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, omnetpp::cObject* details) {
  PositionObserver::receiveSignal(src, id, value, details);
  std::unordered_map<unsigned, omnetpp::simtime_t> n( //current neighborhood
    computeOneHopNeighborhood(nodeId)
  );
  //Computes new neighbors
  for (auto& neighbor : n) {
    if (llt_t[nodeId].find(neighbor.first) == llt_t[nodeId].end()) {
      EV_INFO << "A brand new link between node " << nodeId << " and "
        << neighbor.first << " has been established at time "
        << omnetpp::simTime() << '\n';
      llt_t[nodeId][neighbor.first] = std::make_pair<omnetpp::simtime_t, bool> (
        omnetpp::simTime(), true
      );
    }
    else if (llt_t[nodeId][neighbor.first].first == false) {
      EV_INFO << "A new link between node " << nodeId << " and " 
        << neighbor.first << " has been established at time "
        << omnetpp::simTime() << '\n';
      llt_t[nodeId][neighbor.first].first = omnetpp::simTime();
      llt_t[nodeId][neighbor.first].second = true;
    }
  }

  for (auto& neighbor : llt_t[nodeId]) 
    for (auto& current_neighbor : n){
      if (n.find(neighbor.first) == n.end())
        if (neighbor.second.second == true) {
          EV_INFO << "Link from " << nodeId << " to " << neighbor.first 
            << " is broken\n";
          auto llt = omnetpp::simTime() - neighbor.second.first;
          if (llt > llt_min) {
            emit(link_life_time, llt);
            counter++;
          }
          llt_t[nodeId][neighbor.first].first = 0.0;
          llt_t[nodeId][neighbor.first].second = false;
          if (counter == sample_size) {
            std::cout << "ContactTimeObserver: " << counter
              << " observations have been gathered" << '\n';
            endSimulation();
          }
      }
    }
}