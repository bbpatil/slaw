#include "InterContactTimeObserver.h"

Define_Module(InterContactTimeObserver);

omnetpp::simsignal_t InterContactTimeObserver::inter_contact_time =     
  registerSignal("interContactTime");

void InterContactTimeObserver::initialize() {
  PositionObserver::initialize();
  sample_size = par("sampleSize");
  ict_min = par("minICT");
  network_size = par("numOfNodes");
  ict_t.resize(network_size);
  WATCH(counter);
}

void InterContactTimeObserver::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, omnetpp::cObject* details) {
  PositionObserver::receiveSignal(src, id, value, details);
  std::unordered_map<unsigned, omnetpp::simtime_t> n( //current neighborhood
    computeOneHopNeighborhood(nodeId)
  );

  //Computes inter-contact time
  for (auto& neighbor : n) {
    EV_INFO << "A link between node " << nodeId << " and " << neighbor.first
      << " has been established at time "<< omnetpp::simTime();
    if (ict_t[nodeId].find(neighbor.first) == ict_t[nodeId].end())
      ict_t[nodeId][neighbor.first] = std::make_pair<omnetpp::simtime_t, bool>(
        0.0, true
      );
    else if (ict_t[nodeId][neighbor.first].second == false) {
      ict_t[nodeId][neighbor.first].second = true;
      auto ict = omnetpp::simTime() - ict_t[nodeId][neighbor.first].second;
      if (ict > ict_min ) {
        emit(inter_contact_time, ict);
        counter++;
      }
      if (counter == sample_size) {
        std::cout << "InterContactTimeObserver: " << counter
          << " observations have been gathered" << '\n';
        endSimulation();
      }
    }
  }

  for (auto& neighbor : ict_t[nodeId]) 
    for (auto& current_neighbor : n) {
      if (n.find(neighbor.first) == n.end()) {
        EV_INFO << "Link from " << nodeId << " to " << neighbor.first 
          << " is broken\n";
        ict_t[nodeId][current_neighbor.first].first = omnetpp::simTime();
        ict_t[nodeId][current_neighbor.first].second = false;
      }
  }
}