#include "ConnectivityObserver.h"

Define_Module(ConnectivityObserver);

ConnectivityObserver::ConnectivityObserver()
  : min_contact_time(0), filename(nullptr) {
}

void ConnectivityObserver::initialize(int stage) {
  if (stage == 0) {
    min_contact_time = par("minContactTime").doubleValue();
    filename = par("filename").stringValue();
  }
}

std::list<unsigned> ConnectivityObserver::computeOneHopNeighborhood(unsigned id) {
  std::list<unsigned> neighborhood;
  return neighborhood;
}
void ConnectivityObserver::receiveSignal(
  omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, 
  omnetpp::cObject* details
) {
  //Aquí viene la magia
}
