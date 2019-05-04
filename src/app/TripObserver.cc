#include "TripObserver.h"

Define_Module(TripObserver);

omnetpp::simsignal_t
  TripObserver::trip_size = registerSignal("tripSize");
omnetpp::simsignal_t
  TripObserver::trip_size_stat = 
  registerSignal("tripSize_stat");
omnetpp::simsignal_t
  TripObserver::next_waypoint = registerSignal("nextWaypoint");
omnetpp::simsignal_t
  TripObserver::next_waypoint_x = registerSignal("nextWaypointX");
omnetpp::simsignal_t
  TripObserver::next_waypoint_y = registerSignal("nextWaypointY");

TripObserver::TripObserver() :
  counter(0), sample_size(0)
{
  getSimulation()->getSystemModule()->subscribe(trip_size, this);
  getSimulation()->getSystemModule()->subscribe(next_waypoint, this);
}

TripObserver::~TripObserver()
{
  getSimulation()->getSystemModule()->unsubscribe(trip_size, this);
  getSimulation()->getSystemModule()->unsubscribe(next_waypoint, this);
}

void TripObserver::initialize()
{
  sample_size = par("sampleSize");
  std::cout << "TripObserver: " << sample_size << " trips to be observed\n";
}

void TripObserver::handleMessage(omnetpp::cMessage* msg)
{
  error("Trip Observer: This module does not receive messages\n");
}

void TripObserver::receiveSignal(
  omnetpp::cComponent* src, omnetpp::simsignal_t id, long size, 
  omnetpp::cObject* details
) {
  counter++;
  if (counter >  sample_size)
    endSimulation();
  emit(trip_size_stat, size);
}

void TripObserver::receiveSignal(
  omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, 
  omnetpp::cObject* details
) {
  auto position = dynamic_cast<inet::Coord*>(value);
  emit(next_waypoint_x, position->x);
  emit(next_waypoint_y, position->y);
}