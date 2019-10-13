#include "ConnectivityObserver.h"
#include <fstream>

Define_Module(ConnectivityObserver);

ConnectivityObserver::ConnectivityObserver()
  : min_contact_time(0), filename(nullptr), adjacency_matrix(nullptr) {
}
ConnectivityObserver::~ConnectivityObserver() {
  if (adjacency_matrix) {
    EV << "ConnectivityObserver: Delete adjacency matrix\n";
    delete adjacency_matrix;
  }
}

void ConnectivityObserver::initialize(int stage) {
  if (stage == 0) {
    PositionObserver::initialize(stage);
    min_contact_time = par("minContactTime").doubleValue();
    filename = par("filename").stringValue();
    //initialize data structure
    auto c = std::make_tuple(0.0, 0.0, false);
    std::vector<cell> row(numOfNodes, c);
    for (int i = 0; i < numOfNodes; i++)
      adjacency_matrix->push_back(row);
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

void ConnectivityObserver::finish() {
  std::ofstream ofs(filename);
  if (ofs.is_open) {
    for (auto row : *adjacency_matrix) {
      for (auto cell : row) {
        if (std::get<0>(cell) > min_contact_time)
          ofs << "1 ";
        else
          ofs << "0 ";
      }
      ofs << '\n';
    }
    ofs.close();
  }
  else
    error("ConnectivityObserver: file %s couldn't be open\n", filename);
}
