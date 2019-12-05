#include "ConnectivityObserver.h"
Define_Module(ConnectivityObserver);

omnetpp::simsignal_t ConnectivityObserver::membership_stat = registerSignal("membership");

ConnectivityObserver::ConnectivityObserver()
  : neighborhood_list(nullptr), 
    llt_min(0.0), 
    observation_counter(0), 
    msg(nullptr)
  { }

ConnectivityObserver::~ConnectivityObserver() {
  if (neighborhood_list) {
    delete(neighborhood_list);
    EV << "ConnectivityObserver: Delete neighborhood list\n";
  }
}

void ConnectivityObserver::initialize(int stage) {
  if (stage == 0) {
    PositionObserver::initialize(stage);
    llt_min = par("minLLT");
    filename = par("filename").stringValue();
    sample_size = par("observations");
    neighborhood_list = new std::vector< std::list<unsigned> >(node_number);
    adjacency_matrix.initialize(node_number);
  }
  if (stage == 2) {
    msg = new omnetpp::cMessage();
    msg->setSchedulingPriority(255); //the lowest priority
    scheduleAt(omnetpp::simTime(), msg);
    auto map_ptr = this->getSimulation()->getSystemModule()->
                    getSubmodule("tripmanager")->getSubmodule("mapmodule");
    polygon = reinterpret_cast<SelfSimilarWaypointMap*>(map_ptr)->getConvexHull();
    WATCH(membership_size);
    WATCH(observation_counter);
  }
}

std::list<unsigned> 
ConnectivityObserver::computeOneHopNeighborhood(unsigned id) {
  std::list<unsigned> neighborhood;
  unsigned square = computeSquare(node_position[id]);
  std::list<unsigned> squareList(std::move(computeNeighboringSquares(square)));
  for (auto& square : squareList) {
    for (auto& neighborId: node_map[square]) {
      if (neighborId != id) {
        double distance = sqrt (
          pow(node_position[id].x - node_position[neighborId].x, 2) + 
          pow(node_position[id].y - node_position[neighborId].y, 2)
        );
        std::cout << "Distance between " << id << " and " << neighborId << " is " << distance << '\n';
        //Nodes are neighbors being at the observation area
        std::cout << "Node position: " << node_position[id] << " at time: " << omnetpp::simTime() << '\n';
        std::cout << "Neighbor position: " << node_position[neighborId] << " at time: " << omnetpp::simTime() << '\n';
        if ((distance < radius) && isInObservationArea(node_position[neighborId])) 
          neighborhood.push_back(neighborId);
      }
    }
  }
  return neighborhood;
}

void ConnectivityObserver::receiveSignal(
  omnetpp::cComponent* src, 
  omnetpp::simsignal_t id, 
  omnetpp::cObject* value, 
  omnetpp::cObject* details
) {
  static omnetpp::simtime_t current_time = 0.0;
  PositionObserver::receiveSignal(src, id, value, details);
  auto state = dynamic_cast<inet::MovingMobilityBase*>(value);
  inet::Coord current_position(state->getCurrentPosition());
  if (isInObservationArea(current_position))
    membership.insert(node_id);
  else if (membership.find(node_id) != membership.end())
    membership.erase(node_id);
}

void ConnectivityObserver::finish() {
  std::ofstream ofs(filename);
  if (ofs.is_open()) {
    for (auto& row : *(adjacency_matrix.get())) {
      for (auto& time : row) {
        if (time > llt_min)
          ofs << time << ' ';
        else
          ofs << time << 0;
      }
      ofs << std::endl;
    }
    ofs.close();
  }
  else 
    error("ConnectivityObserver: %s file couldn't be opened\n", filename);
}

bool ConnectivityObserver::isInObservationArea(inet::Coord& position) {
  auto predicate = CGAL::bounded_side_2(
    polygon->begin(), polygon->end(), point_2(position.x, position.y), K()
  );
  return predicate != CGAL::ON_UNBOUNDED_SIDE;
}

void ConnectivityObserver::computeNewNeighbors(
  unsigned id,
  std::list<unsigned>& current_neighborhood
) {
  for (auto& cn : current_neighborhood) {
    if (
      std::find_if (
        neighborhood_list->at(id).begin(),
        neighborhood_list->at(id).end(),
        [cn] (unsigned x) { return cn == x; }
      ) == neighborhood_list->at(id).end()
    ) {
      neighborhood_list->at(id).push_back(cn);
      std::cout << "\tnode: " << cn << " is new neighbor of node " << id << " at " << omnetpp::simTime() << "\n";
    }
    else 
      //This increment only works if nodes emit a signal each second
      adjacency_matrix.get(id, cn)++;
  }
}

void ConnectivityObserver::computeOldNeighbors(
  unsigned id,
  std::list<unsigned>& current_neighborhood
) {
  auto n_it =  neighborhood_list->at(id).begin();
  while (n_it != neighborhood_list->at(id).end()) {
    auto n_jt = std::find_if(
                current_neighborhood.begin(),
                current_neighborhood.end(),
                [n_it] (unsigned x) { return x == *n_it; }
              );
    if (n_jt == current_neighborhood.end()) {
      EV << "Erase node: " << *n_it << '\n';
      neighborhood_list->at(node_id).erase(n_it++);
    }
    else 
      ++n_it;
  }
}

void ConnectivityObserver::handleMessage(omnetpp::cMessage* msg) {
  if (msg->isSelfMessage()) {
    std::cout << "Simulation time: " << omnetpp::simTime() <<'\n';
    std::cout << "Membership: ";
    for (auto& member : membership)
      std::cout << member << ' ';
    std::cout << '\n';
    for (auto& member : membership) {
      std::list<unsigned> current_neighborhood = 
        computeOneHopNeighborhood(member);
      std::list<unsigned> new_neighbor;

      std::cout << "Current neighborhood of node " 
                << member << "\n";
      for (auto& nid : current_neighborhood)
        std::cout << "\tid: " << nid << "\n";

      computeNewNeighbors(member, current_neighborhood);

      EV << "neighbor list of node: " << member << '\n';
      for (auto& nid : neighborhood_list->at(member))
        EV << "neighbor: " << nid << '\n';

      computeOldNeighbors(member, current_neighborhood);
    }
    std::cout << "Adjacency matrix\n" << adjacency_matrix;
    scheduleAt(omnetpp::simTime()+1.0, msg);
  }
  else
    error("Connectivity Observer: This module does not receive messages\n");
}
