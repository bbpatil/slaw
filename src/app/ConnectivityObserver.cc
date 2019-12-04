#include "ConnectivityObserver.h"
Define_Module(ConnectivityObserver);

omnetpp::simsignal_t ConnectivityObserver::membership_stat = registerSignal("membership");

ConnectivityObserver::ConnectivityObserver()
  : adjacency_matrix(nullptr), neighborhood_list(nullptr), llt_min(0.0), observation_counter(0)
  { }

ConnectivityObserver::~ConnectivityObserver() {
  if (adjacency_matrix) {
    delete(adjacency_matrix);
    EV << "ConnectivityObserver: Delete adjacency_matrix\n";
  }
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
    neighborhood_list = new std::vector< std::list<neighbor> >(node_number);
    adjacency_matrix = new std::vector< std::vector<omnetpp::simtime_t> >;
    for (size_t i = 0; i < node_number; i++) {
      std::vector<omnetpp::simtime_t> row(node_number, 0.0);
      adjacency_matrix->push_back(std::move(row));
    }
  }
  if (stage == 2) {
    auto map_ptr = this->getSimulation()->getSystemModule()->
                    getSubmodule("tripmanager")->getSubmodule("mapmodule");
    polygon = reinterpret_cast<SelfSimilarWaypointMap*>(map_ptr)->getConvexHull();
    WATCH(membership_size);
    WATCH(observation_counter);
  }
}

std::list<ConnectivityObserver::neighbor> 
ConnectivityObserver::computeOneHopNeighborhood(int id) {
  std::list<neighbor> neighborhood;
  unsigned square = computeSquare(node_position[node_id]);
  std::list<unsigned> squareList(std::move(computeNeighboringSquares(square)));
  for (auto& square : squareList) {
    for (auto& neighborId: node_map[square]) {
      if (neighborId != node_id) {
        double distance = sqrt (
          pow(node_position[node_id].x - node_position[neighborId].x, 2) + 
          pow(node_position[node_id].y - node_position[neighborId].y, 2)
        );
        EV << "Distance between " << node_id << " and " << neighborId << " is " << distance << '\n';
        //Nodes are neighbors being at the observation area
        EV << "Node position: " << node_position[node_id] << '\n';
        EV << "Neighbor position: " << node_position[neighborId] << '\n';
        if ((distance < radius) && isInObservationArea(node_position[neighborId])) 
          neighborhood.push_back(std::make_pair(neighborId, omnetpp::simTime()));
      }
    }
  }
  return neighborhood;
}

void ConnectivityObserver::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, omnetpp::cObject* details) {
  static omnetpp::simtime_t current_time = 0.0;
  PositionObserver::receiveSignal(src, id, value, details);
  auto state = dynamic_cast<inet::MovingMobilityBase*>(value);
  inet::Coord current_position(state->getCurrentPosition());
  if (isInObservationArea(current_position)) {
    std::list<neighbor> current_neighborhood = computeOneHopNeighborhood(node_id);
    std::list<neighbor> new_neighbor, old_neighbor;

    //Prints current neighborhood
    EV << "Current neighborhood of node " << node_id << ":\n";
    for (auto& entry : current_neighborhood)
      EV << "\tid: " << entry.first << "\n";

    //Finds new neighbors
    //EV << "New neighbors of node " << node_id << ":\n";
    for (auto& cn : current_neighborhood) {
      if (
        std::find_if(
          neighborhood_list->at(node_id).begin(),
          neighborhood_list->at(node_id).end(),
          [cn] (neighbor x) { return cn.first == x.first; }
        ) == neighborhood_list->at(node_id).end()
      ) {
        neighborhood_list->at(node_id).push_back(cn);
        EV << "\tnew neighbor: " << cn.first << " time: " << cn.second << "\n";
      }
    }

    //Prints new-neighbor's id
    EV << "neighbor list of node: " << node_id << '\n';
    for (auto& n : neighborhood_list->at(node_id)) {
      EV << "neighbor: " << n.first << " at " << n.second << '\n';
    }

    //Finds old neighbors, the iterator points to a node in N(node_id)
    //EV << "Old neighbors of node " << node_id << ":\n";
    auto n_it =  neighborhood_list->at(node_id).begin();
    while (n_it != neighborhood_list->at(node_id).end()) {
      auto n_jt = std::find_if(
                  current_neighborhood.begin(),
                  current_neighborhood.end(),
                  [n_it] (neighbor x) { return x.first == n_it->first; }
                );
      if (n_jt == current_neighborhood.end()) {
        omnetpp::simtime_t link_lifetime = omnetpp::simTime() - n_it->second;
        if (link_lifetime > llt_min) {
          adjacency_matrix->at(node_id).at(n_it->first) += link_lifetime;
          observation_counter++;
          EV << "\told neighbor: " << n_it->first << " time: " << n_it->second << "\n";
        }
        EV << "Erase node: " << n_it->first << '\n';
        neighborhood_list->at(node_id).erase(n_it++);
      }
      else 
        ++n_it;
    }
    if (current_time == omnetpp::simTime()) {
      membership.insert(node_id);
      membership_size = membership.size();
    }
    else {
      emit(membership_stat, membership.size());
      membership.clear();
      membership.insert(node_id);
      membership_size = 1;
      current_time = omnetpp::simTime();
    }
    
    if (observation_counter == sample_size) {
      EV << "ConnectivityObserver: total number of observations: "
                << ' ' << observation_counter << '\n';
      endSimulation(); 
    }
  }
}

void ConnectivityObserver::finish() {
  std::ofstream ofs(filename);
  if (ofs.is_open()) {
    for (auto& row : *adjacency_matrix) {
      for (auto& time : row)
        ofs << time << ' ';
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