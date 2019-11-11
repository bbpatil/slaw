#include "ConnectivityObserver.h"
Define_Module(ConnectivityObserver);

ConnectivityObserver::ConnectivityObserver()
  : adjacency_matrix(nullptr), neighborhood_list(nullptr), llt_min(0.0), observation_counter(0)
  { }

ConnectivityObserver::~ConnectivityObserver() {
  if (adjacency_matrix) {
    delete(adjacency_matrix);
    std::cout << "ConnectivityObserver: Delete adjacency_matrix\n";
  }
  if (neighborhood_list) {
    delete(neighborhood_list);
    std::cout << "ConnectivityObserver: Delete neighborhood list\n";
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
      std::vector<omnetpp::simtime_t> row(0.0, node_number);
      adjacency_matrix->push_back(std::move(row));
    }
  }
  if (stage == 2) {
    auto map_ptr = this->getSimulation()->getSystemModule()->
                    getSubmodule("tripmanager")->getSubmodule("mapmodule");
    polygon = reinterpret_cast<SelfSimilarWaypointMap*>(map_ptr)->getConvexHull();
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
        EV_INFO << "Distance between " << node_id << " and " << neighborId << " is " << distance << '\n';
        //Nodes are neighbors being at the observation area
        if ((distance < radius) && isInObservationArea(node_position[neighborId])) 
          neighborhood.push_back(std::make_pair(neighborId, omnetpp::simTime()));
      }
    }
  }
  return neighborhood;
}

void ConnectivityObserver::receiveSignal(omnetpp::cComponent* src, omnetpp::simsignal_t id, omnetpp::cObject* value, omnetpp::cObject* details) {
  auto state = dynamic_cast<inet::MovingMobilityBase*>(value);
  inet::Coord current_position(state->getCurrentPosition());
  if (isInObservationArea(current_position)) {
    PositionObserver::receiveSignal(src, id, value, details);
    std::list<neighbor> current_neighborhood = computeOneHopNeighborhood(node_id);
    std::list<neighbor> new_neighbor, old_neighbor;

    //Prints current neighborhood

    //Finds new neighbors
    for (auto& cn : current_neighborhood)
      if (
        std::find_if(
          neighborhood_list->at(node_id).begin(),
          neighborhood_list->at(node_id).end(),
          [cn] (neighbor x) { return cn.first == x.first; }
        ) == neighborhood_list->at(node_id).end()
      )
        neighborhood_list->at(node_id).push_back(cn);

    //Prints new-neighbor's id

    //Finds old neighbors, the iterator it points to a node in N(node_id)
    auto it =  neighborhood_list->at(node_id).begin();
    while (it != neighborhood_list->at(node_id).end()) {
      auto jt = std::find_if(
                  current_neighborhood.begin(),
                  current_neighborhood.end(),
                  [it] (neighbor x) { return x.first == it->first; }
                );
      if (jt == neighborhood_list->at(node_id).end()) {
        omnetpp::simtime_t link_lifetime = omnetpp::simTime() - it->second;
        if (link_lifetime > llt_min) {
          adjacency_matrix->at(node_id).at(it->first) += link_lifetime;
          observation_counter++;
        }
        neighborhood_list->at(node_id).erase(it++);
      }
      else 
        ++it;
    }

    //Prints old-neighbor's id
    

    if (observation_counter == sample_size) {
      std::cout << "ConnectivityObserver: total number of observations: "
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