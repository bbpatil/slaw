#ifndef ADJACENCY_MATRIX_H
#define ADJACENCY_MATRIX_H

#include <vector>
#include <iostream>

class AdjacencyMatrix {
private:
  std::vector< std::vector<unsigned> >* m;
public:
  AdjacencyMatrix() : m(nullptr) {}
  ~AdjacencyMatrix() {
    if (m)
      delete m;
  }
  void initialize(size_t size) {
    m = new std::vector <std::vector<unsigned> >;
    for (size_t i = 0; i < size; i++) {
      std::vector<unsigned> row(size, 0);
      m->push_back(std::move(row));
    }
  }
  const std::vector< std::vector<unsigned> >* get() const {
    const std::vector< std::vector<unsigned> >* n = m;
    return n;
  }
  unsigned& get(size_t row, size_t col) const {
    return m->at(row).at(col);
  }
  friend std::ostream& operator<<(std::ostream& os, const AdjacencyMatrix& am) {
    for (auto& row : *(am.get())) {
      for (auto& element : row)
        os << element << ' ';
      os << '\n';
    }
    return os;
  }
};

#endif