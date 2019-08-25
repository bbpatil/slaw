#include "UniformSpeedModel.h"

Define_Module(UniformSpeedModel);

void UniformSpeedModel::initialize() {
  a = par("a").doubleValue();
  b = par("b").doubleValue();
}

double UniformSpeedModel::computeSpeed(double par) {
  return uniform(a, b);
}