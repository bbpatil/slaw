#include "UniformPauseTimeModel.h"

Define_Module(UniformPauseTimeModel);

void UniformPauseTimeModel::initialize() {
  a = par("a").doubleValue();
  b = par("b").doubleValue();
}

double UniformPauseTimeModel::computePauseTime() {
  return uniform(a, b);
}