#include "ConstantPauseTimeModel.h"

Define_Module(ConstantPauseTimeModel);

void ConstantPauseTimeModel::initialize() {
  pause_time = par("pauseTime").doubleValue();
}

double ConstantPauseTimeModel::computePauseTime() {
  return pause_time;
}