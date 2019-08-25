#include "NormalPauseTimeModel.h"

void NormalPauseTimeModel::initialize() {
  mean = par("mean").doubleValue();
  std_dev = par("stdDev").doubleValue();
}

double NormalPauseTimeModel::computePauseTime() {
  return truncnormal(mean, std_dev);
}