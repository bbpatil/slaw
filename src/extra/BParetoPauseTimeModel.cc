#include "BParetoPauseTimeModel.h"

Define_Module(BParetoPauseTimeModel);

void BParetoPauseTimeModel::initialize() {
  alpha = par("alpha").doubleValue();
  x_min = par("min").doubleValue();
  x_max = par("max").doubleValue();
}

double BParetoPauseTimeModel::computePauseTime() {
  double u = uniform(0, 1);
  double op1 = pow(x_max, alpha);
  double op2 = pow(x_min, alpha);
  return pow(-(u*op1 - u*op2 - op1) / pow(x_min*x_max, alpha), -1.0/alpha);
}