#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  long data_size = estimations.size();
  if (ground_truth.size() != data_size || data_size == 0) {
  	throw invalid_argument("number of estimations is uneaqual to number of groundtruth or zero length data");
  }

	VectorXd rmse;
	rmse = VectorXd(4);
	rmse << 0,0,0,0;
	for (long i = 0; i < data_size; i++) {
		VectorXd d = estimations[i] - ground_truth[i];
		d = d.array()*d.array();
		rmse += d;
  }
  rmse /= data_size;
  rmse = rmse.array().sqrt();
  return rmse;
}
