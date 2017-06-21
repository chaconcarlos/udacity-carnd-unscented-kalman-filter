#ifndef SDCE_TOOLS_H_
#define SDCE_TOOLS_H_

/* INCLUDES ******************************************************************/

#include <vector>

#include "Eigen/Dense"

/* FUNCTIONS DECLARATION *****************************************************/

namespace Sdce
{
namespace Tools
{

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

} /* namespace Sdce */
} /* namespace Tools */

#endif /* SDCE_TOOLS_H_ */
