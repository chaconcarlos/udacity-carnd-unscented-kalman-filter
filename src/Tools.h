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
   * @brief Calculates the RMSE.
   *
   * @param estimations  The estimations from sensors.
   * @param ground_truth The ground truth.
   *
   * @return The RMSE.
   */
  Eigen::VectorXd CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth);
} /* namespace Sdce */
} /* namespace Tools */

#endif /* SDCE_TOOLS_H_ */
