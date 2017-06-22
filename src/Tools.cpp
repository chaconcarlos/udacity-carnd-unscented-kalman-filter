/* INCLUDES ******************************************************************/

#include <iostream>

#include "Tools.h"

/* DEFINITIONS ***************************************************************/

static const int RMSE_VECTOR_SIZE        = 4;

/* USINGS ********************************************************************/

using Eigen::VectorXd;
using Eigen::MatrixXd;

/* IMPLEMENTATION ************************************************************/

namespace Sdce
{
namespace Tools
{

VectorXd
CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth)
{
  const bool hasCorrectSize = estimations.size() > 0 && estimations.size() == ground_truth.size();
  VectorXd   rmse(RMSE_VECTOR_SIZE);

  rmse.fill(0.0);

  if(hasCorrectSize == false)
  {
    std::cout << "Can't calculate RMSE. Invalid input size." << std::endl;
    return rmse;
  }

  for(unsigned int i = 0; i < estimations.size(); ++i)
  {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse    += residual;
  }

  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

} /* namespace Sdce */
} /* namespace Tools */
