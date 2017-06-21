/* INCLUDES ******************************************************************/

#include "Eigen/Dense"
#include <iostream>
#include "UnscentedKalmanFilter.h"

/* USINGS ********************************************************************/

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/* DEFINITIONS ***************************************************************/

static const int    LIDAR_STATE_DIMENSION               = 2;
static const int    RADAR_STATE_DIMENSION               = 3;
static const int    STATE_DIMENSION                     = 5;
static const int    AUGMENTED_STATE_DIMENSION           = 7;
static const double LAMBDA                              = 3 - STATE_DIMENSION;
static const int    SIGMA_POINTS_COUNT                  = 2 * AUGMENTED_STATE_DIMENSION + 1;
static const double NOISE_STD_LONGITUDINAL_ACCELERATION = 0.2;
static const double NOISE_STD_YAW_ACCELERATION          = 0.2;
static const double NOISE_STD_LASER_SENSOR_PX           = 0.15;
static const double NOISE_STD_LASER_SENSOR_PY           = 0.15;
static const double NOISE_STD_RADAR_SENSOR_RADIUS       = 0.3;
static const double NOISE_STD_RADAR_SENSOR_PHI          = 0.03;
static const double NOISE_STD_RADAR_SENSOR_CHANGE       = 0.3;

/* STATIC FUNCTIONS **********************************************************/

/**
 * @brief Normalizes an angle between -PI and PI.
 *
 * @param angle The angle to normalize.
 *
 * @return The normalized angle.
 */
static double
getNormalizedAngle(double angle)
{
  double result = angle;

  while (result > M_PI)
    result -= 2. * M_PI;

  while (result < -M_PI)
    result += 2. * M_PI;

  return result;
}

/* CLASS IMPLEMENTATION ******************************************************/

namespace Sdce
{

/**
 * Initializes Unscented Kalman filter
 */
UnscentedKalmanFilter::UnscentedKalmanFilter()
: m_isInitialized(false)
, m_useLaser(true)
, m_useRadar(true)
, previousTimestamp(0)
{
  m_stateVector          = VectorXd(STATE_DIMENSION);
  m_matrixP              = MatrixXd::Identity(STATE_DIMENSION, STATE_DIMENSION);
  m_lidarMatrixR         = MatrixXd(LIDAR_STATE_DIMENSION, LIDAR_STATE_DIMENSION);
  m_radarMatrixR         = MatrixXd(RADAR_STATE_DIMENSION, RADAR_STATE_DIMENSION);
  m_predictedSigmaPoints = MatrixXd(STATE_DIMENSION, SIGMA_POINTS_COUNT);

  double laserMatrixRCx = NOISE_STD_LASER_SENSOR_PX * NOISE_STD_LASER_SENSOR_PX;
  double laserMatrixRCy = NOISE_STD_LASER_SENSOR_PY * NOISE_STD_LASER_SENSOR_PY;

  m_lidarMatrixR << laserMatrixRCx,              0,
                                 0, laserMatrixRCx;

  double radarMatrixRCr   = NOISE_STD_RADAR_SENSOR_RADIUS * NOISE_STD_RADAR_SENSOR_RADIUS;
  double radarMatrixRCphi = NOISE_STD_RADAR_SENSOR_PHI    * NOISE_STD_RADAR_SENSOR_PHI;
  double radarMatrixRCyaw = NOISE_STD_RADAR_SENSOR_CHANGE * NOISE_STD_RADAR_SENSOR_CHANGE;

  m_radarMatrixR << radarMatrixRCr,                0,                0,
                                 0, radarMatrixRCphi,                0,
                                 0,                0, radarMatrixRCyaw;
}

UnscentedKalmanFilter::~UnscentedKalmanFilter()
{
}

void
UnscentedKalmanFilter::generatePredictedSigmaPoints(double delta_t)
{
  MatrixXd Xsig_aug = MatrixXd(AUGMENTED_STATE_DIMENSION, SIGMA_POINTS_COUNT);
  VectorXd x_aug    = VectorXd(AUGMENTED_STATE_DIMENSION);
  MatrixXd P_aug    = MatrixXd(AUGMENTED_STATE_DIMENSION, AUGMENTED_STATE_DIMENSION);

  //create augmented mean state
  x_aug.head(STATE_DIMENSION) = m_stateVector;
  x_aug(5)                    = 0;
  x_aug(6)                    = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = m_matrixP;
  P_aug(5,5)               = NOISE_STD_LONGITUDINAL_ACCELERATION * NOISE_STD_LONGITUDINAL_ACCELERATION;
  P_aug(6,6)               = NOISE_STD_YAW_ACCELERATION * NOISE_STD_YAW_ACCELERATION;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;

  double sqrtLambdaAug = LAMBDA + AUGMENTED_STATE_DIMENSION;

  for (int i = 0; i< AUGMENTED_STATE_DIMENSION; i++)
  {
    size_t index = i + 1;
    Xsig_aug.col(index)         = x_aug + sqrt(sqrtLambdaAug) * L.col(i);
    Xsig_aug.col(index + AUGMENTED_STATE_DIMENSION) = x_aug - sqrt(sqrtLambdaAug) * L.col(i);
  }

  for (size_t i = 0; i < SIGMA_POINTS_COUNT; ++i)
  {
    //extract values for better readability
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p;
    double py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001)
    {
        double c1 = v / yawd;
        double c2 = yaw + yawd * delta_t;

        px_p = p_x + c1 * (sin (c2) - sin(yaw));
        py_p = p_y + c1 * (cos(yaw) - cos(c2));
    }
    else
    {
        double vXdelta_t = v * delta_t;
        px_p = p_x + vXdelta_t * cos(yaw);
        py_p = p_y + vXdelta_t * sin(yaw);
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd * delta_t;
    double yawd_p = yawd;

    double nuaXdelta_t = nu_a * delta_t;
    double xy_noise_c1 = 0.5 * nuaXdelta_t * delta_t;

    //add noise
    px_p = px_p + xy_noise_c1 * cos(yaw);
    py_p = py_p + xy_noise_c1 * sin(yaw);
    v_p  = v_p  + nuaXdelta_t;

    yaw_p  = yaw_p  + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    m_predictedSigmaPoints(0,i) = px_p;
    m_predictedSigmaPoints(1,i) = py_p;
    m_predictedSigmaPoints(2,i) = v_p;
    m_predictedSigmaPoints(3,i) = yaw_p;
    m_predictedSigmaPoints(4,i) = yawd_p;
  }
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UnscentedKalmanFilter::processMeasurement(const MeasurementPackage& package)
{
  if (m_isInitialized == false)
  {
    initialize(package);
    return;
  }

  float dt = (package.timestamp - previousTimestamp) / 1000000.0;

  previousTimestamp = package.timestamp;

  if (dt > 0.0001)
    predict(dt);

  if (package.sensorType == SENSOR_TYPE_RADAR)
    updateFromRadar(package);
  else if (package.sensorType == SENSOR_TYPE_LASER)
    updateFromLidar(package);
}

void
UnscentedKalmanFilter::initialize(const MeasurementPackage& package)
{
  if (package.sensorType == SENSOR_TYPE_RADAR)
  {
    const float x = package.rawMeasurements[0] * std::cos(package.rawMeasurements[1]);
    const float y = package.rawMeasurements[0] * std::sin(package.rawMeasurements[1]);

    m_stateVector << x, y, package.rawMeasurements[2], 0, 0;
  }
  else if (package.sensorType == SENSOR_TYPE_LASER)
  {
    m_stateVector << package.rawMeasurements[0], package.rawMeasurements[1], 0, 0, 0;
  }

  previousTimestamp = package.timestamp;
  m_isInitialized   = true;
}

void
UnscentedKalmanFilter::predict(double delta_t)
{
  generatePredictedSigmaPoints(delta_t);

  m_sigmaPointsWeights = VectorXd(SIGMA_POINTS_COUNT);

  VectorXd      x            = VectorXd(STATE_DIMENSION);
  MatrixXd      P            = MatrixXd(STATE_DIMENSION, STATE_DIMENSION);
  static double lambdaPn_aug = LAMBDA + AUGMENTED_STATE_DIMENSION;

  m_sigmaPointsWeights(0) = LAMBDA / (lambdaPn_aug);

  // TODO: Do this one time only.
  for (int i = 1; i< SIGMA_POINTS_COUNT; ++i)
    m_sigmaPointsWeights(i) = 0.5 / (lambdaPn_aug);

  //predicted state mean
  x.fill(0.0);

  //iterate over sigma points
  for (int i = 0; i < SIGMA_POINTS_COUNT; ++i)
    x = x + m_sigmaPointsWeights(i) * m_predictedSigmaPoints.col(i);

  //predicted state covariance matrix
  P.fill(0.0);

  //iterate over sigma points
  for (int i = 0; i < SIGMA_POINTS_COUNT; ++i)
  {
    // state difference
    VectorXd x_diff = m_predictedSigmaPoints.col(i) - m_stateVector;

    x_diff(3) = getNormalizedAngle(x_diff(3));

    P = P + m_sigmaPointsWeights(i) * x_diff * x_diff.transpose() ;
  }

  m_stateVector = x;
  m_matrixP     = P;
}

void
UnscentedKalmanFilter::updateFromLidar(const MeasurementPackage& package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MatrixXd Zsig = m_predictedSigmaPoints.block(0, 0, LIDAR_STATE_DIMENSION, SIGMA_POINTS_COUNT);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(LIDAR_STATE_DIMENSION);

  z_pred.fill(0.0);

  for (int i=0; i < SIGMA_POINTS_COUNT; i++)
  {
      z_pred = z_pred + m_sigmaPointsWeights(i) * Zsig.col(i);
  }

  MatrixXd S = MatrixXd(LIDAR_STATE_DIMENSION, LIDAR_STATE_DIMENSION);
   S.fill(0.0);

   for (int i = 0; i < SIGMA_POINTS_COUNT; i++)
   {  //2n+1 simga points
     //residual
     VectorXd z_diff = Zsig.col(i) - z_pred;

     z_diff(1) = getNormalizedAngle(z_diff(1));

     S = S + m_sigmaPointsWeights(i) * z_diff * z_diff.transpose();
   }

    S = S + m_lidarMatrixR;

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    // Parse radar measurement
    VectorXd z = VectorXd(LIDAR_STATE_DIMENSION);
    z << package.rawMeasurements[0],  package.rawMeasurements[1];

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(STATE_DIMENSION, LIDAR_STATE_DIMENSION);

    //calculate cross correlation matrix
      Tc.fill(0.0);
      for (int i = 0; i < SIGMA_POINTS_COUNT; ++i) {  //2n+1 simga points

        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        z_diff(1) = getNormalizedAngle(z_diff(1));

        // state difference
        VectorXd x_diff = m_predictedSigmaPoints.col(i) - m_stateVector;
        //angle normalization

        x_diff(3) = getNormalizedAngle(x_diff(3));

        Tc = Tc + m_sigmaPointsWeights(i) * x_diff * z_diff.transpose();
      }

      //Kalman gain K;
      MatrixXd K = Tc * S.inverse();

      //residual
      VectorXd z_diff = z - z_pred;

      //angle normalization

      z_diff(1) = getNormalizedAngle(z_diff(1));

      //update state mean and covariance matrix
      m_stateVector = m_stateVector + K * z_diff;
      m_matrixP     = m_matrixP - K*S*K.transpose();
}

void
UnscentedKalmanFilter::updateFromRadar(const MeasurementPackage& package)
{
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(RADAR_STATE_DIMENSION, SIGMA_POINTS_COUNT);

  for (int i = 0; i < SIGMA_POINTS_COUNT; i++)
  {  //2n+1 simga points

      // extract values for better readibility
      double p_x = m_predictedSigmaPoints(0,i);
      double p_y = m_predictedSigmaPoints(1,i);
      double v  = m_predictedSigmaPoints(2,i);
      double yaw = m_predictedSigmaPoints(3,i);

      double v1 = cos(yaw)*v;
      double v2 = sin(yaw)*v;

      // measurement model
      Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
      Zsig(1,i) = atan2(p_y,p_x);                                 //phi
      Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
    }

    //mean predicted measurement
    VectorXd z_pred = VectorXd(RADAR_STATE_DIMENSION);
    z_pred.fill(0.0);

    for (int i=0; i < SIGMA_POINTS_COUNT; i++)
    {
        z_pred = z_pred + m_sigmaPointsWeights(i) * Zsig.col(i);
    }

    //measurement covariance matrix S
    MatrixXd S = MatrixXd(RADAR_STATE_DIMENSION, RADAR_STATE_DIMENSION);
    S.fill(0.0);
    for (int i = 0; i < SIGMA_POINTS_COUNT; i++) {  //2n+1 simga points
      //residual
      VectorXd z_diff = Zsig.col(i) - z_pred;

      z_diff(1) = getNormalizedAngle(z_diff(1));

      S = S + m_sigmaPointsWeights(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix

    S = S + m_radarMatrixR;

    /*****************************************************************************
    *  Update
    ****************************************************************************/

    // Parse radar measurement
    VectorXd z = VectorXd(RADAR_STATE_DIMENSION);
    z << package.rawMeasurements[0],
        package.rawMeasurements[1],
        package.rawMeasurements[2];

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(STATE_DIMENSION, RADAR_STATE_DIMENSION);

    //calculate cross correlation matrix
      Tc.fill(0.0);
      for (int i = 0; i < SIGMA_POINTS_COUNT; i++)
      {
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;

        z_diff(1) = getNormalizedAngle(z_diff(1));

        // state difference
        VectorXd x_diff = m_predictedSigmaPoints.col(i) - m_stateVector;

        x_diff(3) = getNormalizedAngle(x_diff(3));

        Tc = Tc + m_sigmaPointsWeights(i) * x_diff * z_diff.transpose();
      }

      //Kalman gain K;
      MatrixXd K = Tc * S.inverse();

      //residual
      VectorXd z_diff = z - z_pred;

      z_diff(1) = getNormalizedAngle(z_diff(1));

      //update state mean and covariance matrix
      m_stateVector = m_stateVector + K * z_diff;
      m_matrixP = m_matrixP - K*S*K.transpose();
}

} /* namespace Sdce */
