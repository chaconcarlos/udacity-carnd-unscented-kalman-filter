#ifndef UKF_H
#define UKF_H

/* INCLUDES ******************************************************************/

#include "MeasurementPackage.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

/* CLASS DECLARATION *********************************************************/

namespace Sdce
{

/**
 * @brief Implementation of the unscented Kalman filter.
 */
class UnscentedKalmanFilter
{
  public:

    /**
     * @brief Initializes an instance of the UnscentedKalmanFilter class.
     */
    UnscentedKalmanFilter();

    /**
     * @brief Finalizes an instance of the UnscentedKalmanFilter class.
     */
    virtual ~UnscentedKalmanFilter();

  public:

    /**
     * @brief Processes a sensor measurement.
     *
     * @param package The latest measurement data of either radar or laser.
     */
    void processMeasurement(const MeasurementPackage& package);

  private:

    /**
     * @brief
     */
    void initialize(const MeasurementPackage& package);

    /**
     * @brief Generates the sigma points based on the time difference delta t.
     *
     * @param deltaT The time difference.
     */
    void generatePredictedSigmaPoints(double dt);

    /**
     * @brief Generates the sigma points weights.
     */
    void generateWeights();

    /**
     * @brief Prediction Predicts sigma points, the state, and the state covariance matrix.
     *
     * @param deltaT Time between k and k+1 in s
     */
    void predict(double deltaT);

    /**
     * @brief Updates the state and the state covariance matrix using a laser measurement.
     *
     * @param package The measurement at k+1
     */
    void updateFromLidar(const MeasurementPackage& package);

    /**
     * @brief Updates the state and the state covariance matrix using a radar measurement.
     *
     * @param package The measurement at k+1.
     */
    void updateFromRadar(const MeasurementPackage& package);

public:

  bool            m_isInitialized;
  long long       previousTimestamp;
  Eigen::MatrixXd m_matrixP;
  Eigen::MatrixXd m_predictedSigmaPoints;
  Eigen::MatrixXd m_lidarMatrixR;
  Eigen::MatrixXd m_radarMatrixR;
  Eigen::VectorXd m_stateVector;
  Eigen::VectorXd m_sigmaPointsWeights;
};

} /* namespace Sdce */
#endif /* UKF_H */
