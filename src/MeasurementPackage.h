#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

/* INCLUDES ******************************************************************/

#include "Eigen/Dense"

/* ENUM DECLARATION **********************************************************/

/**
 * @brief Represents the types of sensor.
 */
enum SensorType
{
  /**
   * @brief The LASER type of sensor.
   */
  SENSOR_TYPE_LASER,

  /**
   * @brief The RADAR type of sensor.
   */
  SENSOR_TYPE_RADAR
};

/* CLASS DECLARATION *********************************************************/

namespace Sdce
{

/**
 * @brief Represents the measurement data package from a sensor.
 */
class MeasurementPackage
{
  public:

    /**
     * @brief Initializes an instance of the MeasurementPackage class.
     */
    MeasurementPackage();

    /**
     * @brief Copy constructor implementation for the MeasurementPackage class.
     *
     * @param other The instance of the MeasurementPackage class to initialize the current instance.
     */
    MeasurementPackage(const MeasurementPackage& other);

    /**
     * @brief Finalizes an instance of the MeasurementPackage class.
     */
    ~MeasurementPackage();

  public:

    /**
     * @brief Assignment operator overload.
     *
     * @param other The MeasurementPackage to copy.
     *
     * @return A reference to this instance.
     */
    MeasurementPackage& operator=(const MeasurementPackage& other);

  public:

    long long timestamp;

    SensorType sensorType;

    Eigen::VectorXd rawMeasurements;
};

} /* namespace Sdce */

#endif /* MEASUREMENT_PACKAGE_H_ */
