/* INCLUDES ******************************************************************/

#include "MeasurementPackage.h"

/* CLASS IMPLEMENTATION ******************************************************/

namespace Sdce
{

MeasurementPackage::MeasurementPackage()
: timestamp(0)
, sensorType(SENSOR_TYPE_LASER)
{
}

MeasurementPackage::MeasurementPackage(const MeasurementPackage& other)
{
  timestamp       = other.timestamp;
  sensorType      = other.sensorType;
  rawMeasurements = other.rawMeasurements;
}

MeasurementPackage::~MeasurementPackage()
{
}

MeasurementPackage&
MeasurementPackage::operator=(const MeasurementPackage& other)
{
  timestamp       = other.timestamp;
  sensorType      = other.sensorType;
  rawMeasurements = other.rawMeasurements;

  return *this;
}

} /* namespace Sdce */
