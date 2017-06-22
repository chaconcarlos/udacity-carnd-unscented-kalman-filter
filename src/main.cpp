/* INCLUDES ******************************************************************/

#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>

//#include "ukf.h"
#include "Tools.h"
#include "UnscentedKalmanFilter.h"

/* USINGS ********************************************************************/

using namespace Sdce;

using Eigen::VectorXd;
using Eigen::MatrixXd;

using json = nlohmann::json;

/* DEFINITIONS ***************************************************************/

static const int         SIMULATOR_PORT                = 4567;
static const int         STATE_VECTOR_SIZE             = 4;
static const int         PX_VECTOR_INDEX               = 0;
static const int         PY_VECTOR_INDEX               = 1;
static const int         VX_VECTOR_INDEX               = 2;
static const int         VY_VECTOR_INDEX               = 3;
static const std::string EMPTY_RESPONSE                = "42[\"manual\",{}]";
static const std::string SENSOR_TYPE_NAME_LASER        = "L";
static const std::string SENSOR_TYPE_NAME_RADAR        = "R";
static const std::string EVENT_TELEMETRY               = "telemetry";
static const std::string SENSOR_MEASUREMENT_FIELD_NAME = "sensor_measurement";

/* STATIC DECLARATIONS *******************************************************/

static uWS::Hub              webSocketHub;
static UnscentedKalmanFilter ukf;
static std::vector<VectorXd> estimations;
static std::vector<VectorXd> ground_truth;

/* PROTOTYPES ****************************************************************/

/**
 * @brief Gets the data from the SocketIO output.
 *
 * @param jsonData The data from the socket.
 *
 * @return If there is data, the JSON object in string format will be returned. Otherwise, an empty string
 * will be returned.
 */
static std::string getData(const std::string& rawData);

/**
 * @brief Parses the LASER data.
 *
 * @param stream The data stream.
 *
 * @return The measurement package with the LASER sensor data.
 */
static MeasurementPackage parseLaserData(std::istringstream& stream);

/**
 * @brief Parses the RADAR data.
 *
 * @param stream The data stream.
 *
 * @return The measurement package with the RADAR sensor data.
 */
static MeasurementPackage parseRadarData(std::istringstream& stream);

/**
 * @brief Parses the ground truth data.
 *
 * @param stream The data stream.
 *
 * @return A vector containing the ground truth data.
 */
static VectorXd parseGroundTruthData(std::istringstream& stream);

/**
 * @brief Gets the estimated state for the tracked object.
 *
 * @param measurementPackage The sensor data.
 *
 * @return A vector containing the estimated state for the tracked object.
 */
static VectorXd getEstimatedState(const MeasurementPackage& measurementPackage);

/**
 * @brief Builds the response to a telemetry calculation request.
 *
 * @param sensorMeasurement The measurement data from the sensor.
 *
 * @return The JSON response to the telemetry calculation request.
 */
static json buildTelemetryResponse(const std::string& sensorMeasurement);

/**
 * @brief Executes when the websocket is connected.
 *
 * @param webSocket The web socket.
 * @param webSocket The HTTP request.
 */
static void onConnection(uWS::WebSocket<uWS::SERVER> webSocket, uWS::HttpRequest request);

/**
 * @brief
 *
 * @param res
 * @param req
 * @param data
 * @param bytesToRead
 * @param length
 */
static void onHttpRequest(uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t bytesToRead, size_t length);

/**
 * @brief Executes when a message is received by the web socket.
 *
 * @param ws     The web socket.
 * @param data   The message data.
 * @param length The lenght of the data.
 * @param opCode The operation code.
 */
static void onMessage(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode);

/**
 * @brief Entry point for the main program.
 *
 * @return Status code of the execution.
 */
int main();

/* IMPLEMENTATION ************************************************************/

std::string
getData(const std::string& rawData)
{
  const bool  foundNull = rawData.find("null") != std::string::npos;
  const auto  b1        = rawData.find_first_of("[");
  const auto  b2        = rawData.find_first_of("]");
  std::string result;

  if (foundNull == false && b1 != std::string::npos && b2 != std::string::npos)
    result = rawData.substr(b1, b2 - b1 + 1);

  return result;
}

MeasurementPackage
parseLaserData(std::istringstream& stream)
{
  MeasurementPackage measurementPackage;
  long long          timestamp;
  float              px = 0;
  float              py = 0;

  stream >> px;
  stream >> py;
  stream >> timestamp;

  measurementPackage.sensorType       = SENSOR_TYPE_LASER;
  measurementPackage.rawMeasurements  = VectorXd(2);
  measurementPackage.rawMeasurements << px, py;
  measurementPackage.timestamp = timestamp;

  return measurementPackage;
}

MeasurementPackage
parseRadarData(std::istringstream& stream)
{
  MeasurementPackage measurementPackage;
  long long          timestamp;
  float              ro     = 0;
  float              theta  = 0;
  float              ro_dot = 0;

  stream >> ro;
  stream >> theta;
  stream >> ro_dot;
  stream >> timestamp;

  measurementPackage.sensorType       = SENSOR_TYPE_RADAR;
  measurementPackage.rawMeasurements  = VectorXd(3);
  measurementPackage.rawMeasurements << ro, theta, ro_dot;
  measurementPackage.timestamp = timestamp;

  return measurementPackage;
}

VectorXd
parseGroundTruthData(std::istringstream& stream)
{
  VectorXd values(STATE_VECTOR_SIZE);
  float    x_gt  = 0;
  float    y_gt  = 0;
  float    vx_gt = 0;
  float    vy_gt = 0;

  stream >> x_gt;
  stream >> y_gt;
  stream >> vx_gt;
  stream >> vy_gt;

  values(PX_VECTOR_INDEX) = x_gt;
  values(PY_VECTOR_INDEX) = y_gt;
  values(VX_VECTOR_INDEX) = vx_gt;
  values(VY_VECTOR_INDEX) = vy_gt;

  return values;
}

VectorXd
getEstimatedState(const MeasurementPackage& measurementPackage)
{
  VectorXd estimate(STATE_VECTOR_SIZE);

  double p_x = ukf.m_stateVector(PX_VECTOR_INDEX);
  double p_y = ukf.m_stateVector(PY_VECTOR_INDEX);
  double v   = ukf.m_stateVector(VX_VECTOR_INDEX);
  double yaw = ukf.m_stateVector(VY_VECTOR_INDEX);
  double v1  = cos(yaw) * v;
  double v2  = sin(yaw) * v;

  estimate(PX_VECTOR_INDEX) = p_x;
  estimate(PY_VECTOR_INDEX) = p_y;
  estimate(VX_VECTOR_INDEX) = v1;
  estimate(VY_VECTOR_INDEX) = v2;

  return estimate;
}

json
buildTelemetryResponse(const std::string& sensorMeasurement)
{
  long long          timestamp;
  std::string        sensorType;
  MeasurementPackage measurementPackage;
  std::istringstream iss(sensorMeasurement);

  iss >> sensorType;

  if (sensorType == SENSOR_TYPE_NAME_LASER)
    measurementPackage = parseLaserData(iss);
  else if (sensorType == SENSOR_TYPE_NAME_RADAR)
    measurementPackage = parseRadarData(iss);

  ground_truth.push_back(parseGroundTruthData(iss));

  ukf.processMeasurement(measurementPackage);

  VectorXd estimate = getEstimatedState(measurementPackage);

  estimations.push_back(estimate);

  VectorXd RMSE = Sdce::Tools::CalculateRMSE(estimations, ground_truth);
  json     msgJson;

  msgJson["estimate_x"] = estimate(PX_VECTOR_INDEX);
  msgJson["estimate_y"] = estimate(PY_VECTOR_INDEX);
  msgJson["rmse_x"]     = RMSE(PX_VECTOR_INDEX);
  msgJson["rmse_y"]     = RMSE(PY_VECTOR_INDEX);
  msgJson["rmse_vx"]    = RMSE(VX_VECTOR_INDEX);
  msgJson["rmse_vy"]    = RMSE(VY_VECTOR_INDEX);

  return msgJson;
}

void
onConnection(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)
{
  std::cout << "Connected!!!" << std::endl;
}

void
onDisconnection(uWS::WebSocket<uWS::SERVER> webSocket, int, char*, size_t)
{
  webSocket.close();
  std::cout << "Disconnected" << std::endl;
}

void
onHttpRequest(uWS::HttpResponse* httpResponse, uWS::HttpRequest request, char* data, size_t bytesToRead, size_t length)
{
  const std::string response = "<h1>Hello world!</h1>";

  if (request.getUrl().valueLength == 1)
    httpResponse->end(response.data(), response.length());
  else
    httpResponse->end(nullptr, 0);
}

void
onMessage(uWS::WebSocket<uWS::SERVER> webSocket, char *data, size_t length, uWS::OpCode opCode)
{
  if (length && length > 2 && data[0] == '4' && data[1] == '2')
  {
    const std::string rawData = getData(std::string(data));

    if (rawData.empty())
    {
     webSocket.send(EMPTY_RESPONSE.data(), EMPTY_RESPONSE.length(), uWS::OpCode::TEXT);
     return;
    }

    auto        j     = json::parse(rawData);
    std::string event = j[0].get<std::string>();

    if (event == EVENT_TELEMETRY)
    {
      json msgJson = buildTelemetryResponse(j[1][SENSOR_MEASUREMENT_FIELD_NAME]);
      auto msg     = "42[\"estimate_marker\"," + msgJson.dump()   + "]";
      webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
}

int
main()
{
  webSocketHub.onMessage(&onMessage);
  webSocketHub.onHttpRequest(&onHttpRequest);
  webSocketHub.onConnection(&onConnection);
  webSocketHub.onDisconnection(&onDisconnection);

  if (webSocketHub.listen(SIMULATOR_PORT))
  {
    std::cout << "Listening to port " << SIMULATOR_PORT << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  webSocketHub.run();
}
