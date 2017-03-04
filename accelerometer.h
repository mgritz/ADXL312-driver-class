/* @file	accelerometer.h
 *
 * @brief	Accelerometer interface class, definitions
 *
 * @date 	12.12.2014
 * @autor 	Martin Gritzan
 */
#include <iostream>
#include <unistd.h>
//#include "Eigen/Dense"
//#include "Eigen/LU"
//using namespace Eigen;

#include "ADXL312_defines.h"
#include "i2cFunctions.h"

#define ACCEL_ERROR_IFACE_FAIL 0	//!< failed to open device
#define ACCEL_ERROR_COM_FAIL 1		//!< target doesn't answer
#define ACCEL_ERROR_WRITE_FAIL 2	//!< tried to write to register w/o permission

#define ACCEL_RETRY_INTERVAL 1	//!< time between two gyro value requests in [ms]
#define ACCEL_LOWPASS_LENGTH 10	//!< number of samples to average readouts over

/** \brief RPY struct to be passed to stability calculation routine. */
struct orientation {
	float roll, pitch, yaw;
};

/** \brief Struct used to pass raw accelerometer readings
 * inside AccelSensor class. */
struct shortTriple{
	short x,y,z;
};

/** \brief Object thrown by AccelSensor to handle errors.
 *
 * This class contains an error number:
 * 0 = Cannot open interface. 1 = Device doesn't answer.
 * 2 (not yet implemented) = Memory address not writeable not yet implemented.
 */
class AccelError{
public:
	AccelError(int newErrorNumber){errorNumber = newErrorNumber;};
	~AccelError(){};
	int errorNumber;
private:
	/** forbidden */
	AccelError();
};

/** \brief Class is used to communicate with gyrosensors over i2c.
 *
 * This class is intedet to communicate with a standard-issue six-axis gyro.
 * In this version, only the ADXL312 is supported.
 */
class AccelSensor{
public:
	/** Constructor for standard initialization of connection and sensor
	 *
	 * @param i2cConnection String containing the device file name, e.g. "/dec/i2c-1".
	 * @param sensorAddress The sensor's i2c address as provided in the datasheet.
	 * @param runSelfTest Run the selft-test during initialization.
	 * @throws AccelError {In case somethin goes very wrong, the constructor will
	 * throw an error.}
	 *
	 */
	AccelSensor(	const char* i2cConnection,
				const char sensorAddress,
				const bool runSelfTest = false);
	/** Destructor, closes the connection and releases the object again */
	~AccelSensor();

	/** Used for detecting the robot orientation relative to gravity.
	 * @param debugOn Debug messages on.
	 * @returns the measured RPY angles from robot's frame to world frame in [rad]
	 * */
	struct orientation getRobotOrientation(const bool debugOn = false);

	/** Performs self-test routine for of IMU. DO NOT USE YET, needs rework.
	 * @param debugOn Debug messages on.
	 * @returns true if sensor passed self test.
	 */
	bool performSelfTest(const bool debugOn = false);

	/** Performs calibration of the IMU and writes values to OFFSET
	 * registers. DO NOT USE YET, needs rework.
	 * @param debugOn Debug messages on.
	 * @returns true if everything worked.
	 */
	bool performCalibration(const bool debugOn = false);


	/** Returns raw sensor data snapshots.
	 * @param debugOn Debug messages on.
	 * @returns raw xyz acceleration components. */
	struct shortTriple getRawAccelValues(const bool debugOn = false);

private:
	/** forbidden */
	AccelSensor(){};

	/** waiting actively for the data-ready-interrupt.
	 * @param debugOn Debug messages on.
	 * @returns false if sensor did not provide data, true otherwise*/
	bool waitForData(const bool debugOn = false);

	/** Initializes the sensor for given mode.
	 * @param debugOn Debug messages on. */
	void init(const bool debugOn = false);

	int i2cConnectionHandle; //!< Handle for the i2c connection. @see i2cfunctions.h
};
