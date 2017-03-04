/* @file	accelerometer.cpp
 *
 * @brief	Accelerometer interface functions, implementation
 *
 * @date 	12.12.2014
 * @autor 	Martin Gritzan
 */

#include "accelerometer.h"

#include <bitset>
#include <unistd.h>
#include <math.h>

/* Calculates the angle of the first axis, in case of a
 * static acceleration by the earth's gravitational field
 */
float angle(float centerAxis, float axis2, float axis3) {
	float root = sqrt( axis2 * axis2 + axis3 * axis3);
	return atan(centerAxis / root);
}


AccelSensor::AccelSensor(	const char* i2cConnection, const char sensorAddress,
		const bool runSelfTest){

	/* initialize device connection */
	i2cConnectionHandle =
			i2cIfaceInit(i2cConnection, sensorAddress, I2CADDRESSMODE_7BITS);

	/* check connection initialization success */
	if (i2cConnectionHandle < 0)
		throw new AccelError(ACCEL_ERROR_IFACE_FAIL);

	/* initialize Accel */
	init(false);

	/* test connection */
	char answer = 'x';
	i2cRequest(i2cConnectionHandle, ADXL312_RA_DEVID, &answer, 1);
	if (answer != ADXL312_DEVID)
		throw new AccelError(ACCEL_ERROR_COM_FAIL);

	if (runSelfTest) {
		if (performSelfTest(false)) {
			std::cout << " Self test passed " << std::endl;
		} else {
			std::cout << " Self test failed " << std::endl;
		}
	}
}

AccelSensor::~AccelSensor(){
	/* stop measurement */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_POWER_CTL, 0x00);
	/* terminate interface */
	i2cClose(i2cConnectionHandle);
}


struct orientation AccelSensor::getRobotOrientation(const bool debugOn){
		/* read Accel raw values */
	struct shortTriple rawSensorValues = getRawAccelValues(debugOn);

	/* resort values for robot frame base
	 * X: -z, Y = -y, Z = -x
	 * */
	struct shortTriple resortedValues;
	resortedValues.x = -rawSensorValues.z;
	resortedValues.y = -rawSensorValues.y;
	resortedValues.z = -rawSensorValues.x;

	if (debugOn)
		std::cout << "Cart. grav. vector in [G]:" << std::endl <<  " X = "
				<< (float) (resortedValues.x * ACCEL_PRESCALER / 1000)
				<< std::endl << " Y = "
				<< (float) (resortedValues.y * ACCEL_PRESCALER / 1000)
				<< std::endl << " Z = "
				<< (float) (resortedValues.z * ACCEL_PRESCALER / 1000)
				<< std::endl;

	/* calculate RPY-angles from kartesian representation */
	struct orientation robotRPY;
	robotRPY.pitch = angle(resortedValues.x, resortedValues.y,
			resortedValues.z);
	robotRPY.roll = angle(resortedValues.y, resortedValues.x, resortedValues.z);
	robotRPY.yaw = angle(resortedValues.z, resortedValues.x, resortedValues.y);

	if (debugOn)
		std::cout << "Robot RPY in [deg]:" << std::endl << " R = "
				<< robotRPY.roll * 180 / M_PI << std::endl << " P = "
				<< robotRPY.pitch * 180 / M_PI << std::endl << " Y = "
				<< robotRPY.yaw * 180 / M_PI << std::endl;

	return robotRPY;
}

bool AccelSensor::performSelfTest(const bool debugOn) {
	struct shortTriple measured = { 0, 0, 0 }, selfTest = { 0, 0, 0 }, temp;

	/* retreive raw data */
	for (int i = 0; i < 100; i++) {
		temp = getRawAccelValues();
		measured.x += temp.x;
		measured.y += temp.y;
		measured.z += temp.z;
	}
	measured.x /= 100;
	measured.y /= 100;
	measured.z /= 100;

	/* activate self test */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_DATA_FORMAT, 0x81);
	/* wait for accelerometer to settle */
	usleep(ADXL312_SETTLING_TIME * 1000);

	/* retreive raw data */
	for (int i = 0; i < 100; i++) {
		temp = getRawAccelValues();
		selfTest.x += temp.x;
		selfTest.y += temp.y;
		selfTest.z += temp.z;
	}
	selfTest.x /= 100;
	selfTest.y /= 100;
	selfTest.z /= 100;

	/* disable self test */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_DATA_FORMAT, 0x08);
	/* wait for accelerometer to settle */
	usleep(ADXL312_SETTLING_TIME * 1000);

	/* check self test response */
	if (selfTest.x - measured.x < 68 || selfTest.y - measured.y < -764
			|| selfTest.y - measured.y < 101 || selfTest.x - measured.x > 764
			|| selfTest.y - measured.y > -32 || selfTest.y - measured.y > 1241)
		return false;	// self test failed
	// else, self test is passed.
	return true;
}

bool AccelSensor::performCalibration(const bool debugOn){
	struct shortTriple measured = { 0, 0, 0 }, temp;

	/* retreive raw data */
	for (int i = 0; i < 100; i++) {
		temp = getRawAccelValues();
		measured.x += temp.x;
		measured.y += temp.y;
		measured.z += temp.z;
	}
	measured.x /= 100;
	measured.y /= 100;
	measured.z /= 100;

	/* calculate offsets*/
	char offsets[3] = { 0, 0, 0 };
	offsets[0] = -(short) (measured.x / 4);
	offsets[1] = -(short) (measured.y / 4);
	offsets[2] = -(short) ((measured.x - 256) / 4);

	/* write to registers */
	i2cWriteCont(i2cConnectionHandle,ADXL312_RA_OFSX,offsets);

	if(debugOn) std::cout << "IMU calibrated" << std::endl;
	return true;

}

struct shortTriple AccelSensor::getRawAccelValues(const bool debugOn){
	shortTriple result = { 0, 0, 0 };
	char burstAnswer[6] = { 0, 0, 0, 0, 0, 0 };

	if (!waitForData(debugOn))
		return result;
	i2cRequest(i2cConnectionHandle, ADXL312_RA_DATAX0, burstAnswer, 6);

	if (debugOn)
		std::cout << "Register content: X0=0b"
				<< (std::bitset<8>) (burstAnswer[0]) << ", X1=0b"
				<< (std::bitset<8>) (burstAnswer[1]) << ", Y0=0b"
				<< (std::bitset<8>) (burstAnswer[2]) << ", Y1=0b"
				<< (std::bitset<8>) (burstAnswer[3]) << ", Z0=0b"
				<< (std::bitset<8>) (burstAnswer[4]) << ", Z1=0b"
				<< (std::bitset<8>) (burstAnswer[5]) << " " << std::endl;

	/* aggregate and interpret as short - respects big-endian */
	short* aggregatedAnswer = reinterpret_cast<short*>(burstAnswer);

	/* alternatively, this is equivalent to
	short aggregatedAnswer[3] = {
			(burstAnswer[1] << 8) | burstAnswer[0],
			(burstAnswer[3] << 8) | burstAnswer[2],
			(burstAnswer[5] << 8) | burstAnswer[4]
	};
	*/


	if (debugOn)
		std::cout << "Aggregated to: " << std::endl << "X: "
				<< (std::bitset<16>) (aggregatedAnswer[0]) << " = "
				<< aggregatedAnswer[0] << " = "
				<< (float) aggregatedAnswer[0] * ACCEL_PRESCALER
						/ 1000 << "[G]" << std::endl << "Y: "
				<< (std::bitset<16>) (aggregatedAnswer[1]) << " = "
				<< aggregatedAnswer[1] << " = "
				<< (float) aggregatedAnswer[1] * ACCEL_PRESCALER
						/ 1000 << "[G]" << std::endl << "Z: "
				<< (std::bitset<16>) (aggregatedAnswer[2]) << " = "
				<< aggregatedAnswer[2] << " = "
				<< (float) aggregatedAnswer[2] * ACCEL_PRESCALER
						/ 1000 << "[G]" << std::endl;

	/* paste into return type */
	result.x = aggregatedAnswer[0];
	result.y = aggregatedAnswer[1];
	result.z = aggregatedAnswer[2];
	return result;
}

bool AccelSensor::waitForData(const bool debugOn) {
	char answer;
	/* wait for next set of Accel values */
	int i = 0;
	while (i < 10000) {
		i2cRequest(i2cConnectionHandle, ADXL312_RA_INT_SOURCE, &answer, 1);
		if ((answer & 0x80) == 0x80) {	// DRDY interrupt bit set
			if (debugOn)
				std::cout << "Data ready after " << i * ACCEL_RETRY_INTERVAL
						<< "ms" << std::endl;
			return true;
		}
		usleep(ACCEL_RETRY_INTERVAL * 1000);
		i++;
	}
	return false;

}

void AccelSensor::init(const bool debugOn){
	/* set activity detection to zero */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_ACT_INACT_CTL, 0x00);
	/* set data rate to 200Hz */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_BW_RATE, 0x0B);
	/* I2C-mode, FULL_RES + [+-12g] */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_DATA_FORMAT, 0x0B);
	/* enable data-ready interrupt */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_INT_ENABLE, 0x80);
	/* data ready interrupt output on INT1 pin */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_INT_MAP, 0x80);
	/* bypass FIFO */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_FIFO_CTL, 0x00);

	/* write calibration data */
//	char offsets[3] = {0x01, 0xFE, 0x02};
//	i2cWriteCont(i2cConnectionHandle,ADXL312_RA_OFSX,offsets);

	/* start measurement and wait for settle */
	i2cWrite(i2cConnectionHandle, ADXL312_RA_POWER_CTL, 0x08);
	usleep(ADXL312_SETTLING_TIME * 1000);

	if(debugOn) std::cout << "ADXL312 accelerometer setup complete." << std::endl;
}

