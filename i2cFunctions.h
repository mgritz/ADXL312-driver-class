/**
 * i2c.h
 * \brief i2c-bus driver functions
 *
 *  Created on: 12.12.2014
 *      Author: martin
 */

#ifndef I2CFUNCTIONS_H_
#define I2CFUNCTIONS_H_

#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#define I2CADDRESSMODE_7BITS 0
#define I2CADDRESSMODE_10BITS 1

/** i2c-File handle */
static int i2cHandle = 0;

/** initializes i2c interface
 *
 * @param device string containing full path to device file, e.g. "/dev/i2c-1".
 * @param slaveAddress contains the slave device address, either 7- or 10-bit mode.
 * @param addressMode specifies the address mode to be used, macros provided.
 * @return integer values used as a file handle for the device.
 */
static int i2cIfaceInit(const char* device, const char slaveAddress,
		const char addressMode){

	/* Initialize the I2C channel */
	i2cHandle = open(device,O_RDWR);
	if (i2cHandle < 0)
	    {
	        fprintf (stderr, "Can not open device %s (errno: %s)!\n",
	                 device, strerror (errno));
	        return i2cHandle;
	    }

	if(addressMode == I2CADDRESSMODE_10BITS){
		ioctl(i2cHandle,I2C_TENBIT,slaveAddress);
	}
	else{
		ioctl(i2cHandle,I2C_SLAVE,slaveAddress);
	}
	return i2cHandle;
}

/** \brief Requests the content of a slave's specific register.
 *
 * This function requests the content of a readable register from the
 * slave device.
 *
 * @param handle the file handle. @see i2cIfaceInit().
 * @param address the register to be requested.
 * @param inputBytes string to be filled with the requested data.
 * @param length number of bytes to be answered by the slave.
 */
static int i2cRequest(const int handle, const char address,
		char* inputBytes, const int length){

	int outBytes = -1, inBytes = -1;

	outBytes = write(handle, &address, 1);
	if( outBytes < 0 ){
			fprintf (stderr, "Can not transmit address (errno: %s)!\n",
					 strerror (errno));
			return -1;
		}
//	printf("%i address bytes sent.\n", outBytes);	//debug

	inBytes = read(handle, inputBytes, length);
	if (inBytes < 0) {
		fprintf(stderr, "There is nothing to be read (errno: %s)!\n",
				strerror(errno));
		return -1;
	}
//	printf("%i register bytes received.\n", inBytes);	//debug
	return 0;
}

/** \brief Writes a line to the slave's memory..
 *
 * This function writes packets of data to the slave device.
 *
 * @param handle the file handle. @see i2cIfaceInit().
 * @param address the registers to be written to (first byte).
 * @param outputBytes string containing all bytes to write.*
 */
static int i2cWriteCont(const int handle, const char address,
		const char* outputBytes){

	int outBytes = -1;
	/* concatenate address and output bytes */
	char* outputWithAddress = (char*)malloc(sizeof(outputBytes) + 1);
	outputWithAddress[0] = address;
	memcpy(outputWithAddress + 1, outputBytes, sizeof(outputBytes));

	outBytes = write(handle, outputWithAddress, sizeof(outputWithAddress));
	if(outBytes < 0){
		fprintf (stderr, "Can not transmit %s (errno: %s)!\n",
				outputBytes, strerror (errno));
		return -1;
	}
//	printf("%i bytes sent.\n", outBytes);	//debug
	free(outputWithAddress);
	return 0;
}

/** \brief Writes do a slave's specific register.
 *
 * This function writes data to the slave device.
 *
 * @param handle the file handle. @see i2cIfaceInit().
 * @param address the register to be written to.
 * @param outputByte the data byte.
 */
static int i2cWrite(const int handle, const char address,
		const char outputByte){

	int outBytes = -1;
	/* concatenate address and output bytes */
	char* outputWithAddress = (char*)malloc(2);
	outputWithAddress[0] = address;
	outputWithAddress[1] = outputByte;

	outBytes = write(handle, outputWithAddress, 2);
	if(outBytes < 0){
		fprintf (stderr, "Can not transmit %c (errno: %s)!\n",
				outputByte, strerror (errno));
		return -1;
	}
//	printf("%i bytes sent.\n", outBytes);	//debug
	free(outputWithAddress);
	return 0;
}

/** \brief Closes the i2c interface.
 *
 * Function used to terminate all transmissions and free the device
 * file again so it can be used by other processes.
 *
 * @param handle the file handle. @see i2cIfaceInit().
 */
static int i2cClose(const int handle) {
	if (close(handle) < 0) {
		fprintf(stderr, "Can close i2c connection (errno: %s)!\n",
				strerror(errno));
		return -1;
	}
	return 0;
}
#endif /* I2CFUNCTIONS_H_ */
