/* @file	ADXL312_defines.h
 *
 * @brief	Preprocessor macros for the ADXL312 accelerometer, directly derived
 * from Analog Devices datasheet.
 *
 * @date 	18.01.2015
 * @autor 	Martin Gritzan
 */

#define ADXL312_BUS_ADDRESS		0x53	///< bus address of the ADXL312
#define ADXL312_DEVID			0xE5	///< device ID, content of DEVID-reg
#define ADXL312_SETTLING_TIME	15		///< config settling time in [ms]

#define ADXL312_RA_DEVID		0x00	///< device ID, read-only
#define ADXL312_RA_OFSX			0x1E	///< x-axis offset
#define ADXL312_RA_OFSY			0x1F	///< y-axis offset
#define ADXL312_RA_OFSZ			0x20	///< z-axis offset
#define ADXL312_RA_THRESH_ACT	0x24	///< activity threshold
#define ADXL312_RA_THRESH_INACT	0x25	///< inactivity threshold
#define ADXL312_RA_TIME_INACT	0x26	///< inactivity time
#define ADXL312_RA_ACT_INACT_CTL	0x27	///< axis enable for activity det.
#define ADXL312_RA_BW_RATE		0x2C	///< data rate and power mode ctrl
#define ADXL312_RA_POWER_CTL	0x2D	///< power saving features control
#define ADXL312_RA_INT_ENABLE	0x2E	///< interrupt enable control
#define ADXL312_RA_INT_MAP		0x2F	///< interrupt mapping control
#define ADXL312_RA_INT_SOURCE	0x30	///< interrupt source, read-only
#define ADXL312_RA_DATA_FORMAT	0x31	///< data format control
#define ADXL312_RA_DATAX0		0x32	///< x-axis data 0(L), read-only
#define ADXL312_RA_DATAX1		0x33	///< x-axis data 1(H), read-only
#define ADXL312_RA_DATAY0		0x34	///< y-axis data 0(L), read-only
#define ADXL312_RA_DATAY1		0x35	///< y-axis data 1(H), read-only
#define ADXL312_RA_DATAZ0		0x36	///< z-axis data 0(L), read-only
#define ADXL312_RA_DATAZ1		0x37	///< z-axis data 1(H), read-only
#define ADXL312_RA_FIFO_CTL		0x38	///< FIFO control
#define ADXL312_RA_FIFO_STATUS	0x39	///< FIFO status

#define ACCEL_PRESCALER	2.9f	///< prescaler for full-res [mg/LSB]
