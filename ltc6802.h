#ifndef __LINUX_PLATFORM_DATA_LTC6802_H__
#define __LINUX_PLATFORM_DATA_LTC6802_H__

/**
* struct ltc6802_platform_data - Platform data for the LTC6802 driver
* @device_address: Address of serial interface set with pins A3..A1
*/
struct ltc6802_platform_data {
	unsigned int device_address;
};

#endif