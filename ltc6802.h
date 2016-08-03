 /*
  * linux/platform_data/ltc6802.h
  *
  * Copyright (C) 2016 Olivier C. Larocque <olivier.c.larocque@gmail.com>
  *
  * Driver for LTC6802-2, LTC6803-2 and LTC6803-4.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  */

#ifndef __LINUX_PLATFORM_DATA_LTC6802_H__
#define __LINUX_PLATFORM_DATA_LTC6802_H__

/**
* struct ltc6802_platform_data - Platform data for the LTC6802 IIO driver
* @device_address: Address of serial interface set with pins A3..A0
*/
struct ltc6802_platform_data {
	unsigned int device_address;
};

#endif
