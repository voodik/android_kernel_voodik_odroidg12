#ifndef __BOARD_ODROID_H
#define __BOARD_ODROID_H

#include <linux/types.h>

#define BOARD_ODROIDN2			0x09221000
#define BOARD_ODROIDC4			0x09051000

extern bool odroid_amlogic_usb(void);
extern u32 odroid_model(void);

#define board_is_odroidn2()		(odroid_model() == BOARD_ODROIDN2)
#define board_is_odroidc4()		(odroid_model() == BOARD_ODROIDC4)

#endif
