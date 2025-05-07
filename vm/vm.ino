/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

// Copyright 2018 John Maloney, Bernat Romagosa, and Jens Mönig

#include "mem.h"
#include "interp.h"
#include "persist.h"
#include <arduino.h>

#if defined(WII)
    #include "ESP32Wiimote.h"
#endif
//#include "wiimote.h"

#if defined(WII)
ESP32Wiimote wiimote;


ButtonState button;
AccelState   accel ;
NunchukState nunchuk;

extern "C" {
	void loopje_wii(void) {

	wiimote.task();
	//Serial.println("loop_wii");

	if (wiimote.available() > 0) 
    {
         button  = wiimote.getButtonState();
         accel   = wiimote.getAccelState();
         nunchuk = wiimote.getNunchukState();
	}
 }
}

int wii_button(void) {
	return button;
}

int wii_nunchuk_x(void) {
	return nunchuk.xStick;
}

int wii_nunchuk_y(void) {
	return nunchuk.yStick;
}

#endif

#ifdef DUELink
	#include "stm32yyxx_ll_utils.h"
#endif

void setup() {
#ifdef ARDUINO_NRF52_PRIMO
	sd_softdevice_disable();
#endif
#ifdef DUELink
	// Workaround: start USB clock
	LL_RCC_HSI48_Enable();
	while (LL_RCC_HSI48_IsReady() != 1) {}
#endif
	//delay(5000);
	memInit();
	primsInit();
	hardwareInit();
	outputString((char *) "Welcome to MicroBlocks!");
	restoreScripts();
	if (BLE_isEnabled()) BLE_start();
	#if defined(WII)
		wiimote.init();
	#endif
   
	startAll();
	

}

void loop() {
	vmLoop();
}
