/* Touchscreen library for XPT2046 Touch Controller Chip
 * Copyright (c) 2015, Paul Stoffregen, paul@pjrc.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "XPT2046_Touchscreen.h"

#define Z_THRESHOLD     400
#define Z_THRESHOLD_INT	75
#define MSEC_THRESHOLD  3
#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)

static XPT2046_Touchscreen 	*isrPinptr;
void isrPin(void);

bool XPT2046_Touchscreen::begin(SPIClass &wspi)
{
	_pspi = &wspi;
	_pspi->begin();
	pinMode(csPin, OUTPUT);
	digitalWrite(csPin, HIGH);
	if (255 != tirqPin) {
		pinMode( tirqPin, INPUT );
		attachInterrupt(digitalPinToInterrupt(tirqPin), isrPin, FALLING);
		isrPinptr = this;
	}
	return true;
}


#ifdef ESP32
void IRAM_ATTR isrPin( void )
#else
void isrPin( void )
#endif
{
	XPT2046_Touchscreen *o = isrPinptr;
	o->isrWake = true;
}

void XPT2046_Touchscreen::setSize(uint16_t _width, uint16_t _height ){
	width = _width;
	height = _height;
}

void XPT2046_Touchscreen::setCalibration(uint16_t _xmin, uint16_t _xmax, uint16_t _ymin, uint16_t _ymax ){
	xmin = _xmin;
	xmax = _xmax;
	ymin = _ymin;
	ymax = _ymax;
	calibrated = true;
}

TS_Point XPT2046_Touchscreen::getPoint()
{
	update();
	return TS_Point(xraw, yraw, zraw);
}

TS_Point XPT2046_Touchscreen::getMappedPoint()
{
	update();
	TS_Point p = getPoint();	
	p.x = ((float)(p.x-xmin)/xmax)*getWidth();//getWidth();         
	p.y = ((float)(p.y-ymin)/ymax)*getHeight();//getHeight(); 
	return TS_Point(p.x, p.y, p.z);
}

bool XPT2046_Touchscreen::tirqTouched()
{
	return (isrWake);
}

uint16_t XPT2046_Touchscreen::getWidth(){
	uint16_t rotWidth;
	switch (rotation) {
		case 0:
		case 2:
			rotWidth = width;
		break;
		case 1:
		case 3:
			rotWidth = height;
		break;
		default: 
			rotWidth = width;
	}
	return rotWidth;
}
uint16_t XPT2046_Touchscreen::getHeight(){
	uint16_t rotHeight;
	switch (rotation) {
		case 0:
		case 2:
			rotHeight = height;
		break;
		case 1:
		case 3:
			rotHeight = width;
		break;
		default: 
			rotHeight = height;
	}
	return rotHeight;
}

bool XPT2046_Touchscreen::touched()
{
	update();
	return (zraw >= Z_THRESHOLD);
}

void XPT2046_Touchscreen::readData(uint16_t *x, uint16_t *y, uint8_t *z)
{
	update();
	*x = xraw;
	*y = yraw;
	*z = zraw;
}

bool XPT2046_Touchscreen::bufferEmpty()
{
	return ((millis() - msraw) < MSEC_THRESHOLD);
}

static int16_t besttwoavg( int16_t x , int16_t y , int16_t z ) {
  int16_t da, db, dc;
  int16_t reta = 0;
  if ( x > y ) da = x - y; else da = y - x;
  if ( x > z ) db = x - z; else db = z - x;
  if ( z > y ) dc = z - y; else dc = y - z;

  if ( da <= db && da <= dc ) reta = (x + y) >> 1;
  else if ( db <= da && db <= dc ) reta = (x + z) >> 1;
  else reta = (y + z) >> 1;   //    else if ( dc <= da && dc <= db ) reta = (x + y) >> 1;

  return (reta);
}

// TODO: perhaps a future version should offer an option for more oversampling,
//       with the RANSAC algorithm https://en.wikipedia.org/wiki/RANSAC

void XPT2046_Touchscreen::update()
{
	int16_t data[6];
	int z;
	if (!isrWake) return;
	uint32_t now = millis();
	if (now - msraw < MSEC_THRESHOLD) return;
	

	_pspi->beginTransaction(SPI_SETTING);
	digitalWrite(csPin, LOW);
	_pspi->transfer(0xB1 /* Z1 */);
	int16_t z1 = _pspi->transfer16(0xC1 /* Z2 */) >> 3;
	z = z1 + 4095;
	int16_t z2 = _pspi->transfer16(0x91 /* X */) >> 3;
	z -= z2;
	if (z >= Z_THRESHOLD) {
		_pspi->transfer16(0x91 /* X */);  // dummy X measure, 1st is always noisy
		data[0] = _pspi->transfer16(0xD1 /* Y */) >> 3;
		data[1] = _pspi->transfer16(0x91 /* X */) >> 3; // make 3 x-y measurements
		data[2] = _pspi->transfer16(0xD1 /* Y */) >> 3;
		data[3] = _pspi->transfer16(0x91 /* X */) >> 3;
	}
	else data[0] = data[1] = data[2] = data[3] = 0;	// Compiler warns these values may be used unset on early exit.
	data[4] = _pspi->transfer16(0xD0 /* Y */) >> 3;	// Last Y touch power down
	data[5] = _pspi->transfer16(0) >> 3;
	digitalWrite(csPin, HIGH);
	_pspi->endTransaction();

	if (z < 0) z = 0;
	if (z < Z_THRESHOLD) { //	if ( !touched ) {
		zraw = 0;
		if (z < Z_THRESHOLD_INT) { //	if ( !touched ) {
			if (255 != tirqPin) isrWake = false;
		}
		return;
	}
	zraw = z;

	int16_t x = besttwoavg( data[0], data[2], data[4] );
	int16_t y = besttwoavg( data[1], data[3], data[5] );

	if (z >= Z_THRESHOLD && z != 4095) { // check for spurious value
		msraw = now;	// good read completed, set wait
		switch (rotation) {
		  case 0:
			xraw = y;
			yraw = x;
			break;
		  case 1:
			xraw = x;
			yraw = 4095-y;
			break;
		  case 2:
			xraw = 4095 - y;
			yraw = 4095 - x;
			break;
		  default: // 3
			xraw = 4095 - x;
			yraw = y;
		}

	}
}
