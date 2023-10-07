#ifndef __arduino_h__
#define __arduino_h__

#include "mbed.h"
#ifndef M_PI
#define M_PI 3.159265358979
#endif

#include <math.h>
#include "Print.h"
#include "Stream.h"

//#include "SerialClass.h" // Arduino style Serial class

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#define PI M_PI
#define RAD_TO_DEG (180.0 / M_PI)

#define delay(ms) wait_us(ms*1000)
#define delayMicroseconds(us) wait_us(us)
#define millis() (USB::read_ms())
#define micros() (USB::read_us())

#endif

