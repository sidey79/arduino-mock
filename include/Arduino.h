/**
 * Arduino mock header
 */
#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include "Serial.h"
#include "Wire.h"

#include <string>
#include <cstring> //for strlen()
#include <string.h> //for memcmp()


#define PROGMEM
 
#define F(expr) expr
#define PROGMEM
#define PGM_P const char *
#define PGM_VOID_P const void *
#define PSTR(s) (s)
 
#define memccpy_P      memccpy
#define memcmp_P       memcmp
#define memcpy_P       memcpy
#define memmem_P       memmem
#define printf_P       printf
#define snprintf_P     snprintf
#define sprintf_P      sprintf
#define strcasecmp_P   strcasecmp
#define strcat_P       strcat
#define strcmp_P       strcmp
#define strcpy_P       strcpy
#define strlen_P       strlen
#define strncasecmp_P  strncasecmp
#define strncat_P      strncat
#define strncmp_P      strncmp
#define strncpy_P      strncpy
#define strnlen_P      strnlen
#define strstr_P       strstr
#define vsnprintf_P    vsnprintf
 
template <typename T>
uint8_t pgm_read_byte(const T * pointer)
{
  const uint8_t * buffer = reinterpret_cast<const uint8_t *>(pointer);
  return buffer[0];
}
 
template <typename T>
uint16_t pgm_read_word(const T * pointer)
{
  const uint16_t * buffer = reinterpret_cast<const uint16_t *>(pointer);
  return buffer[0];
}
 
template <typename T>
uint32_t pgm_read_dword(const T * pointer)
{
  const uint32_t * buffer = reinterpret_cast<const uint32_t *>(pointer);
  return buffer[0];
}
 
template <typename T>
float pgm_read_float(const T * pointer)
{
  const float * buffer = reinterpret_cast<const float *>(pointer);
  return buffer[0];
}
 
//All *_near and *_far functions provides the same functionnality as their counterpart.
#define pgm_read_byte_near(expr)  pgm_read_byte(expr)
#define pgm_read_word_near(expr)  pgm_read_word(expr)
#define pgm_read_dword_near(expr) pgm_read_dword(expr)
#define pgm_read_float_near(expr) pgm_read_float(expr)
#define pgm_read_ptr_near(expr)   pgm_read_ptr(expr)
#define pgm_read_byte_far(expr)   pgm_read_byte(expr)
#define pgm_read_word_far(expr)   pgm_read_word(expr)
#define pgm_read_dword_far(expr)  pgm_read_dword(expr)
#define pgm_read_float_far(expr)  pgm_read_float(expr)
#define pgm_read_ptr_far(expr)    pgm_read_ptr(expr)

#ifdef __cplusplus
extern "C" {
#endif

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#ifdef WIN32
#elif linux
#else
#define true 0x1
#define false 0x0
#endif

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define NUM_DIGITAL_PINS            20
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 6) ? (p) + 14 : -1)

#define digitalPinHasPWM(p)         ((p) == 9 || (p) == 10 || (p) == 11)


typedef uint8_t boolean;
typedef uint8_t byte;

void init(void);

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

#ifdef __cplusplus
} // extern "C"
#endif

#include <gmock/gmock.h>

#define UNUSED(expr) do { (void)(expr); } while (0)
#define F(x) (x)

class ArduinoMock {
  public:
    MOCK_METHOD2(pinMode, void (uint8_t, uint8_t));
    MOCK_METHOD2(analogWrite, void (uint8_t, int));
    MOCK_METHOD2(digitalWrite, void (uint8_t, uint8_t));
    MOCK_METHOD1(digitalRead, int (int));
    MOCK_METHOD1(analogRead, int (int));
    MOCK_METHOD1(delay, void (int));
    MOCK_METHOD0(millis, unsigned long ());
};
ArduinoMock* arduinoMockInstance();
void releaseArduinoMock();

#endif // ARDUINO_H
