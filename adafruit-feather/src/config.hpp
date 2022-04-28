#ifndef CONFIG_HPP
#define CONFIG_HPP

//For debugging purposes define -D DEBUG=1 in platform.io or create macro here

#define DEBUG 0
#define SPS30_CLEAN_INTERVAL_IN_DAYS 1
#define TX_TIMER_SECONDS 60
#define PAYLOAD_BUFFER_SIZE 25
#define SERIAL_SPEED 9600
#define MINIMUM_ALLOWED_TX_TIMER_SECONDS 60

#if DEBUG == 1
  #define DBG_SERIAL_BEGIN(x) Serial.begin(x);
  #define DBG_PRINTF(x, y) Serial.printf(x, y);
  #define DBG_PRINT(x) Serial.print(x)
  #define DBG_PRINTDEC(x) Serial.print(x, DEC)
  #define DBG_PRINTHEX(x) Serial.print(x, HEX)
  #define DBG_PRINTLN(x) Serial.println(x)
  #define DBG_FLUSH() Serial.flush();
#else
  #define DBG_SERIAL_BEGIN(x)
  #define DBG_PRINTF(X, Y)
  #define DBG_PRINT(x)
  #define DBG_PRINTDEC(x)
  #define DBG_PRINTHEX(x)
  #define DBG_PRINTLN(x)
  #define DBG_FLUSH()
#endif

#endif