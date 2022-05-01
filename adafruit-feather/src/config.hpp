#ifndef CONFIG_HPP
#define CONFIG_HPP

//For debugging purposes define -D DEBUG=1 in platform.io or create macro here

#define DEBUG 1 // Enable or disable Serial prints
#define SPS30_CLEAN_INTERVAL_IN_DAYS 7
#define TX_TIMER_SECONDS 60 // Default time for data transmission interval in seconds
#define PAYLOAD_BUFFER_SIZE 25 // Payload(array containing data) buffer size
#define SERIAL_SPEED 9600 // Serial communication speed
#define MINIMUM_ALLOWED_TX_TIMER_IN_SECONDS 60 // Minimum allowed interval for speed transmission -> do not set to too low, you must respect regional transmission time
#define MAXIMUM_ALLOWED_TX_TIMER_IN_SECONDS 604800 //Minimum allowed interval for speed transmission, default value is 604800 == 1 week

#if DEBUG == 1
  #define DBG_SERIAL_BEGIN(x) Serial.begin(x);
  #define DBG_PRINTF(x, y) Serial.printf(x, y);
  #define DBG_PRINT(x) Serial.print(x);
  #define DBG_PRINT_DEC(x) Serial.print(x, DEC);
  #define DBG_PRINTLN_DEC(x) Serial.print(x, DEC);
  #define DBG_PRINT_HEX(x) Serial.print(x, HEX);
  #define DBG_PRINTLN_HEX(x) Serial.print(x, HEX);
  #define DBG_PRINTLN(x) Serial.println(x);
  #define DBG_FLUSH() Serial.flush();
#else
  #define DBG_SERIAL_BEGIN(x)
  #define DBG_PRINTF(X, Y)
  #define DBG_PRINT(x)
  #define DBG_PRINT_DEC(x)
  #define DBG_PRINTLN_DEC(x)
  #define DBG_PRINT_HEX(x)
  #define DBG_PRINTLN_HEX(x)
  #define DBG_PRINTLN(x)
  #define DBG_FLUSH()
#endif

#endif