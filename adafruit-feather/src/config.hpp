#ifndef CONFIG_HPP
#define CONFIG_HPP

/** \file config.hpp
 * This file holds config values such as server ip, delay values, pin values etc.
 */

//

/** @cond */
/************************************************
 *               Configuration
 ***********************************************/
/** @endcond */

/**< Enable or disable Serial prints. For debugging purposes define -D DEBUG=1 in platform.io or create macro here */
#define DEBUG 0

/**< SPS30 cleanign interval in days. */
#define SPS30_CLEAN_INTERVAL_IN_DAYS 7

/**< Default time for data transmission interval in seconds. */
#define TX_TIMER_SECONDS 60

/**< Payload(array containing data) buffer size. */
#define PAYLOAD_BUFFER_SIZE 25

/**< Serial communication speed. */
#define SERIAL_SPEED 9600

/**< Minimum allowed interval for speed transmission -> do not set to too low, you must respect regional transmission time. */
#define MINIMUM_ALLOWED_TX_TIMER_IN_SECONDS 60

/**< Minimum allowed interval for speed transmission, default value is 604800 == 1 week. */
#define MAXIMUM_ALLOWED_TX_TIMER_IN_SECONDS 604800

/** @cond */
/************************************************
 *               Debug macros
 ***********************************************/
/** @endcond */
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
