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

/**< Enable or disable Serial prints. For debugging purposes define -D DEBUG=1 in platformio.ini or create macro here */
#define DEBUG 1

#define REFERENCE 1

#if REFERENCE == 1
  #define UPLOAD_PORT 2
#else
  #define UPLOAD_PORT 1
#endif

#define SPI_CS_PIN 10

/**< SPS30 cleaning interval in days. */
#define SPS30_CLEAN_INTERVAL_IN_DAYS 7

/**< Default delay between measurements start */
#define DEFAULT_MEASUREMENT_DELAY 0 // 60

/**< Max delay between measurements start */
#define MAX_MEASUREMENT_DELAY 900

/**< Whatever or not to stop delay start */
#define MEASUREMENT_STOP true

/**< Default time for data transmission interval in seconds. */
#define TX_TIMER_SECONDS 60//3600 ; 540

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
