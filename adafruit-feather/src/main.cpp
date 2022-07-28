#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <sps30.h>
#include <Adafruit_HTU21DF.h>
#include <Wire.h>

#include "config.hpp"

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = {0x96, 0x1B, 0x52, 0x19, 0x4F, 0xF8, 0x76, 0x03, 0x22, 0xE8, 0x24, 0x32, 0xF1, 0xAB, 0x25, 0xD6};

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = {0x2B, 0x4E, 0x49, 0x27, 0x54, 0x04, 0xC4, 0x77, 0xC1, 0x81, 0x07, 0x24, 0xD9, 0xAB, 0xED, 0xAA};

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0x260B0F26; // <-- Change this address for every node!

// Payload array, contains data to be transmitted over to TTN network.
static uint8_t payload[PAYLOAD_BUFFER_SIZE];
static osjob_t sendjob;
static Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
uint32_t tx_interval = TX_TIMER_SECONDS;
uint8_t sps30_clean_interval_days = SPS30_CLEAN_INTERVAL_IN_DAYS;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
};

/**
 * @brief Converts floating data into f2sflt16 format and saves the result into payload as a 2 bytes at specific position.
 *
 * @param data Data to convert and save.
 * @param payload Pointer to array of data.
 * @param position Position at which data is stored. Each data takes 2 bytes in payload array due to conversion.
 */
void saveToPayload(float data, uint8_t *payload, int position);

/**
 * @brief Converts sps30 struct into f2sflt16 format and saves the result into payload as a 2 bytes at specific position.
 *
 * @param data Data to convert and save.
 * @param payload Pointer to array of data.
 * @param position Position at which data is stored. Each atribute in data struct takes 2 bytes in payload array due to conversion.
 */
void saveToPayload(sps30_measurement &data, uint8_t *payload, int position);

/**
 * @brief Function handles data collection and transmission.
 *
 * @param j
 */
void do_send(osjob_t *j);

/**
 * @brief Function handles events.
 *
 * @param ev Event to handle
 */
void onEvent(ev_t ev);

/**
 * @brief Callback function for downlink
 *
 * @param pUserData Argument passed to LMIC_registerRxMessageCb()
 * @param port Port number
 * @param pMessage Pointer to first byte of the message
 * @param nMessage Number of bytes in message
 */
void rx_callback(void *pUserData, u1_t port, const u1_t *pMessage, size_t nMessage);

/**
 * @brief Callback function for handling LMIC events
 *
 * @param pUserData User data
 * @param ev Event type
 */
void event_callback(void *pUserData, ev_t ev);

/**
 * @brief Setup function contains necessarily things to setup MCU, modules and sensors
 *
 */
void setup()
{
#if DEBUG == 1
    while (!Serial)
        ; // wait for Serial to be initialized
    DBG_SERIAL_BEGIN(SERIAL_SPEED);
    delay(3000); // per sample code on RF_95 test
    DBG_PRINTLN("Waited 3 s");
#endif

    sensirion_i2c_init();

    while (sps30_probe() != 0)
    {
        delay(500);
    }
    
    //By default cleaning interval is set to 168 hours = 1 week
    //If sensor is switched off then counter is reset to 0
    sps30_set_fan_auto_cleaning_interval_days(sps30_clean_interval_days);

    if (!htu.begin())
    {
        DBG_PRINTLN(F("Failed to initialize htu"));
        abort();
    }

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession(0x13, DEVADDR, nwkskey, appskey);
#else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession(0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    LMIC.rxDelay = 1;// zkouška
    LMIC.rx1DrOffset = 0;// zkouška

    // Set data rate and transmit power for uplink
    //LMIC_setDrTxpow(DR_SF7, 14); // default
    LMIC_setDrTxpow(DR_SF9, 14); // email
    LMIC_setAdrMode(0); // email

    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // email -> Should not be needed since LMIC version v3.1.0
    LMIC_registerEventCb(event_callback, NULL);
    LMIC_registerRxMessageCb(rx_callback, NULL);
    //   Start job
    do_send(&sendjob);
}

/**
 * @brief Main loop function, keep clear from any heavy tasks. os_runloop_once() needs to be called often otherwise LMIC would not work properly.
 *
 */
void loop()
{
    os_runloop_once();
}

void saveToPayload(float data, uint8_t *payload, int position)
{
    uint16_t payloadTmp = LMIC_f2sflt16(data / 100);
    byte low = lowByte(payloadTmp);
    byte high = highByte(payloadTmp);
    payload[position] = low;
    payload[position + 1] = high;
}

void saveToPayload(sps30_measurement &data, uint8_t *payload, int position)
{
    saveToPayload(data.mc_1p0, payload, position);
    saveToPayload(data.mc_2p5, payload, position + 2);
    saveToPayload(data.mc_4p0, payload, position + 4);
    saveToPayload(data.mc_10p0, payload, position + 6);
    saveToPayload(data.nc_0p5, payload, position + 8);
    saveToPayload(data.nc_1p0, payload, position + 10);
    saveToPayload(data.nc_2p5, payload, position + 12);
    saveToPayload(data.nc_4p0, payload, position + 14);
    saveToPayload(data.nc_10p0, payload, position + 16);
    saveToPayload(data.typical_particle_size, payload, position + 18);
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        DBG_PRINTLN(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        struct sps30_measurement m;
        uint16_t data_ready;
        int16_t ret;
        float temp = NAN;
        float hum = NAN;

        temp = htu.readTemperature();
        // DBG_PRINT(F("Temp: "));
        // DBG_PRINTLN(temp);

        hum = htu.readHumidity();
        // DBG_PRINT(F("Humidity: "));
        // DBG_PRINTLN(hum);

        sps30_start_measurement();
        do
        {
            ret = sps30_read_data_ready(&data_ready);
            if (ret < 0)
            {
                DBG_PRINT(F("SPS30 measure error"));
                DBG_PRINTLN(ret);
            }
            else if (data_ready)
                break;
            delay(100);
        } while (1);

        ret = sps30_read_measurement(&m);
        sps30_stop_measurement();
        /*
                DBG_PRINT(F("PM  1.0: "));
                DBG_PRINTLN(m.mc_1p0);
                DBG_PRINT(F("PM  2.5: "));
                DBG_PRINTLN(m.mc_2p5);
                DBG_PRINT(F("PM  4.0: "));
                DBG_PRINTLN(m.mc_4p0);
                DBG_PRINT(F("PM 10.0: "));
                DBG_PRINTLN(m.mc_10p0);

                DBG_PRINT(F("NC  0.5: "));
                DBG_PRINTLN(m.nc_0p5);
                DBG_PRINT(F("NC  1.0: "));
                DBG_PRINTLN(m.nc_1p0);
                DBG_PRINT(F("NC  2.5: "));
                DBG_PRINTLN(m.nc_2p5);
                DBG_PRINT(F("NC  4.0: "));
                DBG_PRINTLN(m.nc_4p0);
                DBG_PRINT(F("NC 10.0: "));
                DBG_PRINTLN(m.nc_10p0);

                DBG_PRINT("Typical partical size: ");
                DBG_PRINTLN(m.typical_particle_size);*/

        saveToPayload(temp, payload, 0); // Save data to payload at [0] and [1]
        saveToPayload(hum, payload, 2);  // Save data to payload at [2] and [3]
        saveToPayload(m, payload, 4);    // Save data to payload at [4] and to [23]

        // Prepare upstream data transmission at the next possible time.
        DBG_PRINTLN(sizeof(payload));
        LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
        DBG_PRINTLN(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void rx_callback(void *pUserData, u1_t port, const u1_t *pMessage, size_t nMessage)
{
    if (port != 0 && nMessage != 0)
    {
        DBG_PRINTLN("Received downlink:");
        DBG_PRINT(F("port: "));
        DBG_PRINTLN(port);
        DBG_PRINT(nMessage);
        DBG_PRINTLN(F(" bytes of payload"));

        if (port == 1) // Port 1 -> Set sleep time
        {
            int32_t total = 0;
            for (size_t i = 0; i < nMessage; i++)
            {
                int32_t tmp = pMessage[i];
                total = total << 8;
                total = total | tmp;
                DBG_PRINT_HEX(pMessage[i]);
            }
            DBG_PRINT(F("Converted value: "));
            DBG_PRINTLN(total);
            DBG_PRINT_HEX(total);
            if (total >= MINIMUM_ALLOWED_TX_TIMER_IN_SECONDS && total <= MAXIMUM_ALLOWED_TX_TIMER_IN_SECONDS)
            {
                tx_interval = total;
            }
        }
        else if (port == 2) // Port 2 -> Set sps30 clean interval in days
        {
            int8_t total = 0;
            for (size_t i = 0; i < nMessage; i++)
            {
                int8_t tmp = pMessage[i];
                total = total << 8;
                total = total | tmp;
                DBG_PRINT_HEX(pMessage[i]);
            }
            DBG_PRINT(F("Converted value: "));
            DBG_PRINTLN(total);
            DBG_PRINT_HEX(total);
            if(total >= 1)
            {
                sps30_clean_interval_days = total;
                sps30_set_fan_auto_cleaning_interval_days(sps30_clean_interval_days);
            }
        }
    }
}

void event_callback(void *pUserData, ev_t ev)
{
    switch (ev)
    {
    case EV_SCAN_TIMEOUT:
        DBG_PRINTLN(F("EV_SCAN_TIMEOUT"));
        break;
    case EV_BEACON_FOUND:
        DBG_PRINTLN(F("EV_BEACON_FOUND"));
        break;
    case EV_BEACON_MISSED:
        DBG_PRINTLN(F("EV_BEACON_MISSED"));
        break;
    case EV_BEACON_TRACKED:
        DBG_PRINTLN(F("EV_BEACON_TRACKED"));
        break;
    case EV_JOINING:
        DBG_PRINTLN(F("EV_JOINING"));
        break;
    case EV_JOINED:
        DBG_PRINTLN(F("EV_JOINED"));
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     DBG_PRINTLN(F("EV_RFU1"));
    ||     break;
    */
    case EV_JOIN_FAILED:
        // DBG_PRINTLN(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        // DBG_PRINTLN(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        DBG_PRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(tx_interval), do_send);
        break;
    case EV_LOST_TSYNC:
        // DBG_PRINTLN(F("EV_LOST_TSYNC"));
        break;
    case EV_RESET:
        // DBG_PRINTLN(F("EV_RESET"));
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        // DBG_PRINTLN(F("EV_RXCOMPLETE"));
        break;
    case EV_LINK_DEAD:
        // DBG_PRINTLN(F("EV_LINK_DEAD"));
        break;
    case EV_LINK_ALIVE:
        DBG_PRINTLN(F("EV_LINK_ALIVE"));
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    DBG_PRINTLN(F("EV_SCAN_FOUND"));
    ||    break;
    */
    case EV_TXSTART:
        DBG_PRINTLN(F("EV_TXSTART"));
        break;
    case EV_TXCANCELED:
        DBG_PRINTLN(F("EV_TXCANCELED"));
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        // DBG_PRINTLN(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
        break;
    default:
        // DBG_PRINT(F("Unknown event: "));
        DBG_PRINTLN((unsigned)ev);
        break;
    }
}