## Knihovny k provozu
https://github.com/sandeepmistry/arduino-LoRa - knihovna do Arduino IDE

## Board pins
https://raw.githubusercontent.com/Xinyuan-LilyGO/TTGO-LoRa-Series/master/README.MD
| Name        | V1.0 | V1.2(T-Fox) | V1.6 | V2.0 |
| ----------- | ---- | ----------- | ---- | ---- |
| OLED RST    | 16   | N/A         | N/A  | N/A  |
| OLED SDA    | 4    | 21          | 21   | 21   |
| OLED SCL    | 15   | 22          | 22   | 22   |
| SDCard CS   | N/A  | N/A         | 13   | 13   |
| SDCard MOSI | N/A  | N/A         | 15   | 15   |
| SDCard MISO | N/A  | N/A         | 2    | 2    |
| SDCard SCLK | N/A  | N/A         | 14   | 14   |
| DS3231 SDA  | N/A  | 21          | N/A  | N/A  |
| DS3231 SCL  | N/A  | 22          | N/A  | N/A  |
| LORA MOSI   | 27   | 27          | 27   | 27   |
| LORA MISO   | 19   | 19          | 19   | 19   |
| LORA SCLK   | 5    | 5           | 5    | 5    |
| LORA CS     | 18   | 18          | 18   | 18   |
| LORA RST    | 14   | 23          | 23   | 23   |
| LORA DIO0   | 26   | 26          | 26   | 26   |

## Links
základy LoRa
https://randomnerdtutorials.com/esp32-lora-rfm95-transceiver-arduino-ide/
https://blog.laskarduino.cz/zaciname-s-iot-lora/

Novinky v The Things Network (TTN) v3
https://blog.squix.org/2021/07/ttgo-lora32-v1-0-with-ttn-v3-and-otaa.html

## Knihovny pro Arduino IDE k vyzkoušení
https://www.arduino.cc/reference/en/libraries/tinylora/
https://github.com/TheThingsNetwork/arduino-device-lib
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series

https://robotzero.one/heltec-lora32-lorawan-node/ - pro TTN v2
https://medium.com/@JoooostB/how-i-send-my-first-lorawan-message-to-the-things-network-using-a-ttgo-esp32-micropython-a3fe447fff82 - pro TTN v 2

https://github.com/cubapp/LilyGO-TTGO-LoRa32-SenderReceiver/blob/master/LoRa.ino - simple LoRa sender/receiver (withou TTN), needs 2 devices

video youtube spektrální analyzátor (na jaké frekvenci funguje anténa)
https://www.youtube.com/watch?v=CJNq2I_PDHQ

## Links
TTN payload formatter.
```
function sflt162f(rawSflt16)
    {
    // rawSflt16 is the 2-byte number decoded from wherever;
    // it's in range 0..0xFFFF
    // bit 15 is the sign bit
    // bits 14..11 are the exponent
    // bits 10..0 are the the mantissa. Unlike IEEE format,
    // the msb is explicit; this means that numbers
    // might not be normalized, but makes coding for
    // underflow easier.
    // As with IEEE format, negative zero is possible, so
    // we special-case that in hopes that JavaScript will
    // also cooperate.
    //
    // The result is a number in the open interval (-1.0, 1.0);
    //

    // throw away high bits for repeatability.
    rawSflt16 &= 0xFFFF;

    // special case minus zero:
    if (rawSflt16 == 0x8000)
        return -0.0;

    // extract the sign.
    var sSign = ((rawSflt16 & 0x8000) != 0) ? -1 : 1;

    // extract the exponent
    var exp1 = (rawSflt16 >> 11) & 0xF;

    // extract the "mantissa" (the fractional part)
    var mant1 = (rawSflt16 & 0x7FF) / 2048.0;

    // convert back to a floating point number. We hope
    // that Math.pow(2, k) is handled efficiently by
    // the JS interpreter! If this is time critical code,
    // you can replace by a suitable shift and divide.
    var f_unscaled = sSign * mant1 * Math.pow(2, exp1 - 15);

    return f_unscaled;
    }

function Decoder(bytes, port) {
  //Data example: 596FCD77C7566A66136D736D1E5D606C9C6E1D6F3A6F2C54
  var decoded = {};
  decoded.bytes = bytes;

  //Temperature and humidity
  rawTemp = bytes[0] + bytes[1] * 256;
  decoded.degreesC = sflt162f(rawTemp) * 100;

  rawHumid = bytes[2] + bytes[3] * 256;
  decoded.humidity = sflt162f(rawHumid) * 100;

  //Particle mass concontration
  pm1 = bytes[4] + bytes[5] * 256;
  decoded.pm1 = sflt162f(pm1) * 100;

  pm2 = bytes[6] + bytes[7] * 256;
  decoded.pm2 = sflt162f(pm2) * 100;

  pm4 = bytes[8] + bytes[9] * 256;
  decoded.pm4 = sflt162f(pm4) * 100;

  pm10 = bytes[10] + bytes[11] * 256;
  decoded.pm10 = sflt162f(pm10) * 100;

  //Particle number concontration
  nc05 = bytes[12] + bytes[13] * 256;
  decoded.nc05 = sflt162f(nc05) * 100;

  nc10 = bytes[14] + bytes[15] * 256;
  decoded.nc10 = sflt162f(nc10) * 100;

  nc25 = bytes[16] + bytes[17] * 256;
  decoded.nc25 = sflt162f(nc25) * 100;

  nc40 = bytes[18] + bytes[19] * 256;
  decoded.nc40 = sflt162f(nc40) * 100;

  nc10p0 = bytes[20] + bytes[21] * 256;
  decoded.nc10p0 = sflt162f(nc10p0) * 100;

  typical = bytes[22] + bytes[23] * 256;
  decoded.typical = sflt162f(typical) * 100;

  return decoded;
}
```
