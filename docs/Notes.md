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
