# Platform for Animal environmental influences. Data Logger functions. 

A standalone Platform to enable long-term environmental observations
of two different selectable sensors (I2C Interface) and two puls counter for e.g. Lightbarriers pulses.

Firmware working process discontinued.

Hardware Design supported:

- Sensirion: Humidity and Temperature Sensor SHT3x-DIS
- Sensirion: CO₂, Humidity and Temperature Sensor SCD30
- Vishay: Fully Integrated Proximity and Ambient Light Sensor with Infrared Emitter VCNL4040
- Puls Counter     	 

![My image](https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/1_Electronic_board_top.jpg)


Board Dimensions: 80mm x 60mm

Board Power:
- Lithium Ionen Akkupack 17V@max.300mA
- Coincell CR-2032        3V@225mAh 

Sensors can powered by VBATT_1(3.3V), VBATT_2(3V):
Sensirion: Humidity and Temperature Sensor SHT3x-DIS:        2.2@200µA
Sensirion: CO₂, Humidity and Temperature Sensor SCD30:       3.3V@ max.75mA
Vishay: Proximity Sensor and Ambient Light Sensor VCNL 4040: 2.5V@300µA

Pushbuttons:
PA4_SIGNAL_POWER:   On/Off Device
PA3_KEY_START_STOP: Collect Data
MCU_RESET
Similar Software: https://github.com/peterloes/HRD/tree/master/Software

2x LED's:
PA2_POWER_LED
PA5_LOG_FLUSH_LED: Log data to SD Card

LEUART: Output to Hyperterminal
PD5_LEUART_RX
PD4_LEUART_TX

USART0/ USART1:
Set Sensor1/Sensor2/PulsCounter1/PulsCounter2
Set Date/Time/Intervall
PE10_USO_TX#0
PE11_US0_RX#0
PC0_US1_TX
PC1_US1_RX
Similar Software: https://github.com/peterloes/Scales

Sensors On/Off:
PE9_SENSOR1_MOSFET
PC9_SENSOR2_MOSFET

First Sensor Connector:
PA0_SENSOR_I2C_SDA
PE12_SENSOR1_ADDR
PE13_SENSOR1_ALERT
PA1_SENSOR_I2C_SCL
VDD_SENSOR1
PE14_SENSOR1_RESET
PE15_SENSOR1_RESERVE
GND
