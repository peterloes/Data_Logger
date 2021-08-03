# Next generation of Data Logger for varied range support of Sensors.

A standalone Platform to get environmental impact around Organisms.(Animals)
Enable long-term environmental observations of different selectable sensors (I2C Interface)
and two pulse counters for e.g. Lightbarriers pulses.

Firmware working process not finished. Firmware not reviewed.

Hardware Design supported:

- Sensirion: Humidity and Temperature Sensor SHT3x-DIS
- Sensirion: CO₂, Humidity and Temperature Sensor SCD30
- Vishay: Fully Integrated Proximity and Ambient Light Sensor with Infrared Emitter VCNL4040
- Pulse Counter     	 

![My image](https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/1_Electronic_board_top.jpg)

Board Dimensions: 80mm x 60mm

Hyperterminal Screens:

Shows SHT31X-D (0x45) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_3.jpg

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_4.jpg

In operation: 3.2V@1.3mA

Shows Vishay VCNL4040 (0x60) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_5.jpg

Shows Vishay VCNL4040 (0x60) and Sensirion SCD30 CO2 (0x61)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_6.jpg

CONFIG.TXT:

https://github.com/peterloes/Data_Logger/blob/main/Software/CONFIG.TXT

Schematics:

https://github.com/peterloes/Data_Logger/blob/main/Schematics/Logger.sch.pdf


Hardware: 

Board Power:
 - Lithium Ionen Akkupack 17V@300mA or
 - Coincell CR-2032        3V@225mAh 

Sensors can powered by VBATT_1(3.3V), VBATT_2(3V):

 - Sensirion: Humidity and Temperature Sensor SHT3x-DIS:        2.2v@200µA

 - Sensirion: CO₂, Humidity and Temperature Sensor SCD30:       3.3V@ max.75mA

 - Vishay: Proximity Sensor and Ambient Light Sensor VCNL 4040: 2.5V@300µA

   Similar Software(SensorMon.c): https://github.com/peterloes/HRD_SENSOR
   
Pushbuttons:

 - PA4_SIGNAL_POWER:   On/Off Device

 - PA3_KEY_START_STOP: Collect Data

 - MCU_RESET

   Similar Software: https://github.com/peterloes/HRD_SENSOR

2x LED's:

 - PA2_POWER_LED

 - PA5_LOG_FLUSH_LED: Log data to SD Card

LEUART:

   Output to Hyperterminal

 - PD5_LEUART_RX

 - PD4_LEUART_TX

USART0/ USART1:

   Set Sensor1/Sensor2/PulsCounter1/PulsCounter2

   Set Date/Time/Interval

 - PE10_USO_TX#0

 - PE11_US0_RX#0

 - PC0_US1_TX

 - PC1_US1_RX

   Similar Software(Scales.c): https://github.com/peterloes/Scales

Sensors On/Off:

 - PE9_SENSOR1_MOSFET

 - PC9_SENSOR2_MOSFET

First Sensor Connector:

 - PA0_SENSOR_I2C_SDA

 - PE12_SENSOR1_ADDR

 - PE13_SENSOR1_ALERT

 - PA1_SENSOR_I2C_SCL

 - VDD_SENSOR1

 - PE14_SENSOR1_RESET

 - PE15_SENSOR1_RESERVE

 - GND

Second Sensor Connector:

 - PD6_SENSOR_I2C_SDA

 - PC10_SENSOR2_ADDR

 - PC11_SENSOR2_ALERT

 - PD7_SENSOR_I2C_SCL

 - VDD_SENSOR2

 - PC13_SENSOR2_RESET

 - PC14_SENSOR2_RESERVE

 - GND

Pulse Counters:

 - PD2_PULS1_COUNTER

 - PD3_PULS2_COUNTER
 
   Input Voltage = 0V...60V


The standalone Logger features EFM32 ...the world´s most energy friendly microcontrollers

ARM Cortex-M3 EFM32G230F128
