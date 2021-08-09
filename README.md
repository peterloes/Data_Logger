# Next generation of Data Logger for varied range support of Sensors.

A standalone Platform to get environmental impact around Organisms.(Animals)
Enable long-term environmental observations of different selectable sensors (I2C Interfaces)
and two pulse counters for e.g. Lightbarriers pulses.

Firmware not reviewed.

Hardware Design (first draft) supported:

- Sensirion: Humidity and Temperature Sensor SHT3x-DIS
- Sensirion: CO₂, Humidity and Temperature Sensor SCD30
- Vishay: Fully Integrated Proximity and Ambient Light Sensor with Infrared Emitter VCNL4040
- Pulse Counter     	 

![My image](https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/1_Electronic_board_top.jpg)

Board Dimensions: 80mm x 60mm

Raw data on SD Card:

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/4_rawdata_BOX0999.TXT

Configuration data on SD Card:

https://github.com/peterloes/Data_Logger/blob/main/Software/CONFIG.TXT

Schematics:

https://github.com/peterloes/Data_Logger/blob/main/Schematics/Logger.sch.pdf

Hyperterminal Screens:

Shows SHT31X-D (0x45) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_3.jpg

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_4.jpg

In operation: 3.2V@1.3mA

Shows Vishay VCNL4040 (0x60) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_5.jpg

Shows Vishay VCNL4040 (0x60) and Sensirion SCD30 CO2 (0x61)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_6.jpg


Board Power:
 - Lithium Ionen Akkupack  17V@300mA(maximum) e.g. 12,8V 7,5Ah 96Wh 
 - Coincell CR-2032        3V@225mAh 

Sensors can powered by VBATT_1(3.3V), VBATT_2(3V):

 - Sensirion: Humidity and Temperature Sensor SHT3x-DIS:        3.3V@600µA

 - Sensirion: CO₂, Humidity and Temperature Sensor SCD30:       3.3V@ max.75mA

 - Vishay: Proximity Sensor and Ambient Light Sensor VCNL 4040: 2.5V@300µA

   Similar Software(SensorMon.c): https://github.com/peterloes/HRD_SENSOR

Optional components:

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL

https://github.com/peterloes/Booter_RFID-MS_MOMO_TAMDL


Outlook:

The Data Logger can feature two arrays with different sensors over 25 meters. (sensor-array x 2)
