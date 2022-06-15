# Next generation of data logger for varied range support of sensors.

A standalone platform to get environmental impact around organisms (animals).
Enable long-term observations of different selectable sensors (I2C Interfaces).
Two pulse counters are included to get lightbarrier activity.

Authors: Loës P., Kempenaers B.(2021) [![DOI](https://zenodo.org/badge/360805367.svg)](https://zenodo.org/badge/latestdoi/360805367)

## Main advantages against other data logger:

- Available sensors can be freely combined as requrired.(Firmware adaption)
- Daily regular synchronisation from date and time with DCF77 radio clock receiver.(Firmware adaption)
- Long battery life with lithium battery. (max.17V)
- SD Card includes configuration file.
- Activation times(x5) for data logging.


## Hardware Design (first draft) supported:

The following firmware is not reviewed.

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


## Hyperterminal Screens

Shows SHT31X-D (0x45) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_3.jpg

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_4.jpg

In operation with Interval (configuration.txt) : 3,2V@0,54mA

Shows Vishay VCNL4040 (0x60) and SHT31X-D (0x45)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_5.jpg

Shows Vishay VCNL4040 (0x60) and Sensirion SCD30 CO2 (0x61)

https://github.com/peterloes/Data_Logger/blob/main/Getting_Started_Tutorial/3_Hyperterminal_6.jpg


Board Power:
 - Lithium battery  17V/300mA(maximum) e.g. 12,8V 7,5Ah 96Wh 
 - LS14250 Primary Lithium-Thionyl chloride    3.6V/1200mAh 

Sensors can powered by VBATT_1(3.3V), VBATT_2(3V):

 - Sensirion: Humidity and Temperature Sensor SHT3x-DIS:        600µA@3,3V

 - Sensirion: CO₂, Humidity and Temperature Sensor SCD30:     max.75mA@3,3V 

 - Vishay: Proximity Sensor and Ambient Light Sensor VCNL 4040: 300µA@2,5V

   Similar Software(SensorMon.c): https://github.com/peterloes/HRD_SENSOR

Optional components:

https://github.com/peterloes/Clock_receiver_RFID-MS_MOMO_TAMDL

https://github.com/peterloes/Booter_RFID-MS_MOMO_TAMDL


Outlook:

The Data Logger can feature two arrays with different sensors over 25 meters. (sensor-array x 2)



C is the Greenest Programming Language

To achieve its power and energy-efficiency features, EFM32 products utilize ultralow active and idle power,
fast wakeup and processing times, and most important, the ability to intelligently interact with peripherals
and sensors autonomously without waking up the CPU and consuming more power. 
