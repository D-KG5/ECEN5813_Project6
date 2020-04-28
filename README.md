# ECEN5813_Project6
PES Project 6 for ECEN 5813. By Dhruva Koley and Sagar Eligar

# Contents
- ECEN5813_Project6_DK_SE
  - source
	- main.c: main source file
	- global_defines.h: header file that contains multiple flags to enable/disable logging, testing and macros
	- led_control.c: source file to initialize and control the LED
	- led_control.h: header file for led_control.c
	- logger.c: source file that contains logging utility functions
	- logger.h: header file for logger.c
	- circ_buffer.c: source file that contains circular buffer functions
	- circ_buffer.h: header file for circ_buffer.c
	- dma.c: source file that contains DMA functions
	- dma.h: header file for dma.c
	- lookup.c: source file that contains ADC, DAC, lookup functions
	- lookup.h: header file for lookup.c
	- FreeRTOSConfig.h: header file for FreeRTOS config

  - uCUnit
	- System.c: source file that contains KL25Z specific system changes to allow uCUnit to work
	- System.h: header file for System.h
	- uCUnit.h: header file containing unit test functions

- project6_progran1
  - source
	- main.c: main source file
	- global_defines.h: header file that contains multiple flags to enable/disable logging, testing and macros
	- led_control.c: source file to initialize and control the LED
	- led_control.h: header file for led_control.c
	- logger.c: source file that contains logging utility functions
	- logger.h: header file for logger.c
	- circ_buffer.c: source file that contains circular buffer functions
	- circ_buffer.h: header file for circ_buffer.c
	- dma.c: source file that contains DMA functions
	- dma.h: header file for dma.c
	- dac.c: source file that contains DAC functions
	- dac.h: header file for dac.c
	- FreeRTOSConfig.h: header file for FreeRTOS config
  - dacregister_value.PNG: screenshot of dac register value plot
  - voltage_dac.PNG: screenshot of voltage dac value plot

# Notes
- math processing is handled by another task that is notified by the handler task
- the first entry of the first run will be 0 because of how the ADC works.

# Environment
 - Windows 10
 - MCUXpresso v11.1.0
