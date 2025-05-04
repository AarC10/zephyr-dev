.. _ms56xx-sample:

MS56XX: MS5607/MS5611 Pressure and Temperature Sensor Sample
############################################################

Overview
********

This sample application demonstrates how to use the MS56XX driver to read 
pressure and temperature from either MS5607 or MS5611 pressure sensors. 
The application reads the pressure and temperature at regular intervals and outputs
the results to the console.

Requirements
***********

- A board with MS5607 or MS5611 sensor support
- The board must have an I2C or SPI interface

The sample is configured to use I2C by default, but it can be easily modified
to use SPI by adjusting the overlay file.

For boards that don't already have a devicetree overlay containing MS56XX nodes,
a devicetree overlay with MS56XX nodes must be provided to build this sample.

Building and Running
*******************

For example, to build this sample for the NUCLEO-F446RE board:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ms56xx
   :board: nucleo_f446re
   :goals: build
   :compact:

Sample Output
************

.. code-block:: console

   MS56XX sensor device ready
   Sample 1:
     Temperature: 22.36 C
     Pressure: 99.12 kPa
   Sample 2:
     Temperature: 22.38 C
     Pressure: 99.13 kPa
   ...

Troubleshooting
**************

- If the sensor device is not ready, check the following:
  - Verify that the sensor is properly connected to the board
  - Check that the correct I2C or SPI bus is enabled in the devicetree
  - Ensure that the sensor address in the devicetree overlay matches the actual
    sensor address (typically 0x76 or 0x77 for I2C)
  - Make sure the "chip" property is correctly set (7 for MS5607, 11 for MS5611)

References
**********

- MS5607 datasheet: https://www.te.com/usa-en/product-CAT-BLPS0035.html
- MS5611 datasheet: https://www.te.com/usa-en/product-CAT-BLPS0036.html