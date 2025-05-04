.. _ms56xx-sample:

MS56XX (MS5607/MS5611) Pressure Sensor Sample
#############################################

Overview
********

This sample demonstrates how to use the MS56XX driver with MS5607 and MS5611
pressure sensors. It reads temperature and pressure values and outputs
them to the console. The sample shows usage of all three compatible strings:

* ``meas,ms56xx`` - Generic compatible that requires a ``chip`` property
* ``meas,ms5607`` - MS5607 specific compatible 
* ``meas,ms5611`` - MS5611 specific compatible

Building and Running
*******************

This sample can be built for a nucleo_f446re board as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/ms56xx
   :board: nucleo_f446re
   :goals: build flash
   :compact:

Sample Output
************

.. code-block:: console

   Found device ms5611@77
   Temperature: 22.500000 °C | Pressure: 101.325000 kPa
   Temperature: 22.500000 °C | Pressure: 101.325000 kPa
   Temperature: 22.500000 °C | Pressure: 101.324000 kPa

Troubleshooting
**************

If you don't see sensor output, check the following:

1. Verify sensor is correctly connected to the I2C bus
2. Confirm the I2C address matches your sensor (typically 0x76 or 0x77)
3. Check that your board's I2C pins are correctly configured in the overlay file