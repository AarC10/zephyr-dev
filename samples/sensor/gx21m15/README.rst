.. _gx21m15-sample:

GX21M15 Temperature Sensor Sample
################################

Overview
********

This sample demonstrates using the GX21M15 temperature sensor driver.
It reads the ambient temperature data from the sensor and outputs
the data to the console every second.

Requirements
************

This sample needs a GX21M15 temperature sensor connected to the target board's
I2C bus.

For the nucleo_f446re board, the sample expects the sensor to be connected
to the I2C1 bus with the device address 0x49.

Building and Running
*******************

Build the application for the nucleo_f446re board:

.. zephyr-app-commands::
   :zephyr-app: samples/sensor/gx21m15
   :board: nucleo_f446re
   :goals: build flash
   :compact:

Sample Output
************

.. code-block:: console

    Found device "gx21m15@48", getting temperature samples
    [1] Temperature: 24.50 °C
    [2] Temperature: 24.50 °C
    [3] Temperature: 24.50 °C
    ...

References
**********

- GX21M15 Datasheet
