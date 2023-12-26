# Vehicles Detection Using a Magnetic Sensor

Copyright (C) 2023 Lana Popova.

## References

[Video demo][1].

## Objective

The primary objective of this project is to design and implement a vehicle detection sensor for the purpose of automating the activation and deactivation of Halloween and Christmas lights near my house. The project scope will be limited to the development of a sensor prototype that can detect vehicles passing on the designated lane where the sensor is installed, while disregarding vehicles on adjacent lanes.

## Overview

The prototype comprises the [NUCLEO-F072RB][2] and [MMC5603NJ-B][3] development boards. The latter integrates the [MMC5603NJ][4] sensor, a 3-axis Anisotropic Magneto-Resistive (AMR) sensor featuring an integrated signal processing circuit and a digital communication interface (I2C). With a measurement range of +/-30 Gauss (G), a 0.0625mG resolution, and a 2mG total RMS noise level, this sensor accurately measures magnetic field strength. Capable of detecting both the Earth's magnetic field and disruptions caused by passing vehicles containing substantial amount of ferrous materials, it plays a crucial role in the prototype's functionality.

The application designed for the NUCLEO board performs the following tasks:

- Acquires magnetic field vector components from the sensor, using them to compute the magnitude of the magnetic field.
- Identifies deviations in magnitude from a standard value, which allows inferring that there is a vehicle in close proximity to the sensor.
- Communicates this information to the user through a serial terminal (UART) and an LED.

## The Application

The "main.c" file contains the application code, which is built upon the standard initialization and HAL code generated by the STM32 Cube MX software. The application uses a straightforward state machine, initializing the sensor, executing measurements, and implementing an algorithm to detect the presence of a nearby vehicle, as detailed in the "app" function. The I2C and UART interfaces, which facilitate communication with the sensor and information transmission over the serial interface respectively, operate in a blocking mode. This streamlined the application's architecture, maintaining an approximately 70 Hz update rate — sufficient for detecting vehicles on the road near my house with a 35 mph speed limit. The application incorporates GPIO interrupt functionality to detect a user button press, enabling or disabling diagnostic output on the serial terminal. This output includes the system tick and the calculated magnetic field magnitude, which was crucial for fine-tuning the vehicle detection algorithm during the development process. The example output is presented in the figure below. The algorithm implementation itself can be found in the "detect_vehicle" function.

![example_measurement.png][5]

## The MMC5603NJ driver

The "mmc5603nj" .c and .h files contain the implementation of the sensor's driver. This driver facilitates the reading and writing of one or more registers to/from the sensor (internal functions) while also providing external functions for sensor initialization and magnetic field measurements. The automatic set/reset functionality is activated to improve measurement accuracy. Notably, the driver omits certain features, aligning with the prototype's proof-of-concept nature rather than a long-term solution requiring such capabilities:

- Configuration customization for specific applications.
- Magnetic field offset calibration in response to temperature variations.
- Sensor reset functionality.

[1]: TBD
[2]: https://www.st.com/en/evaluation-tools/nucleo-f072rb.html
[3]: https://www.digikey.com/en/products/detail/memsic-inc/MMC5603NJ-B/10452797
[4]: https://www.digikey.com/en/products/detail/memsic-inc/MMC5603NJ/10452796
[5]: https://github.com/LanaPopova/cs50_project/blob/144b5ef51799dd05c3b5ae371c17734e4b7b0f30/example_measurement.png
