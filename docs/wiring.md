# Wiring

The project uses two separate I2C buses on the XIAO ESP32-S3.

## Grove Vision AI V2

| Signal | XIAO ESP32-S3 |
| --- | --- |
| SDA | GPIO 5 |
| SCL | GPIO 6 |
| GND | GND |
| Power | Connect according to the module requirements |

## PCA9685 Servo Driver

| Signal | XIAO ESP32-S3 |
| --- | --- |
| SDA | GPIO 9 |
| SCL | GPIO 7 |
| GND | GND |
| Logic power | Connect according to the PCA9685 board requirements |

## Servos

| Servo | PCA9685 Channel |
| --- | --- |
| Pan | 0 |
| Tilt | 1 |

The servos are powered from an external 5 V supply.

**Important:** Connect the external power supply ground to the XIAO ground so all parts of the system share the same reference.

Do not power both servos from the XIAO 3.3 V pin.
