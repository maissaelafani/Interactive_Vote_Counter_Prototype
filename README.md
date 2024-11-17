## Project Description
This project implements a smart voting box that uses a custom capacitive sensor to detect the insertion of voting letters. 
Each time a letter is inserted, the sensor detects it and sends a signal to increment the counter displayed on an OLED screen. 
The system is designed to provide real-time feedback, ensuring accurate and transparent vote tracking.

## Components Used
1. STM32 Microcontroller: Core processor for handling signals and updating the display.
2. Custom capacitive Sensor: Detects the presence of letters entering the voting box.
3. OLED Screen (SSD1306): Displays the vote count in an intuitive manner.
4. Push Button: Resets or starts specific functions of the system.

## Dependencies
1. STM32 HAL Library
2. SSD1306 OLED Driver Library

