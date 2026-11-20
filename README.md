# 4607_Final_Project

## Project Goal
The goal of this project is to design and implement a sound-reactive speaker that connects to a laptop over bluetooth to play audio files, changing the behavior of an LCD matrix based on the audio file played. As a popular kind of product on the market right now, we believe that it would be very beneficial to our learning to see how this speaker works at a low level, along with learning about communication protocols, audio analysis, multiprocessing, and the manipulation of inputs and outputs to control a device.

## Components
![Block Diagram of Project Components](component_layout.png)

## Current Layout
### Test Scripts
Currently, we are testing the components of the speaker individually
- *Speaker_Test* - Plays tones from each speaker to ensure that they can be controlled by the Pico individually and together. (**Working**)
- *LCD_Test* - Use the buttons to display Mode and Volume on the LCD screen connected to an I2C breakout board. (**Working**)
- *Bluetooth_Test* - connect to the pico via bluetooth and play a audio file from the speakers. (**Working**)
- *LED_Matrix_Test* - flash a pattern on the LED matrix (**Working**)
- *Web_Server_Test* - Turn off and on the onboard LED via a python webapp. (**In progress**)

### Demo Script (In Progress)
- Currently combines all components sans the Web App
    - Both the web app and avrcp for song title display need worked on.
### Final Script (TODO)
- This will be the adapted from the demo script, cleaned and organized for the final product.
