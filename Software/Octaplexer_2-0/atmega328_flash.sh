#!/bin/bash
echo"Selecting external clock on octaplexer board"
sudo avrdude -p atmega328p -c usbtiny -U lfuse:w:0xEF:m 

echo"External clock should be set, not flashing program to atmega328p"

sudo avrdude -p atmega328p -c usbtiny -U flash:w:Octaplexer_2-0.ino.standard.hex:m
