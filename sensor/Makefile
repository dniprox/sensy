

all: build upload listen


build:
	ino build -m pro328 --cxxflags="-fno-exceptions -O2"

upload:
	avrdude -p atmega328p -carduino -v -U flash:w:.build/pro328/firmware.hex:i -P /dev/ttyUSB? -C /etc/avrdude.conf -b9600

listen:
	ino serial -b 9600

clean:
	ino clean

# Program the fuses for no BOD, internal 8mhz oscillator, divided by 8, using Arduino-based ISP
fuses:
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0x62:m -U hfuse:w:0xda:m -U efuse:w:0xff:m

# Load optiboot on the Atmega.  Who knows what the clones are loaded with...
bootload:
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:atmega328_1mhz.hex -v

