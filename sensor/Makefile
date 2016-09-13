

all: build upload listen


build:
	ino build -v -m pro328

upload:
	avrdude -p atmega328p -carduino -v -U flash:w:.build/pro328/firmware.hex:i -P /dev/ttyUSB? -C /etc/avrdude.conf -b57600

listen:
	ino serial -b 9600

clean:
	ino clean

# Program the fuses for no BOD, internal 8mhz oscillator using Arduino-based ISP
fuses:
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xda:m -U efuse:w:0xff:m 

# Load optiboot on the Atmega.  Who knows what the clones are loaded with...
bootload:
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200
	avrdude -p m328p -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:/usr/share/arduino/hardware/arduino/bootloaders/atmega/ATmegaBOOT_168_atmega328_pro_8MHz.hex -v

