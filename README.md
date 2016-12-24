# Serial To MIDI - MIDI To Serial
A command line utility to convert serial MIDI signals to an alsa_seq port and back.

Should support all MIDI events, but some are untested.

Supports the following baud rates: 50, 75, 110, 134.5, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 19200, 38400, 115200

##Usage

	serial-to-midi [OPTION...] device baudrate
	Example: serial-to-midi -n serial /dev/ttyACM0 115200

## Help

	serial-to-midi --help

## Compile
The program is based on asoundlib.

	gcc -std=c99 -o serial-to-midi serial-to-midi.c -lasound

## Contribute

Just fork, open issues and open pull requests to help improving this utility. Every contribution is welcome.
