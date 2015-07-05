/*
 Serial To Midi
 Copyright (C) 2015 Valentin Pratz

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#define _POSIX_C_SOURCE	199309L /* Needed to get sleep working */
#include <stdio.h>   /* Standard input/output definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <signal.h>
#include <alsa/asoundlib.h>
#include <argp.h> /* Argument parsing */
#include <time.h> /* Sleep */

// argp arguments
const char *argp_program_version = "SerialToMidi 0.1";
const char *argp_program_bug_address = "<vp@vocaword.org>";
// structure to communicate with argopt
struct arguments {
  char *args[2]; /* terminal name, baud rate */
  char *alsa_seq_name; /* name used to identify alsa_seq device */
  int dump; /* Dump MIDI events */
};
// options
static struct argp_option options[] = {
  {"alsa-seq-name", 'n', "seq-name", 0, "Name of the alsa_seq device"},
  {"dump", 'd', 0, 0, "Dump MIDI events to stdout"},
  {"supported-baudrates", 'b', 0, 0, "List supported baudrates"},
  {0}
};
// supported baudrates
const char supported_baudrates[] = "Supported baudrates:\n"
  "50, 75, 110, 134.5, 150, 200, 300, 600, 1200, 1800, 2400\n"
  "4800, 9600, 19200, 38400, 115200\n";
// parser
static error_t parse_opt (int key, char *arg, struct argp_state *state) {
  struct arguments *arguments = state->input;

  switch (key) {
  case 'n':
    arguments->alsa_seq_name = arg;
    break;
  case 'd':
    arguments->dump = 1;
    break;
  case 'b':
    printf(supported_baudrates);
    exit(1);
  case ARGP_KEY_ARG:
    if (state->arg_num >= 2)
      {
	argp_usage(state);
      }
    arguments->args[state->arg_num] = arg;
    break;
  case ARGP_KEY_END:
    if (state->arg_num < 2)
      {
	argp_usage (state);
      }
    break;
  default:
    return ARGP_ERR_UNKNOWN;
  }
  return 0;
}
// doc
static char args_doc[] = "device baudrate";
static char doc[] =
  "SerialToMidi -- Simply convert serial signals into MIDI signals.";
// argp struct
static struct argp argp = {options, parse_opt, args_doc, doc};

// status bytes (byte 7-3)
static const unsigned char NOTE_OFF = 0b1000;
static const unsigned char NOTE_ON = 0b1001;
static const unsigned char POLYPHONIC_AFTERTOUCH = 0b1010;
static const unsigned char CONTROL_CHANGE = 0b1011;
static const unsigned char PROGRAM_CHANGE = 0b1100;
static const unsigned char CHANNEL_PRESSURE = 0b1101;
static const unsigned char PITCH_BEND_CHANGE = 0b1110;
static const unsigned char SYSTEM_COMMON = 0b1111;

// System common messages (bytes 3-0)
static const unsigned char SYSTEM_EXCLUSIVE = 0b0000;
static const unsigned char TIME_CODE_QUARTER_FRAME = 0b0001;
static const unsigned char SONG_POSITION_POINTER = 0b0010;
static const unsigned char SONG_SELECT = 0b0011;
static const unsigned char UNDEFINED_1 = 0b0100;
static const unsigned char UNDEFINED_2 = 0b0101;
static const unsigned char TUNE_REQUEST = 0b0110;
static const unsigned char END_OF_EXCLUSIVE = 0b0111;

// System realtime messages (bytes 3-0)
static const unsigned char TIMING_CLOCK = 0b1000;
static const unsigned char UNDEFINED_3 = 0b1001;
static const unsigned char START = 0b1010;
static const unsigned char CONTINUE = 0b1011;
static const unsigned char STOP = 0b1100;
static const unsigned char UNDEFINED_4 = 0b1101;
static const unsigned char ACTIVE_SENSING = 0b1110;
static const unsigned char RESET = 0b1111;

int baudrate;
int port_out_id;
snd_seq_t *seq_handle;
snd_seq_event_t ev;

unsigned char status_byte = 0;
unsigned char message_type = 0;
unsigned char ch; // channel
int data_byte_count = 0;
int data_byte_num = 0;
unsigned char data_bytes[100];
char event_name[100];

int send, in_sysex, dump; //boolean

int open_serial_port(char port[], int baudrate) {
  int fd;
  struct termios options;

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY, O_NONBLOCK);
  if (fd == -1) {
    printf("[serial-to-midi] open_serial_port: Unable to open serial port %s\n", port);
  }
  else {
    fcntl(fd, F_SETFL, O_RDWR);
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP
			 | INLCR | IGNCR | ICRNL | IXON);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_cflag &= ~(CSIZE | PARENB);
    options.c_cflag |= CS8;
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);
  }

  return (fd);
}

snd_seq_t *open_seq(char alsa_seq_name[]) {
  snd_seq_t *seq_handle;

  if (snd_seq_open(&seq_handle, "default", SND_SEQ_OPEN_DUPLEX, 0) < 0) {
    fprintf(stderr, "[serial-to-midi] Error opening ALSA sequencer.\n");
    exit(1);
  }
  snd_seq_set_client_name(seq_handle, alsa_seq_name);
  if ((port_out_id = snd_seq_create_simple_port(seq_handle, alsa_seq_name,
						SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
						SND_SEQ_PORT_TYPE_APPLICATION)) < 0) {
    fprintf(stderr, "[serial-to-midi] Error creating sequencer port: %s.\n", alsa_seq_name);
    exit(1);
  }
  return(seq_handle);
}

int dump_event(char event[], int channel, unsigned char data[], int data_len) {
  if (dump) {
    printf("%23s %3d ", event, channel);
    for (int i=0;i<data_len;i++) {
      printf("%#x ", data[i]);
    }
    printf("\n");
  }
}

void send_event() {
  snd_seq_ev_set_subs(&ev);
  snd_seq_ev_set_source(&ev, port_out_id);
  snd_seq_ev_set_direct(&ev);
  snd_seq_event_output(seq_handle, &ev);
  snd_seq_drain_output(seq_handle);
  snd_seq_ev_clear(&ev);
}

void compute_byte(unsigned char d) {
  if (d >> 7) {
    // status byte
    status_byte = d;
    message_type = d >> 4;
    ch = d & 0b1111;
    if (!in_sysex) {
      data_byte_count = 0;
    }
    if (message_type == NOTE_OFF) {
      data_byte_count = 2;
    }
    else if (message_type == NOTE_ON) {
      data_byte_count = 2;
    }
    else if (message_type == POLYPHONIC_AFTERTOUCH) {
      data_byte_count = 2;
    }
    else if (message_type == CONTROL_CHANGE) {
      data_byte_count = 2;
    }
    else if (message_type == PROGRAM_CHANGE) {
      data_byte_count = 1;
    }
    else if (message_type == CHANNEL_PRESSURE) {
      data_byte_count = 1;
    }
    else if (message_type == PITCH_BEND_CHANGE) {
      data_byte_count = 2;
    }
    else if (message_type == SYSTEM_COMMON) {
      // System Common Messages
      if (ch == SYSTEM_EXCLUSIVE) {
	data_bytes[0] = d;
	data_byte_num = 1;
	in_sysex = 1;
	data_byte_count = sizeof(data_bytes);
      }
      else if (ch == TIME_CODE_QUARTER_FRAME) {
	data_byte_count = 1;
      }
      else if (ch == SONG_POSITION_POINTER) {
	data_byte_count = 2;
      }
      else if (ch == SONG_SELECT) {
	data_byte_count = 1;
      }
      else if (ch == UNDEFINED_1 || ch == UNDEFINED_2) {
	printf("[serial-to-midi] Warning: Received undefined system common message: %d\n", ch);
      }
      else if (ch == TUNE_REQUEST) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_TUNE_REQUEST;
	send_event();
	dump_event("Tune Request", ch, data_bytes, 0);
      }
      else if (ch == END_OF_EXCLUSIVE) {
	data_bytes[data_byte_num] = d;
	in_sysex = 0;
	data_byte_count = 0;
	snd_seq_ev_set_sysex(&ev, data_byte_num + 1, data_bytes);
	send_event();
	dump_event("System Exclusive", 0, data_bytes, 0);
      }

      // System Real-Time messages
      else if (ch == TIMING_CLOCK) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_CLOCK;
	send_event();
	dump_event("Timing Clock", 0, data_bytes, 0);
      }
      else if (ch == UNDEFINED_3 || ch == UNDEFINED_4) {
	printf("[serial-to-midi] Warning: Received undefined system common message: %d\n", ch);
      }
      else if (ch == START) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_START;
	send_event();
	dump_event("Start", 0, data_bytes, 0);
      }
      else if (ch == CONTINUE) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_CONTINUE;
	send_event();
	dump_event("Continue", 0, data_bytes, 0);
      }
      else if (ch == STOP) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_STOP;
	send_event();
	dump_event("Stop", 0, data_bytes, 0);
      }
      else if (ch == ACTIVE_SENSING) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_SENSING;
	send_event();
	dump_event("Active Sensing", 0, data_bytes, 0);
      }
      else if (ch == RESET) {
	snd_seq_ev_set_fixed(&ev);
	ev.type = SND_SEQ_EVENT_RESET;
	send_event();
	dump_event("Reset", 0, data_bytes, 0);
      }
    }
    else {
      printf("[serial-to-midi] Unknown status byte: %d", d >> 4);
    }
    data_byte_num = 0;
  }
  else {
    // data byte
    if (data_byte_count == 0) {
      return;
    }
    if (data_byte_num >= sizeof(data_bytes)) {
      data_byte_num = 0;
      printf("[serial-to-midi] Warning: data byte num exceeded maximum\nReset to 0\n");
    }
    data_bytes[data_byte_num] = d;
    data_byte_num++;

    if (data_byte_num >= data_byte_count && message_type > 0 && !in_sysex) {
      // send message
      data_byte_num = 0;
      send = 1;

      // status bytes
      if (message_type == NOTE_OFF) {
	snd_seq_ev_set_noteoff(&ev, ch, data_bytes[0], data_bytes[1]);
	strncpy(event_name, "Note Off", 100);
      }
      else if (message_type == NOTE_ON) {
	if (data_bytes[1] == 0) {
	  snd_seq_ev_set_noteoff(&ev, ch, data_bytes[0], data_bytes[1]);
	  strncpy(event_name, "Note Off", 100);
	}
	else {
	  snd_seq_ev_set_noteon(&ev, ch, data_bytes[0], data_bytes[1]);
	  strncpy(event_name, "Note On", 100);
	}
      }
      else if (message_type == POLYPHONIC_AFTERTOUCH) {
	snd_seq_ev_set_keypress(&ev, ch, data_bytes[0], data_bytes[1]);
	strncpy(event_name, "Polyphonic Aftertouch", 100);
      }
      else if (message_type == CONTROL_CHANGE) {
        snd_seq_ev_set_controller(&ev, ch, data_bytes[0], data_bytes[1]);
	strncpy(event_name, "Control Change", 100);
      }
      else if (message_type == PROGRAM_CHANGE) {
	snd_seq_ev_set_pgmchange(&ev, ch, data_bytes[0]);
        strncpy(event_name, "Program Change", 100);
      }
      else if (message_type == CHANNEL_PRESSURE) {
        snd_seq_ev_set_chanpress(&ev, ch, data_bytes[0]);
	strncpy(event_name, "Channel Pressure", 100);
      }
      else if (message_type == PITCH_BEND_CHANGE) {
	snd_seq_ev_set_pitchbend(&ev, ch, (int) (((int)data_bytes[1] & 0x7f) << 7 | ((int)data_bytes[0] & 0x7f)) - 0x2000);
	strncpy(event_name, "Pitch Bend Change", 100);
      }
      else if (message_type == SYSTEM_COMMON) {
	// system common messages
	if (ch == TIME_CODE_QUARTER_FRAME) {
	  snd_seq_ev_set_fixed(&ev);
	  ev.type = SND_SEQ_EVENT_QFRAME;
	  ev.data.control.value = data_bytes[0];
	  strncpy(event_name, "TC Quarter Frame", 100);
	}
	else if (ch == SONG_POSITION_POINTER) {
	  snd_seq_ev_set_fixed(&ev);
	  ev.type = SND_SEQ_EVENT_SONGPOS;
	  ev.data.control.value = (data_bytes[1] << 7) | data_bytes[0];
	  strncpy(event_name, "Song Pos Pointer", 100);
	}
	else if (ch == SONG_SELECT) {
	  snd_seq_ev_set_fixed(&ev);
	  ev.type = SND_SEQ_EVENT_SONGSEL;
	  ev.data.control.value = data_bytes[0];
	  strncpy(event_name, "Song Select", 100);
	}
	else {
	  send = 0;
	}
      }
      else {
	send = 0;
      }
      if (send) {
	send_event();
	dump_event(event_name, ch, data_bytes, data_byte_count);
      }
    }
  }
}

int main(int argc, char **argv) {
  // argp
  struct arguments arguments;
  arguments.args[0] = "/dev/ttyACM0";
  arguments.args[1] = "9600";
  arguments.alsa_seq_name = "MidiToSerial";
  arguments.dump = 0;
  argp_parse(&argp, argc, argv, 0, 0, &arguments);
  dump = arguments.dump;
  if (strcmp(arguments.args[1], "50") == 0)
    baudrate = B50;
  else if (strcmp(arguments.args[1], "75") == 0)
    baudrate = B75;
  else if (strcmp(arguments.args[1], "110") == 0)
    baudrate = B110;
  else if (strcmp(arguments.args[1], "134.5") == 0)
    baudrate = B134;
  else if (strcmp(arguments.args[1], "150") == 0)
    baudrate = B150;
  else if (strcmp(arguments.args[1], "200") == 0)
    baudrate = B200;
  else if (strcmp(arguments.args[1], "300") == 0)
    baudrate = B300;
  else if (strcmp(arguments.args[1], "600") == 0)
    baudrate = B600;
  else if (strcmp(arguments.args[1], "1200") == 0)
    baudrate = B1200;
  else if (strcmp(arguments.args[1], "1800") == 0)
    baudrate = B1800;
  else if (strcmp(arguments.args[1], "2400") == 0)
    baudrate = B2400;
  else if (strcmp(arguments.args[1], "4800") == 0)
    baudrate = B4800;
  else if (strcmp(arguments.args[1], "9600") == 0)
    baudrate = B9600;
  else if (strcmp(arguments.args[1], "19200") == 0)
    baudrate = B19200;
  else if (strcmp(arguments.args[1], "38400") == 0)
    baudrate = B38400;
  else if (strcmp(arguments.args[1], "115200") == 0)
    baudrate = B115200;
  else {
    printf("[serial-to-midi] Error: Unsupported baud rate: %s\n", arguments.args[1]);
    exit(1);
  }

  unsigned char buffer[4];
  int available;

  struct timespec ts;
  ts.tv_sec=0;
  ts.tv_nsec = 500;

  int serial_fd = open_serial_port(arguments.args[0], baudrate);
  if (serial_fd < 0) {
    exit(1);
  }
  seq_handle = open_seq(arguments.alsa_seq_name);
  while(1) {
    available = read(serial_fd, &buffer, sizeof(buffer));
    for (int i=0;i<available;i++) {
      compute_byte(buffer[i]);
    }
    if (available == 0) {
      if (access(arguments.args[0], R_OK) == -1) {
	// device removed
	printf("[serial-to-midi] Error: Device removed -> exit\n");
	// send all notes off
	snd_seq_ev_set_controller(&ev, ch, 0x7B, 0);
	if (snd_seq_close(seq_handle) < 0)
	  printf("Error: Couldn't close alsa_seq port\n");
	exit(1);
      }
    }
    nanosleep(&ts, NULL);
  }
}
