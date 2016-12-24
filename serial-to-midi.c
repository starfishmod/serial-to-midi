/*
 Serial To Midi
 Copyright (C) 2016 Valentin Pratz

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
#include <alloca.h>
#include <argp.h> /* Argument parsing */
#include <time.h> /* Sleep */

// argp arguments
const char *argp_program_version = "SerialToMidi 0.2";
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
  {"dump", 'd', 0, 0, "Dump MIDI events from input to stdout"},
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

int baudrate;
int port_out_id;
snd_seq_t *seq_handle;
snd_midi_event_t *parser;
snd_seq_event_t ev;

int data_byte_count = 0;
int data_byte_num = 0;
unsigned char data_bytes[1000];

int send, in_sysex, dump; //boolean

int open_serial_port(char port[], int baudrate) {
  int fd;
  struct termios options;

  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY, O_NONBLOCK);
  if (fd == -1) {
    printf("[serial-to-midi] open_serial_port: Unable to open serial port %s\n", port);
  }
  else {
    fcntl(fd, F_SETFL, O_RDWR | O_NONBLOCK);
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
  if ((port_out_id = snd_seq_create_simple_port(seq_handle, "midi out",
						SND_SEQ_PORT_CAP_READ|SND_SEQ_PORT_CAP_SUBS_READ,
						SND_SEQ_PORT_TYPE_APPLICATION)) < 0) {
    fprintf(stderr, "[serial-to-midi] Error creating sequencer output port: %s.\n", alsa_seq_name);
    exit(1);
  }
  if ((snd_seq_create_simple_port(seq_handle, "midi in",
                                  SND_SEQ_PORT_CAP_WRITE|SND_SEQ_PORT_CAP_SUBS_WRITE,
                                  SND_SEQ_PORT_TYPE_APPLICATION)) < 0) {
    fprintf(stderr, "[serial-to-midi] Error creating sequencer input port: %s.\n", alsa_seq_name);
    exit(1);
  }
  return(seq_handle);
}

void send_event() {
  snd_seq_ev_set_subs(&ev);
  snd_seq_ev_set_source(&ev, port_out_id);
  snd_seq_ev_set_direct(&ev);
  snd_seq_event_output(seq_handle, &ev);
  snd_seq_drain_output(seq_handle);
  snd_seq_ev_clear(&ev);
}

void compute_byte(unsigned char d, snd_midi_event_t *parser) {
  int num = snd_midi_event_encode_byte(parser, d, &ev);
  if (num < 0) {
    fprintf(stderr, "Error computing byte\n");
  } else if (num == 1) {
    send_event();
  }
}

void compute_midi_event(snd_seq_t *seq_handle, snd_midi_event_t *parser, int serial_fd) {
  snd_seq_event_t *m_ev;
  unsigned char out_buffer[1000];
  int size;
  do {
    snd_seq_event_input(seq_handle, &m_ev);
    size = (int) snd_midi_event_decode(parser, out_buffer, 1000, m_ev);
    for (int i=0; i<size; i++) {
      printf("%#x", out_buffer[i]);
      if (write(serial_fd, out_buffer, (size_t) size) != size) {
        fprintf(stderr, "Error sending event\n");
      }
    }
    printf("\n");
    if (dump) {
      switch (m_ev->type) {
        case SND_SEQ_EVENT_CONTROLLER:
          printf("Control event on Channel %2d: %5d\n",
                 m_ev->data.control.channel, m_ev->data.control.value);
          break;
        case SND_SEQ_EVENT_PITCHBEND:
          printf("Pitchbender event on Channel %2d: %5d\n",
                 m_ev->data.control.channel, m_ev->data.control.value);
          break;
        case SND_SEQ_EVENT_NOTEON:
          printf("Note On event on Channel %2d: %5d\n",
                 m_ev->data.control.channel, m_ev->data.note.note);
          break;
        case SND_SEQ_EVENT_NOTEOFF:
          printf("Note Off event on Channel %2d: %5d\n",
                 m_ev->data.control.channel, m_ev->data.note.note);
          break;
        default:
          printf("Computed unknown event\n");
          break;
      }
    }
    snd_seq_free_event(m_ev);
  } while (snd_seq_event_input_pending(seq_handle, 0) > 0);
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
  if (snd_midi_event_new(10000, &parser) < 0) {
    fprintf(stderr, "Couldn't create MIDI en-/decoder\n");
    exit(1);
  }
  int npfd;
  struct pollfd *pfd;
  npfd = snd_seq_poll_descriptors_count(seq_handle, POLLIN);
  pfd = (struct pollfd *)alloca(npfd * sizeof(struct pollfd));
  snd_seq_poll_descriptors(seq_handle, pfd, npfd, POLLIN);

  while(1) {
    available = read(serial_fd, &buffer, sizeof(buffer));
    if (available > -1) {
      for (int i = 0; i < available; i++) {
        compute_byte(buffer[i], parser);
      }
    }
    else {
      if (access(arguments.args[0], R_OK) == -1) {
        // device removed
        printf("[serial-to-midi] Error: Device removed -> exit\n");
        // send all notes off
        snd_seq_ev_set_controller(&ev, 0, 0x7B, 0);
        if (snd_seq_close(seq_handle) < 0)
          printf("Error: Couldn't close alsa_seq port\n");
        exit(1);
      }
    }
    if (poll(pfd, npfd, 10) > 0) {
      compute_midi_event(seq_handle, parser, serial_fd);
    }
    nanosleep(&ts, NULL);
  }
}
