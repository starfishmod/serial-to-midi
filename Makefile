default: serial-to-midi

serial-to-midi: 
    gcc -std=c99 -o serial-to-midi serial-to-midi.c -lasound

clean:
    -rm -f serial-to-midi
