// Interactive command console — shared by the USB-serial line and the
// telnet client (telnet.cpp's handleTelnet feeds lines here too). One large
// dispatcher (`status`, `gps`, `imu`, `ls`, `cat`, `upload`, `wifi`,
// `reboot`, `race arm`, ... — see the `help` command's own listing for
// the full set) rather than a command-per-file, since every branch is a
// few lines of read-and-print over state owned by another module.
#ifndef SAILFRAMES_CONSOLE_H
#define SAILFRAMES_CONSOLE_H

#include <Arduino.h>

// Print to both Serial and the connected telnet client (if any).
void tprint(const char* msg);
void tprintf(const char* fmt, ...);
void tprintln(const char* msg);

// Parses and executes one command line from serial or telnet.
void processCommand(String cmd, bool fromTelnet);
// Reads one line from USB-serial (if available) and runs it.
void handleSerialCommand();

#endif  // SAILFRAMES_CONSOLE_H
