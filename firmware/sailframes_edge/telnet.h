// Telnet remote-console transport: a minimal WiFiServer-backed listener
// that feeds line-buffered client input into console.cpp's
// processCommand(). Split out of the old OTA module — it never had
// anything to do with firmware updates, it just happened to live there.
// Off by default (config.h's TELNET_ENABLED_DEFAULT / the 'telneton'
// console command) — see config.h for the Core-1/LWIP contention notes.
#ifndef SAILFRAMES_TELNET_H
#define SAILFRAMES_TELNET_H

#include <Arduino.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

extern WiFiServer telnetServer;
extern WiFiClient telnetClient;
extern bool telnetEnabled;
extern bool telnetServerRunning;
extern String telnetBuffer;

void startTelnetServer();
// Services the telnet client: line-buffered input -> processCommand().
void handleTelnet();

#endif  // SAILFRAMES_TELNET_H
