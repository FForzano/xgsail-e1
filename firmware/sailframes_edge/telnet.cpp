// Telnet transport glue — see telnet.h.
#include "telnet.h"
#include "config.h"
#include "console.h"
#include "shared_state.h"

WiFiServer telnetServer(23);
WiFiClient telnetClient;
bool telnetEnabled = TELNET_ENABLED_DEFAULT;
bool telnetServerRunning = false;
String telnetBuffer = "";

void startTelnetServer() {
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  telnetServerRunning = true;
  Serial.println("[TELNET] Server started on port 23");
}

void handleTelnet() {
  // Bail if the listener was never started OR if Core 0 is mid-upload.
  // telnetServer.hasClient() goes through LWIP and deadlocks under
  // sustained Core 0 traffic (firmware 2026.05.03.04 fleet hang).
  if (!telnetServerRunning || wifiBusy) return;

  // Check for new clients
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient) telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.println("\n=================================");
      telnetClient.printf("  SailFrames Edge %s\n", FW_VERSION);
      telnetClient.printf("  Boat: %s\n", config.boat_id);
      telnetClient.println("  Type 'help' for commands");
      telnetClient.println("=================================\n");
      telnetClient.print("> ");
      Serial.println("[TELNET] Client connected");
    } else {
      // Reject additional clients
      telnetServer.available().stop();
    }
  }

  // Handle client input
  if (telnetClient && telnetClient.connected()) {
    while (telnetClient.available()) {
      char c = telnetClient.read();
      if (c == '\n' || c == '\r') {
        if (telnetBuffer.length() > 0) {
          telnetClient.println();  // Echo newline
          processCommand(telnetBuffer, true);
          telnetBuffer = "";
          telnetClient.print("> ");
        }
      } else if (c == 127 || c == 8) {  // Backspace
        if (telnetBuffer.length() > 0) {
          telnetBuffer.remove(telnetBuffer.length() - 1);
          telnetClient.print("\b \b");  // Erase character
        }
      } else if (c >= 32 && c < 127) {  // Printable
        telnetBuffer += c;
        telnetClient.print(c);  // Echo
      }
    }
  }
}
