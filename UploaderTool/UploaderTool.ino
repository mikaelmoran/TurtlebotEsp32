#include <WiFi.h>           // Include the WiFi library
#include <WebServer.h>      // Include the WebServer library
#include <Update.h>         // Include the Update library (comes with the ESP32 Arduino core)

// Configure static IP for the access point
IPAddress local_IP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// AP credentials
const char* ap_ssid = "ESP32-C3-TURTLE-AP-FLASHER";     // SSID of the access point
const char* ap_password = "12345678"; // Password (min 8 characters for encryption)

// Create a web server on port 80
WebServer server(80);

// HTML upload form for firmware update
const char* uploadForm = 
  "<html>"
  "<head><meta charset='utf-8'><title>Turtle Firmware Uploader for ESP32-C3 mini</title></head>"
  "<body>"
  "<h1>Firmware Uploader</h1>"
  "<form method='POST' action='/update' enctype='multipart/form-data'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Upload Firmware'>"
  "</form>"
  "</body>"
  "</html>";

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set the static IP address for the access point
  WiFi.softAPConfig(local_IP, gateway, subnet);
  // Start the access point
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Define the root endpoint to serve the upload form
  server.on("/", HTTP_GET, [](){
    server.send(200, "text/html", uploadForm);
  });

  // Define the firmware update endpoint for POST requests
  server.on("/update", HTTP_POST, [](){
    // When the file upload is complete, send a response and restart the device
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "Firmware update received. Restarting...");
    ESP.restart();
  }, [](){
    // Handle the file upload process
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Start updating: %s\n", upload.filename.c_str());
      uint32_t maxSketchSpace = ESP.getFreeSketchSpace();
      if (!Update.begin(maxSketchSpace)) { // Start the OTA update with maximum available space
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u bytes received\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  // Start the web server
  server.begin();
  Serial.println("Web server started. Connect to the AP and open the IP address in your browser to upload firmware.");
}

void loop() {
  // Handle incoming client requests
  server.handleClient();
}
