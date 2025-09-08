// Variables storing data from the UI
float Tuned_Kp[3] = { ROLL_RATE_KP, PITCH_RATE_KP, YAW_RATE_KP };       // P gain for (Roll), (Pitch), and (Yaw)
float Tuned_Ki[3] = { ROLL_RATE_KI, PITCH_RATE_KI, YAW_RATE_KI };       // I gain for (Roll), (Pitch), and (Yaw)
float Tuned_Kd[3] = { ROLL_RATE_KD, PITCH_RATE_KD, YAW_RATE_KD };       // D gain for (Roll), (Pitch), and (Yaw)
float Tuned_AngularGain[2] = { ROLL_ANGULAR_GAIN, PITCH_ANGULAR_GAIN }; // Angular Gain for (Roll) and (Pitch)

#ifdef USE_WIFI

  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
  #include <Arduino_JSON.h>
  #include <LittleFS.h>

  // WiFi informations here:
  const char* ssid = "QFC WIFI";
  const char* password = "QFCpassword";

  static unsigned long lastEventTime = 0; // The last event time time

  String motors[MTR_NUMBER] = {"right_front", "right_back", "left_back", "left_front"};

  String dimensions[3] = {"x", "y", "z"}; // The X Y Z dimensions (sensor data arrays)

  AsyncWebServer server(80); // Initiate the server

  AsyncEventSource events("/events"); // Initiate the events source

  static const unsigned long event_period = 200; // The period it takes for the Dashboard to be updated [ms]

  JSONVar JSONdata; // Data to be logged to the user interface in JSON format

  unsigned long cnt = 0; // Events counter

  void notFound(AsyncWebServerRequest *request); // not found server response


  // This function should be in the main setup
  void wifi_setup(void) {

    // Set the device as a Station and Soft Access Point simultaneously
    WiFi.mode(WIFI_STA);

    // Begin the WiFi connection
    WiFi.begin(ssid, password);

    // Begin the Little File System
    LittleFS.begin();

    // Check if the Wifi connection is established
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      TURN_OFF_LED;
      delay(500);
      TURN_ON_YELLOW_LED;
      delay(500);
      Serial.println("WiFi Failed!");
    }
    Serial.println();
    Serial.print("Server: IP Address: ");
    Serial.println(WiFi.localIP()); // Log the server IP


    // Send the web page with input fields to client
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(LittleFS, "/index.html");
    });


    // Send a GET request to <ESP_IP>/update?arg=<inputArguement>
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
      for (uint8_t i = 0; i < 3; i++) {
          // GET Kp values on <ESP_IP>/update?Kp=<inputArguement>
          if (request->getParam("Kp" + String(i))->value() != "") {
              Tuned_Kp[i] = atof(&request->getParam("Kp" + String(i))->value().c_str()[0]);
          } else {
              Tuned_Kp[i] = 0.0f;
          }
          // GET Ki values on <ESP_IP>/update?Ki=<inputArguement>
          if (request->getParam("Ki" + String(i))->value() != "") {
              Tuned_Ki[i] = atof(&request->getParam("Ki" + String(i))->value().c_str()[0]);
          } else {
              Tuned_Ki[i] = 0.0f;
          }
          // GET Kd values on <ESP_IP>/update?Kd=<inputArguement>
          if (request->getParam("Kd" + String(i))->value() != "") {
              Tuned_Kd[i] = atof(&request->getParam("Kd" + String(i))->value().c_str()[0]);
          } else {
              Tuned_Kd[i] = 0.0f;
          }
      }
      for (uint8_t i = 0; i < 2; i++) {
        // GET Angular gain value
        if (request->getParam("AngularGain" + String(i))->value() != "") {
            Tuned_AngularGain[i] = atof(&request->getParam("AngularGain" + String(i))->value().c_str()[0]);
        } else {
            Tuned_AngularGain[i] = 0.0f;
        }
      }
      request->send(200, "text/plain", "OK");
      request->redirect("/");
    });

    events.onConnect([](AsyncEventSourceClient *client){
      // send event with message "hello!", id current millis
      // and set reconnect delay to 1 second
      client->send("hello!", NULL, millis(), 10000);
    });

    server.addHandler(&events); // Add an event handler
    server.onNotFound(notFound); // Check if the request is not found
    server.begin(); // Begin the server
  }

  // This function should be in the main setup
  void wifi_loop(float Quads_Roll_Angle, float Quads_Pitch_Angle) {

    // Receive the current sample ID
    JSONdata["id"] = String(cnt);

    // Receive the current angles in the dashboard
    JSONdata["angles"][dimensions[0]] = Quads_Roll_Angle * RAD_TO_DEG;
    JSONdata["angles"][dimensions[1]] = Quads_Pitch_Angle * RAD_TO_DEG;

    // Receive the current motor throttles in the dashboard
    for (int i = 0; i < MTR_NUMBER; i++) {
      JSONdata["motor_throttles"][motors[i]] = map(motorInputs[i], 1000, 2000, 0, 1000);
    }

    // Update the Dashboard
    if (millis() - lastEventTime > event_period) {
      String JSONdatastring = JSON.stringify(JSONdata);
      events.send(JSONdatastring.c_str(), "data", millis());
      lastEventTime = millis();
      // Increment the event counter
      cnt++;
    }
  }

  void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
  }
#endif