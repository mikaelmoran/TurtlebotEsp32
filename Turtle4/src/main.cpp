/****************************************************
 * Copyright (C) 2025 Mikael Morán
 *
 * Permission is granted to copy and distribute this
 * file in its original, unaltered form for personal
 * and non-commercial use.
 *
 * Modification, adaptation, or alteration of this code
 * is strictly prohibited without explicit permission.
 *
 * This file is part of the TurtleRobot project.
 *
 * Created by Mikael Morán on 2025-03-29.
 ****************************************************/



 #include <Arduino.h>
 #include <WiFi.h>
 #include <WiFiManager.h>       
 #include <ESPAsyncWebServer.h>
 #include <AsyncTCP.h>
 #include <ArduinoJson.h>
 #include <Preferences.h>
 #include <ESP32Servo.h>
 #include <math.h>
 #include <PrettyOTA.h>
 #include "html.h" 
 #include <ESPmDNS.h>
 
 
 #ifdef DEBUG
   #define DEBUG_PRINT(x)   Serial.print(x)
   #define DEBUG_PRINTLN(x) Serial.println(x)
   #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
 #else
   #define DEBUG_PRINT(x)
   #define DEBUG_PRINTLN(x)
   #define DEBUG_PRINTF(...)
 #endif
 

 #include <freertos/FreeRTOS.h>
 #include <freertos/task.h>
 #include <freertos/semphr.h>
 
 
 enum RobotCommandType { PEN_UP, PEN_DOWN, SET_HEADING, FORWARD, ROTATE };
 
 struct RobotCommand {
   RobotCommandType type;
   float value; 
 };
 
 constexpr int PEN_UP_ANGLE   = 10;
 constexpr int PEN_DOWN_ANGLE = 80;
 constexpr float mmPerPixel   = 2.0f;
 
 const int motor1Pins[4] = {4, 3, 2, 1};  
 const int motor2Pins[4] = {5, 6, 7, 8};  
 constexpr int penServoPin   = 10;         
 

 bool autoPenUpWhenIdle = true;
 

 template<typename T, size_t SIZE>
 class RingBuffer {
 private:
   T buffer[SIZE];
   size_t head;
   size_t tail;
   size_t count;
 public:
   RingBuffer() : head(0), tail(0), count(0) {}
   inline bool empty() const { return count == 0; }
   inline bool full() const { return count == SIZE; }
   inline bool push(const T &item) {
     if (full()) return false;
     buffer[tail] = item;
     tail = (tail + 1) % SIZE;
     ++count;
     return true;
   }
   inline bool pop() {
     if (empty()) return false;
     head = (head + 1) % SIZE;
     --count;
     return true;
   }
   inline T& front() { return buffer[head]; }
   inline void clear() { head = tail = count = 0; }
   inline size_t size() const { return count; }
 };
 
 
 constexpr size_t COMMAND_BUFFER_SIZE = 2048;
 

 void updateWebsocketStatus();
 void setupWebServer();
 

 class PenServoController {
 private:
   Servo penServo;
   const int pin;
   int currentAngle;
   int targetAngle;
   bool isActive;
   unsigned long lastChangeTime;
   static constexpr unsigned long servoDelay = 500; // ms
 public:
   explicit PenServoController(int servoPin)
   : pin(servoPin), currentAngle(PEN_UP_ANGLE), targetAngle(PEN_UP_ANGLE),
     isActive(false), lastChangeTime(0) {}
 
   inline void setAngle(int angle) {
     if (!isActive) {
       penServo.attach(pin);
       isActive = true;
       DEBUG_PRINTLN("Servo ansluten.");
     }
     DEBUG_PRINTF("Sätter vinkel: %d\n", angle);
     penServo.write(angle);
     targetAngle = angle;
     lastChangeTime = millis();
   }
 
   inline void update() {
     if (isActive && (millis() - lastChangeTime >= servoDelay)) {
       
       isActive = false;
       currentAngle = targetAngle;
     }
   }
 
   inline int getCurrentAngle() const { return currentAngle; }
   inline bool isMoving() const { return isActive; }
 };
 

 class MotorController {
 public:
   static const int STEP_SEQUENCE_SIZE = 8;
   int motor1Pins[4];
   int motor2Pins[4];
   const int STEP_SEQ[8][4] = {
     {1,0,0,0},
     {1,1,0,0},
     {0,1,0,0},
     {0,1,1,0},
     {0,0,1,0},
     {0,0,1,1},
     {0,0,0,1},
     {1,0,0,1}
   };
 
   int leftStepIndex;
   int rightStepIndex;
   long leftStepsRemaining;
   long rightStepsRemaining;
   int leftStepDirection;
   int rightStepDirection;
   unsigned long lastStepTime;
   unsigned long STEP_INTERVAL;
   long movementTotalSteps;
 
   MotorController(const int m1Pins[4], const int m2Pins[4]) {
     for (int i = 0; i < 4; i++) {
       motor1Pins[i] = m1Pins[i];
       motor2Pins[i] = m2Pins[i];
       pinMode(motor1Pins[i], OUTPUT);
       digitalWrite(motor1Pins[i], LOW);
       pinMode(motor2Pins[i], OUTPUT);
       digitalWrite(motor2Pins[i], LOW);
     }
     leftStepIndex        = 0;
     rightStepIndex       = 0;
     leftStepsRemaining   = 0;
     rightStepsRemaining  = 0;
     leftStepDirection    = 1;
     rightStepDirection   = 1;
     lastStepTime         = 0;
     STEP_INTERVAL        = 1;
     movementTotalSteps   = 0;
   }
 
   inline void doStepMotor(const int motorPins[4], int stepIndex) {
     for (int i = 0; i < 4; i++) {
       digitalWrite(motorPins[i], STEP_SEQ[stepIndex][i]);
     }
   }
 
 
   inline unsigned long computeAccelInterval(long remainingSteps, long totalSteps) {
     const unsigned long initialInterval = 5;
     const unsigned long minInterval     = 1;
     const long accelSteps = (totalSteps < 100 ? totalSteps : 100);
 
     if (totalSteps == 0) return minInterval;
     long stepsCompleted = totalSteps - remainingSteps;
     if (stepsCompleted < 0) stepsCompleted = 0;
 
     if (stepsCompleted < accelSteps) {
       float faktor = (1.0f - cos(M_PI * stepsCompleted / (float)accelSteps)) * 0.5f;
       return (unsigned long)(minInterval + (initialInterval - minInterval) * (1.0f - faktor));
     } else {
       return minInterval;
     }
   }
 
   inline void updateStepping() {
     unsigned long currentTime = millis();
     unsigned long currentInterval = STEP_INTERVAL;
 
     if (movementTotalSteps > 0) {
       long combinedRemaining = max(leftStepsRemaining, rightStepsRemaining);
       currentInterval = computeAccelInterval(combinedRemaining, movementTotalSteps);
     }
 
     if (currentTime - lastStepTime >= currentInterval) {
       lastStepTime = currentTime;
       if (leftStepsRemaining > 0) {
         leftStepIndex = (leftStepIndex + leftStepDirection + STEP_SEQUENCE_SIZE) % STEP_SEQUENCE_SIZE;
         doStepMotor(motor1Pins, leftStepIndex);
         --leftStepsRemaining;
       }
       if (rightStepsRemaining > 0) {
         rightStepIndex = (rightStepIndex + rightStepDirection + STEP_SEQUENCE_SIZE) % STEP_SEQUENCE_SIZE;
         doStepMotor(motor2Pins, rightStepIndex);
         --rightStepsRemaining;
       }
     }
   }
 
   inline void disableMotors() {
     for (int i = 0; i < 4; i++) {
       digitalWrite(motor1Pins[i], LOW);
       digitalWrite(motor2Pins[i], LOW);
     }
   }
 };
 
 
 class Robot {
 public:
   float posX;
   float posY;
   float posTheta;       
   float currentHeading; 
 
   float wheelDiameter;  
   float wheelBase;      
   int stepsPerRev;      
 
   RingBuffer<RobotCommand, COMMAND_BUFFER_SIZE> commandQueue;
   enum RobotExecState { IDLE, EXECUTING, WAIT_FOR_MOTOR, WAIT_FOR_SERVO };
   RobotExecState execState;
   bool penDown;
 
   MotorController motor;
   PenServoController penController;
   Preferences prefs;
 
   Robot()
     : posX(0.f), posY(0.f), posTheta(0.f), currentHeading(0.f),
       wheelDiameter(63.5f), wheelBase(111.42f), stepsPerRev(4096),
       execState(IDLE), penDown(false),
       motor(motor1Pins, motor2Pins),
       penController(penServoPin)
   {}
 
   inline void loadCalibration() {
     prefs.begin("myCalib", true);
     wheelDiameter = prefs.getFloat("wheelDia", wheelDiameter);
     wheelBase     = prefs.getFloat("wheelBase", wheelBase);
     stepsPerRev   = prefs.getInt("stepsRev", stepsPerRev);
     prefs.end();
     DEBUG_PRINTF("Kalibrering laddad: wheelDiameter=%.2f, wheelBase=%.2f, stepsPerRev=%d\n",
                  wheelDiameter, wheelBase, stepsPerRev);
   }
 
   inline void resetRobotPosition() {
     posX = 0.f;
     posY = 0.f;
     posTheta = 0.f;
     currentHeading = 0.f;
     commandQueue.clear();
     DEBUG_PRINTLN("Robotposition nollställd (0,0).");
   }
 
   inline long calcSteps(float distance_mm) const {
     float circumference = M_PI * wheelDiameter;
     return (long)((fabs(distance_mm) / circumference) * stepsPerRev);
   }
 
   inline void robotForward(float distance_mm) {
     long steps = calcSteps(distance_mm);
     DEBUG_PRINTF("Robot forward: %.1f mm => %ld steg\n", distance_mm, steps);
     posX += distance_mm * sin(posTheta);
     posY += distance_mm * cos(posTheta);
     motor.leftStepsRemaining  = abs(steps);
     motor.rightStepsRemaining = abs(steps);
     int dir = (distance_mm >= 0) ? 1 : -1;
     motor.leftStepDirection  = dir;
     motor.rightStepDirection = dir;
     motor.movementTotalSteps = abs(steps);
   }
 
   inline void robotRotate(float angle_deg) {
     float arcLength = M_PI * wheelBase * fabs(angle_deg) / 360.0f;
     long steps = calcSteps(arcLength);
     DEBUG_PRINTF("Robot rotate: %.1f deg => %ld steg\n", angle_deg, steps);
     posTheta += angle_deg * (M_PI / 180.f);
     while (posTheta >  M_PI) posTheta -= 2.f * M_PI;
     while (posTheta < -M_PI) posTheta += 2.f * M_PI;
     motor.leftStepsRemaining  = abs(steps);
     motor.rightStepsRemaining = abs(steps);
     motor.movementTotalSteps  = abs(steps);
     if (angle_deg > 0) {
       motor.leftStepDirection  = 1;
       motor.rightStepDirection = -1;
     } else {
       motor.leftStepDirection  = -1;
       motor.rightStepDirection = 1;
     }
   }
 
   inline void penUp() {
     DEBUG_PRINTLN("Pen up");
     penController.setAngle(PEN_UP_ANGLE);
     penDown = false;
   }
 
   inline void penDownFunc() {
     DEBUG_PRINTLN("Pen down");
     penController.setAngle(PEN_DOWN_ANGLE);
     penDown = true;
   }
 
   inline void processCommands() {
     switch (execState) {
       case IDLE:
         if (!commandQueue.empty()) {
           RobotCommand cmd = commandQueue.front();
           switch(cmd.type) {
             case PEN_UP:
               penUp();
               execState = WAIT_FOR_SERVO;
               break;
             case PEN_DOWN:
               penDownFunc();
               execState = WAIT_FOR_SERVO;
               break;
             case SET_HEADING:
             {
               float delta = cmd.value - currentHeading;
               if (delta > 180)  delta -= 360;
               if (delta < -180) delta += 360;
               robotRotate(delta);
               execState = WAIT_FOR_MOTOR;
             }
             break;
             case ROTATE:
               robotRotate(cmd.value);
               execState = WAIT_FOR_MOTOR;
               break;
             case FORWARD:
               robotForward(cmd.value);
               execState = WAIT_FOR_MOTOR;
               break;
           }
         }
         break;
 
       case WAIT_FOR_SERVO:
         if (!penController.isMoving()) {
           commandQueue.pop();
           execState = IDLE;
         }
         break;
 
       case WAIT_FOR_MOTOR:
         if (motor.leftStepsRemaining == 0 && motor.rightStepsRemaining == 0) {
           RobotCommand finishedCmd = commandQueue.front();
           if (finishedCmd.type == SET_HEADING) {
             currentHeading = finishedCmd.value;
           }
           commandQueue.pop();
           execState = IDLE;
           DEBUG_PRINTLN("Rörelse klar, nästa kommando");
         }
         break;
 
       default:
         execState = IDLE;
         break;
     }
   }
 

inline void enqueueGoHome() {
  penUp();
  float dx = -posX;
  float dy = -posY;
  float distance = sqrtf(dx * dx + dy * dy);
  
 
  float targetAngleDeg = atan2(dx, dy) * (180.f / M_PI);
  if (targetAngleDeg < 0.f)
    targetAngleDeg += 360.f;

  float deltaDeg = targetAngleDeg - currentHeading;
  if (deltaDeg > 180.f)  deltaDeg -= 360.f;
  if (deltaDeg < -180.f) deltaDeg += 360.f;
  
  commandQueue.push({SET_HEADING, currentHeading + deltaDeg});
  commandQueue.push({FORWARD, distance});
  commandQueue.push({SET_HEADING, 0.f});
}
 
  
   inline void parseAndQueueCommands(String&& jsonStr) {
     if (!commandQueue.empty() || execState != IDLE) {
       DEBUG_PRINTLN("Robot is currently busy, ignoring new commands!");
       return;
     }
   
     StaticJsonDocument<2048> doc;
     DeserializationError error = deserializeJson(doc, jsonStr);
     if (error) {
       DEBUG_PRINT("JSON parse error: ");
       DEBUG_PRINTLN(error.c_str());
       return;
     }
     JsonArray segArray = doc.as<JsonArray>();
   
     float currentX = posX;
     float currentY = posY;
     bool penIsDownLocal = penDown;
   

auto addMoveCommand = [&](float x0, float y0, float x, float y) {

  float dx = x - x0;
  float dy = y - y0;
  float distPix = sqrtf(dx*dx + dy*dy);
  float distMm  = distPix * mmPerPixel; 

 
  float angleRad = atan2(dx, dy);   
  float angleDeg = angleRad * (180.f / M_PI);

  while (angleDeg >= 360.f) angleDeg -= 360.f;
  while (angleDeg < 0.f)    angleDeg += 360.f;

  
  float delta = angleDeg - currentHeading;
  if (delta > 180.f)  delta -= 360.f;   
  if (delta < -180.f) delta += 360.f;  


  commandQueue.push({ROTATE, delta});
  commandQueue.push({FORWARD, distMm});


  currentHeading = angleDeg; 
};
 
     auto ensurePenUp = [&]() {
       if (penIsDownLocal) {
         commandQueue.push({PEN_UP, 0});
         penIsDownLocal = false;
       }
     };
     auto ensurePenDown = [&]() {
       if (!penIsDownLocal) {
         commandQueue.push({PEN_DOWN, 0});
         penIsDownLocal = true;
       }
     };
   
     for (size_t i = 0; i < segArray.size(); i++) {
       JsonObject seg = segArray[i];
       float x1 = seg["x1"].as<float>();
       float y1 = seg["y1"].as<float>();
       float x2 = seg["x2"].as<float>();
       float y2 = seg["y2"].as<float>();
   
       float dx = x1 - currentX;
       float dy = y1 - currentY;
       float gapDistPix = sqrtf(dx*dx + dy*dy);
       float thresholdPix = 2.0f;
   
       if (gapDistPix > thresholdPix) {
         ensurePenUp();
         addMoveCommand(currentX, currentY, x1, y1);
         currentX = x1;
         currentY = y1;
         ensurePenDown();
       } else {
         ensurePenDown();
       }
   
       addMoveCommand(currentX, currentY, x2, y2);
       currentX = x2;
       currentY = y2;
     }
   }
   
   inline void update() {
     motor.updateStepping();
     processCommands();
     penController.update();
   }
 };
 

 Robot robot;
 SemaphoreHandle_t commandQueueMutex;
 AsyncWebServer asyncServer(80);
 AsyncWebSocket ws("/ws");
 PrettyOTA OTAUpdates;
 bool webServerStarted = false;
 

 void updateWebsocketStatus() {
   static unsigned long lastWsTime = 0;
   unsigned long currentTime = millis();
   if (currentTime - lastWsTime > 1000) {
     bool busy = (!robot.commandQueue.empty() || robot.execState != Robot::IDLE);
     float deg = robot.posTheta * (180.f / M_PI);
     String status = "{";
     status += "\"type\":\"status\",";
     status += "\"posX\":" + String(robot.posX) + ",";
     status += "\"posY\":" + String(robot.posY) + ",";
     status += "\"posThetaDeg\":" + String(deg) + ",";
     status += "\"penDown\":" + String(robot.penDown ? "true" : "false") + ",";
     status += "\"isBusy\":" + String(busy ? "true" : "false");
     status += "}";
     ws.textAll(status);
     DEBUG_PRINTLN("WebSocket status uppdaterad:");
     DEBUG_PRINTLN(status);
     lastWsTime = currentTime;
   }
 }
 
 
 void setupWebServer() {
   asyncServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
     request->send_P(200, "text/html", index_html_page);
   });
  
   asyncServer.on("/execute", [](AsyncWebServerRequest *request){
     if (request->hasParam("cmd")) {
       auto p = request->getParam("cmd");
       String cmdStr = p->value();
       cmdStr.replace("+", " ");
       if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
         robot.parseAndQueueCommands(std::move(cmdStr));
         xSemaphoreGive(commandQueueMutex);
       }
       request->send(200, "text/plain", "Commands received");
     } else {
       request->send(400, "text/plain", "Bad Request");
     }
   });
  
   asyncServer.on("/status", [](AsyncWebServerRequest *req){
     float deg = robot.posTheta * (180.f / M_PI);
     String json = "{";
     json += "\"type\":\"status\",";
     json += "\"posX\":"; json += robot.posX; json += ",";
     json += "\"posY\":"; json += robot.posY; json += ",";
     json += "\"posThetaDeg\":"; json += deg; json += ",";
     json += "\"penDown\":";
     json += (robot.penDown ? "true" : "false");
     json += "}";
     req->send(200, "application/json", json);
   });
  
   asyncServer.on("/calibrate_draw", [](AsyncWebServerRequest *req){
     if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
       robot.commandQueue.clear();
       robot.commandQueue.push({PEN_UP, 0});
       robot.commandQueue.push({SET_HEADING, 0});
       robot.commandQueue.push({FORWARD, 50});
       robot.commandQueue.push({PEN_DOWN, 0});
       float side = 100.f;
       robot.commandQueue.push({FORWARD, side});
       robot.commandQueue.push({SET_HEADING, 90});
       robot.commandQueue.push({FORWARD, side});
       robot.commandQueue.push({SET_HEADING, 180});
       robot.commandQueue.push({FORWARD, side});
       robot.commandQueue.push({SET_HEADING, 270});
       robot.commandQueue.push({FORWARD, side});
       robot.commandQueue.push({PEN_UP, 0});
       robot.execState = Robot::IDLE;
       xSemaphoreGive(commandQueueMutex);
     }
     req->send(200, "text/plain", "Ritar 100x100 mm kvadrat. Mät sedan noga!");
   });
  
   asyncServer.on("/calibrate_input", [](AsyncWebServerRequest *req){
     req->send_P(200, "text/html", calibrate_input_html);
   });
  
   asyncServer.on("/calibrate_save", [](AsyncWebServerRequest *req){
     if(req->hasParam("width") && req->hasParam("height")){
       float measuredW = req->getParam("width")->value().toFloat();
       float measuredH = req->getParam("height")->value().toFloat();
       float nominal = 100.f;
       float avgMeasured = (measuredW + measuredH)*0.5f;
       float scaleFactor = nominal / avgMeasured;
  
       robot.wheelDiameter *= scaleFactor;
       robot.prefs.begin("myCalib", false);
       robot.prefs.putFloat("wheelDia", robot.wheelDiameter);
       robot.prefs.putFloat("wheelBase", robot.wheelBase);
       robot.prefs.putInt("stepsRev", robot.stepsPerRev);
       robot.prefs.end();
  
       String msg = "Kalibrering klar! Ny wheelDiameter=";
       msg += String(robot.wheelDiameter, 2);
       msg += " mm, skalfaktor=";
       msg += String(scaleFactor, 4);
       req->send(200, "text/plain", msg);
     } else {
       req->send(400, "text/plain", "Fel: Ange 'width' & 'height'!");
     }
   });
  
   asyncServer.on("/manual", [](AsyncWebServerRequest *req){
     if(req->hasParam("move")){
       String mv = req->getParam("move")->value();
       if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
         if(mv == "F")      { robot.robotForward(10.f); }
         else if(mv == "B") { robot.robotForward(-10.f); }
         else if(mv == "R") { robot.robotRotate(15.f); }
         else if(mv == "L") { robot.robotRotate(-15.f); }
         xSemaphoreGive(commandQueueMutex);
       }
       req->send(200, "text/plain", "OK");
     } else {
       req->send(400, "text/plain", "Param 'move' saknas!");
     }
   });
  
   asyncServer.on("/reset_position", [](AsyncWebServerRequest *req){
     robot.resetRobotPosition();
     req->send(200, "text/plain", "Robotposition nollställd (centrum, 0,0).");
   });
  
   asyncServer.on("/return_home", [](AsyncWebServerRequest *req){
     if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
       robot.enqueueGoHome();
       xSemaphoreGive(commandQueueMutex);
     }
     req->send(200, "text/plain", "Robot återvänder hem (0,0, nos uppåt).");
   });
  
   asyncServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *req){
     req->send(204);
   });
  
   asyncServer.on("/calib_dist_test", HTTP_GET, [](AsyncWebServerRequest *req){
     if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
       robot.commandQueue.clear();
       robot.commandQueue.push({PEN_UP, 0});
       robot.commandQueue.push({FORWARD, 200.f});
       xSemaphoreGive(commandQueueMutex);
     }
     req->send(200, "text/plain", "Kör 200 mm test");
   });
  
   asyncServer.on("/calib_dist_save", HTTP_GET, [](AsyncWebServerRequest *req){
     if (req->hasParam("faktiskDist")) {
       float measured = req->getParam("faktiskDist")->value().toFloat();
       float nominal = 200.f;
       float scaleFactor = measured / nominal;
       robot.wheelDiameter *= scaleFactor;
  
       robot.prefs.begin("myCalib", false);
       robot.prefs.putFloat("wheelDia", robot.wheelDiameter);
       robot.prefs.putFloat("wheelBase", robot.wheelBase);
       robot.prefs.putInt("stepsRev", robot.stepsPerRev);
       robot.prefs.end();
  
       req->send(200, "text/plain", "Ok, ny hjuldiameter: " + String(robot.wheelDiameter, 2));
     } else {
       req->send(400, "text/plain", "Param 'faktiskDist' saknas!");
     }
   });
  
   asyncServer.on("/calib_rot_test", HTTP_GET, [](AsyncWebServerRequest *req){
     if (xSemaphoreTake(commandQueueMutex, (TickType_t)100) == pdTRUE) {
       robot.commandQueue.clear();
       robot.commandQueue.push({PEN_UP, 0});
       robot.commandQueue.push({ROTATE, 360.f});
       xSemaphoreGive(commandQueueMutex);
     }
     req->send(200, "text/plain", "Roterar 360 grader");
   });
  
   asyncServer.on("/calib_rot_save", HTTP_GET, [](AsyncWebServerRequest *req){
     if (req->hasParam("faktiskVinkel")) {
       float measuredAngle = req->getParam("faktiskVinkel")->value().toFloat();
       float nominalAngle  = 360.f;
       float scaleFactor = nominalAngle / measuredAngle;
       robot.wheelBase *= scaleFactor;
  
       robot.prefs.begin("myCalib", false);
       robot.prefs.putFloat("wheelDia", robot.wheelDiameter);
       robot.prefs.putFloat("wheelBase", robot.wheelBase);
       robot.prefs.putInt("stepsRev", robot.stepsPerRev);
       robot.prefs.end();
  
       req->send(200, "text/plain", "Ok, ny hjulbas: " + String(robot.wheelBase, 2));
     } else {
       req->send(400, "text/plain", "Param 'faktiskVinkel' saknas!");
     }
   });
  
   asyncServer.addHandler(&ws);
   asyncServer.begin();
 }
 
 
 void robotTask(void *pvParameters) {
   (void) pvParameters;
   for (;;) {
     if (xSemaphoreTake(commandQueueMutex, (TickType_t)10) == pdTRUE) {
       robot.update();
       xSemaphoreGive(commandQueueMutex);
     }
     if (robot.commandQueue.empty() &&
         robot.motor.leftStepsRemaining == 0 &&
         robot.motor.rightStepsRemaining == 0)
     {
       if (autoPenUpWhenIdle && robot.penDown) {
         robot.penUp();
       }
       robot.motor.disableMotors();
     }
     vTaskDelay(1 / portTICK_PERIOD_MS);
   }
 }
 
 void websocketTask(void *pvParameters) {
   (void) pvParameters;
   for (;;) {
     updateWebsocketStatus();
     ws.cleanupClients();
     vTaskDelay(100 / portTICK_PERIOD_MS);
   }
 }
 

 void setup() {
   Serial.begin(115200);
   delay(1000);
   DEBUG_PRINTLN("Startar...");
 
   commandQueueMutex = xSemaphoreCreateMutex();
   robot.loadCalibration();
   robot.resetRobotPosition();
   robot.penUp();
 
   WiFiManager wifiManager;
   // wifiManager.resetSettings();
   wifiManager.autoConnect("TurtleRobot-AP");
 
   Serial.println("WiFi är anslutet!");
   Serial.print("IP address: ");
   if (!MDNS.begin("turtlebot")) {
    Serial.println("Fel vid start av mDNS");
  } else {
    Serial.println("mDNS responder startad");
  }
   Serial.println(WiFi.localIP());
 
   setupWebServer();
   OTAUpdates.Begin(&asyncServer);
 
   xTaskCreatePinnedToCore(robotTask, "RobotTask", 4096, NULL, 1, NULL, 1);
   xTaskCreatePinnedToCore(websocketTask, "WebsocketTask", 2048, NULL, 1, NULL, 0);
   vTaskDelay(1000 / portTICK_PERIOD_MS);
 }
 
 void loop() {
   vTaskDelay(1000 / portTICK_PERIOD_MS);
 }
 