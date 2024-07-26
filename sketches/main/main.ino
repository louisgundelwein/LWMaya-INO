#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include "mcp_can.h"
#include "TMotor_ServoConnection.h"
#include <Stepper.h>

bool abortFlag = false;

const int stepsPerRevolution = 2048;  // Anzahl der Schritte pro Umdrehung des Motors
const int rolePerMinute = 12;         // Einstellbarer Bereich des 28BYJ-48 Steppermotors ist 0~17 U/min

// Initialisiere die Stepper-Bibliothek auf den Pins 7 bis 4:
Stepper myStepper(stepsPerRevolution, 7, 6, 5, 4);

#define ROWS 8
#define COLS 12

#define CAN0_INT 2
#define MOTOR_ID_1 1  // Motor für Winkel
#define MOTOR_ID_2 2  // Motor für Entfernung
#define MOTOR_ID_3 3  // Motor für Höhe
#define SPI_CS_PIN 10

// Setze dies auf true, um den Simulationsmodus zu aktivieren
#define SIMULATION_MODE false

// Definiere maximale und minimale Positionen für jeden Motor
#define MAX_POS_ANGLE 90
#define MIN_POS_ANGLE -90
#define MAX_POS_DISTANCE 8000
#define MIN_POS_DISTANCE -8000
#define MAX_POS_HEIGHT 8000
#define MIN_POS_HEIGHT -8000

float motorOffset1 = 0;
float motorOffset2 = 0;
float motorOffset3 = 0;

// Maximale Geschwindigkeiten für jeden Motor
float maxSpeedMotor1 = 50;    // Standardwert, kann geändert werden
float maxSpeedMotor2 = 4000;  // Standardwert, kann geändert werden
float maxSpeedMotor3 = 4000;  // Standardwert, kann geändert werden

float minSpeedMotor1 = 10;   // Standardwert, kann geändert werden
float minSpeedMotor2 = 100;  // Standardwert, kann geändert werden
float minSpeedMotor3 = 100;  // Standardwert, kann geändert werden

// Integration Variablen
float integratedPos1 = 0;
float integratedPos2 = 0;
float integratedPos3 = 0;

unsigned long lastUpdateTime = 0;
unsigned long currentUpdateTime = 0;

// Pin Definitionen
const int pin1 = 7;
const int pin2 = 6;
const int pin3 = 5;
const int pin4 = 4;

// Schrittsequenzen für den Motor
const int stepSequence[8][4] = {
  { 1, 0, 0, 0 },
  { 1, 1, 0, 0 },
  { 0, 1, 0, 0 },
  { 0, 1, 1, 0 },
  { 0, 0, 1, 0 },
  { 0, 0, 1, 1 },
  { 0, 0, 0, 1 },
  { 1, 0, 0, 1 }
};

int currentStep = 0;

ArduinoLEDMatrix matrix;
Servo myServo;
uint8_t combinedFrame[ROWS][COLS];

#if !SIMULATION_MODE
MCP_CAN CAN0(SPI_CS_PIN);
TMotor_ServoConnection base_motor_conn(CAN0);
#endif

float currentAngle = 0;
float currentDistance = 0;
float currentHeight = 0;

// Variable zur Steuerung des Loggings
bool logMotor1Position = true;

void setup() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);

  myStepper.setSpeed(rolePerMinute);
  Serial.begin(9600);
  matrix.begin();
  myServo.attach(9);  // Schließe den Servo an Pin 9 an
  Serial.println("Arduino bereit, Befehle zu empfangen.");
  clearFrame();  // Initialisiere die Matrix

#if !SIMULATION_MODE
  while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("Fehler bei der Initialisierung des MCP2515...");
    delay(100);
  }
  CAN0.setMode(MCP_NORMAL);
#endif

  lastUpdateTime = millis();
}

void loop() {

  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    if (receivedData.startsWith("LED:")) {
      parseLEDMessage(receivedData.substring(4));
    } else if (receivedData.startsWith("POS:")) {
      parsePOSMessage(receivedData.substring(4));
    } else if (receivedData.startsWith("ANGLE:")) {
      parseAngleMessage(receivedData.substring(6));
    } else if (receivedData.startsWith("DISTANCE:")) {
      parseDistanceMessage(receivedData.substring(9));
    } else if (receivedData.startsWith("HEIGHT:")) {
      parseHeightMessage(receivedData.substring(7));
    } else if (receivedData.startsWith("GRIPPER:")) {
      parseGripperMessage(receivedData.substring(8));
    } else if (receivedData.startsWith("DEMO:START")) {
      startDemo();
    } else if (receivedData.startsWith("DUTY:")) {
      parseDutyMessage(receivedData.substring(5));
    } else if (receivedData.startsWith("SPEED:")) {
      parseSpeedMessage(receivedData.substring(6));
    } else if (receivedData.startsWith("goHome")) {
      move_motor_to_position(3, 0);
      move_motor_to_position(2, 0);
      move_motor_to_position(1, 0);
    } else if (receivedData.startsWith("CALIBRATE:")) {
      int firstComma = receivedData.indexOf(',');
      float angle = receivedData.substring(10, firstComma).toFloat();
      float distance = receivedData.substring(firstComma + 1).toFloat();

      move_motor_to_position(MOTOR_ID_1, angle);
      move_motor_to_position(MOTOR_ID_2, distance);
      parseGripperMessage("CLOSE");
    }else if (receivedData.startsWith("HOME")) {
      float old_position_1 = base_motor_conn.get_pos(MOTOR_ID_1);
      float old_position_2 = base_motor_conn.get_pos(MOTOR_ID_2);
      float old_position_3 = base_motor_conn.get_pos(MOTOR_ID_3);
      Serial.print("Setze Home ... ");
      Serial.println("Alte Positionen:");
      Serial.println(old_position_1);
      Serial.println(old_position_2);
      Serial.println(old_position_3);

      // Speichern der aktuellen Position als Offset
      motorOffset1 = old_position_1;
      motorOffset2 = old_position_2;
      motorOffset3 = old_position_3;

      Serial.println("Offsets gesetzt:");
      Serial.println(motorOffset1);
      Serial.println(motorOffset2);
      Serial.println(motorOffset3);

      float new_position_1 = base_motor_conn.get_pos(MOTOR_ID_1);
      float new_position_2 = base_motor_conn.get_pos(MOTOR_ID_2);
      float new_position_3 = base_motor_conn.get_pos(MOTOR_ID_3);
      Serial.println("Neue Positionen:");
      Serial.println(new_position_1);
      Serial.println(new_position_2);
      Serial.println(new_position_3);
    } else if (receivedData.startsWith("LOG:")) {
      if (receivedData.substring(4) == "ON") {
        logMotor1Position = true;
        Serial.println("Protokollierung der Position von Motor 1: EIN");
      } else if (receivedData.substring(4) == "OFF") {
        logMotor1Position = false;
        Serial.println("Protokollierung der Position von Motor 1: AUS");
      }
    }
    matrix.renderBitmap(combinedFrame, ROWS, COLS);  // Zeige die Gesamtmatrix
  }
  base_motor_conn.can_receive();  // Fügt dies hinzu, um CAN-Nachrichten regelmäßig zu verarbeiten

  // Update Motor Positions
  updateMotorPositions();
}

void parseLEDMessage(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  int row = data.substring(0, firstComma).toInt();
  int col = data.substring(firstComma + 1, secondComma).toInt();
  int value = data.substring(secondComma + 1).toInt();

  if (row >= 0 && row < ROWS && col >= 0 && col < COLS) {
    combinedFrame[row][col] = value;
  }
}

void parsePOSMessage(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);

  float angleFrom = data.substring(0, firstComma).toFloat();
  float distanceFrom = data.substring(firstComma + 1, secondComma).toFloat();
  float angleTo = data.substring(secondComma + 1, thirdComma).toFloat();
  float distanceTo = data.substring(thirdComma + 1).toFloat();

#if SIMULATION_MODE
  Serial.print("Simuliert: Von Winkel: ");
  Serial.print(angleFrom);
  Serial.print(" Von Entfernung: ");
  Serial.println(distanceFrom);
  Serial.print("Simuliert: Zu Winkel: ");
  Serial.print(angleTo);
  Serial.print(" Zu Entfernung: ");
  Serial.println(distanceTo);
#else
  Serial.print("Bewege von Winkel: ");
  Serial.print(angleFrom);
  Serial.print(" Entfernung: ");
  Serial.println(distanceFrom);
  move_motor_to_position(MOTOR_ID_1, angleFrom);
  move_motor_to_position(MOTOR_ID_2, distanceFrom);
  parseGripperMessage("CLOSE");
  move_motor_to_position(MOTOR_ID_3, -3000);
  parseGripperMessage("OPEN");
  move_motor_to_position(MOTOR_ID_3, -3600);
  delay(5);
  base_motor_conn.set_speed(MOTOR_ID_3, 0);
  delay(5);
  updateMotorPositions();
  parseGripperMessage("CLOSE");
  move_motor_to_position(MOTOR_ID_3, -3500);
  parseGripperMessage("CLOSE");
  move_motor_to_position(MOTOR_ID_3, -3400);
  parseGripperMessage("CLOSE");
  move_motor_to_position(MOTOR_ID_3, -10);
  delay(5);
  base_motor_conn.set_speed(MOTOR_ID_3, 0);
  delay(5);
  updateMotorPositions();
  parseGripperMessage("CLOSE");
  Serial.print("Bewege zu Winkel: ");
  Serial.print(angleTo);
  Serial.print(" Entfernung: ");
  Serial.println(distanceTo);
  move_motor_to_position(MOTOR_ID_1, angleTo);
  move_motor_to_position(MOTOR_ID_2, distanceTo);
  move_motor_to_position(MOTOR_ID_3, -3600);
  delay(5);
  base_motor_conn.set_speed(MOTOR_ID_3, 0);
  delay(5);
  updateMotorPositions();
  parseGripperMessage("OPEN");
  move_motor_to_position(MOTOR_ID_3, -1);
  delay(5);
  base_motor_conn.set_speed(MOTOR_ID_3, 0);
  delay(5);
  updateMotorPositions();
  parseGripperMessage("CLOSE");
#endif
}

void parseAngleMessage(String data) {
  float angleChange = data.toFloat();
  currentAngle += angleChange;
  currentAngle = constrain(currentAngle, MIN_POS_ANGLE, MAX_POS_ANGLE);  // Begrenze den Winkel auf die festgelegten Grenzwerte
#if SIMULATION_MODE
  Serial.print("Simuliert: Neuer Winkel: ");
  Serial.println(currentAngle);
#else
  move_motor_to_position(MOTOR_ID_1, currentAngle);
#endif
}

void parseDistanceMessage(String data) {
  float distanceChange = data.toFloat();
  currentDistance += distanceChange;
  currentDistance = constrain(currentDistance, MIN_POS_DISTANCE, MAX_POS_DISTANCE);  // Begrenze die Entfernung auf die festgelegten Grenzwerte
#if SIMULATION_MODE
  Serial.print("Simuliert: Neue Entfernung: ");
  Serial.println(currentDistance);
#else
  move_motor_to_position(MOTOR_ID_2, currentDistance);
#endif
}

void parseHeightMessage(String data) {
  float heightChange = data.toFloat();
  currentHeight += heightChange;
  currentHeight = constrain(currentHeight, MIN_POS_HEIGHT, MAX_POS_HEIGHT);  // Begrenze die Höhe auf die festgelegten Grenzwerte
#if SIMULATION_MODE
  Serial.print("Simuliert: Neue Höhe: ");
  Serial.println(currentHeight);
#else
  move_motor_to_position(MOTOR_ID_3, currentHeight);
#endif
}

void parseGripperMessage(String data) {
  if (data.equals("OPEN")) {
    for (int i = 0; i < stepsPerRevolution; i++) {
      stepMotor(-1);  // vorwärts
      delay(2);
    }
    delay(200);
    Serial.println("Greifer geöffnet");
  } else if (data.equals("CLOSE")) {
    for (int i = 0; i < stepsPerRevolution; i++) {
      stepMotor(1);  // rückwärts
      delay(2);
    }
    delay(200);
    Serial.println("Greifer geschlossen");
  }
}

void parseSpeedMessage(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  int motorId = data.substring(0, firstComma).toInt();
  float speed = data.substring(firstComma + 1, secondComma).toFloat();

  if (motorId == MOTOR_ID_1) {
    base_motor_conn.set_speed(MOTOR_ID_1, speed);
    Serial.print("Geschwindigkeit für Motor 1 gesetzt auf: ");
    Serial.println(speed);
  } else if (motorId == MOTOR_ID_2) {
    base_motor_conn.set_speed(MOTOR_ID_2, speed);
    Serial.print("Geschwindigkeit für Motor 2 gesetzt auf: ");
    Serial.println(speed);
  } else if (motorId == MOTOR_ID_3) {
    base_motor_conn.set_speed(MOTOR_ID_3, speed);
    Serial.print("Geschwindigkeit für Motor 3 gesetzt auf: ");
    Serial.println(speed);
  }
}


void startDemo() {
  Serial.println("Starte Demo...");
  for (int i = 0; i < 5; i++) {
#if SIMULATION_MODE
    Serial.println("Simuliert: Demo Schritt");
#else
    // Drehung in eine Richtung
    Serial.println("Setze Motorpositionen auf: 1800, 1000, 500");
    move_motor_to_position(MOTOR_ID_1, 1800);
    move_motor_to_position(MOTOR_ID_2, 1000);
    move_motor_to_position(MOTOR_ID_3, 500);
#endif
    delay(2000);

#if !SIMULATION_MODE
    // Drehung in die andere Richtung
    Serial.println("Setze Motorpositionen auf: -1800, -1000, -500");
    move_motor_to_position(MOTOR_ID_1, -1800);
    move_motor_to_position(MOTOR_ID_2, -1000);
    move_motor_to_position(MOTOR_ID_3, -500);
#endif
    delay(2000);
  }
}

void clearFrame() {
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      combinedFrame[i][j] = 0;  // Alle LEDs ausschalten
    }
  }
}

void parseDutyMessage(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  int motorId = data.substring(0, firstComma).toInt();
  float dutyCycle = data.substring(firstComma + 1, secondComma).toFloat();

#if SIMULATION_MODE
  Serial.print("Simuliert: Setze Duty Cycle für Motor ");
  Serial.print(motorId);
  Serial.print(" auf ");
  Serial.print(dutyCycle);
  Serial.println("%");
#else
  Serial.print("Setze Duty Cycle für Motor ");
  Serial.print(motorId);
  Serial.print(" auf ");
  Serial.print(dutyCycle);
  Serial.println("%");
  base_motor_conn.set_duty_cycle(motorId, dutyCycle);  // Setze den Duty Cycle des Motors
#endif
}

void move_motor_to_position(long unsigned int id, float target_position) {
#if !SIMULATION_MODE
  // Berechne die Zielposition unter Berücksichtigung des Offsets
  if (id == MOTOR_ID_1) {
    target_position += motorOffset1;
    target_position = constrain(target_position, MIN_POS_ANGLE, MAX_POS_ANGLE);
  } else if (id == MOTOR_ID_2) {
    target_position += motorOffset2;
    target_position = constrain(target_position, MIN_POS_DISTANCE, MAX_POS_DISTANCE);
  } else if (id == MOTOR_ID_3) {
    target_position += motorOffset3;
    target_position = constrain(target_position, MIN_POS_HEIGHT, MAX_POS_HEIGHT);
  }

  float maxSpeed = 500;  // Default max speed if no specific max speed is set
  float minSpeed = 0;    // Default min speed if no specific min speed is set
  if (id == MOTOR_ID_1) {
    maxSpeed = maxSpeedMotor1;
    minSpeed = minSpeedMotor1;
  } else if (id == MOTOR_ID_2) {
    maxSpeed = maxSpeedMotor2;
    minSpeed = minSpeedMotor2;
  } else if (id == MOTOR_ID_3) {
    maxSpeed = maxSpeedMotor3;
    minSpeed = minSpeedMotor3;
  }

  float current_position_out_of_loop = base_motor_conn.get_pos(id);

  // Integrierte Position verwenden, wenn außerhalb von ±100
  if (id == MOTOR_ID_1) {
    if (abs(current_position_out_of_loop) >= 100) {
      current_position_out_of_loop = integratedPos1;
    }
  } else if (id == MOTOR_ID_2) {
    if (abs(current_position_out_of_loop) >= 100) {
      current_position_out_of_loop = integratedPos2;
    }
  } else if (id == MOTOR_ID_3) {
    if (abs(current_position_out_of_loop) >= 100) {
      current_position_out_of_loop = integratedPos3;
    }
  }

  float distance = abs(target_position - current_position_out_of_loop);
  float rampUpDistance = distance * 0.25;  // 25% der Strecke zum Beschleunigen
  float rampDownDistance = distance * 0.25;  // 25% der Strecke zum Abbremsen

  if (distance < rampUpDistance + rampDownDistance) {
    maxSpeed = maxSpeed * (distance / (rampUpDistance + rampDownDistance));
  }

  while (true) {
    // Prüfe auf eingehende Nachrichten
    /*checkForAbort();
    if (abortFlag) {
      base_motor_conn.set_speed(id, 0);  // Stoppe den Motor
      abortFlag = false;  // Reset abort flag
      return;
    }*/

    // Prüfe auf eingehende Nachrichten
    if (!digitalRead(CAN0_INT)) {
      // Hole alle Nachrichten in der Warteschlange
      while (CAN_MSGAVAIL == CAN0.checkReceive()) {
        // Empfange und verarbeite Nachrichten
        base_motor_conn.can_receive();
      }
    }

    float current_position = base_motor_conn.get_pos(id);

    // Integrierte Position verwenden, wenn außerhalb von ±100
    if (id == MOTOR_ID_1) {
      if (abs(current_position) >= 2000) {
        current_position = integratedPos1;
      }
    } else if (id == MOTOR_ID_2) {
      if (abs(current_position) >= 2000) {
        current_position = integratedPos2;
      }
    } else if (id == MOTOR_ID_3) {
      if (abs(current_position) >= 2000) {
        current_position = integratedPos3;
      }
    }

    float position_difference = target_position - current_position;

    // Passe die Geschwindigkeit basierend auf der Entfernung zur Zielposition an
    float speed = maxSpeed;
    if (abs(position_difference) < rampDownDistance) {
      speed = maxSpeed * (abs(position_difference) / rampDownDistance);  // Verlangsamen
    } else if (distance - abs(position_difference) < rampUpDistance) {
      speed = maxSpeed * ((distance - abs(position_difference)) / rampUpDistance);  // Beschleunigen
    }

    // Stelle sicher, dass die Geschwindigkeit mindestens minSpeed beträgt
    if (abs(speed) < minSpeed) {
      speed = (position_difference < 0) ? -minSpeed : minSpeed;
    }

    // Geschwindigkeit entsprechend dem Vorzeichen des Positionsunterschieds einstellen
    speed = (position_difference < 0) ? -abs(speed) : abs(speed);

    base_motor_conn.set_speed(id, speed);
    updateMotorPositions();

    Serial.print("Diff: ");
    Serial.print(position_difference);
    Serial.print(" Speed: ");
    Serial.print(speed);
    Serial.print(" Target: ");
    Serial.print(target_position);
    Serial.print(" Current: ");
    Serial.println(current_position);

    if (abs(current_position - target_position) <= (id == 1 ? 0.2 : 5)) {
      break; // Beenden, wenn die Zielposition erreicht ist
    }
    updateMotorPositions();
  }

  base_motor_conn.set_speed(id, 0);  // Stoppe den Motor
  updateMotorPositions();

#endif
}


void updateMotorPositions() {
  currentUpdateTime = millis();
  unsigned long deltaTime = currentUpdateTime - lastUpdateTime;
  float deltaTimeS = deltaTime * 0.001;

  if (!SIMULATION_MODE) {
    // Prüfe auf eingehende Nachrichten
    if (!digitalRead(CAN0_INT)) {
      // Hole alle Nachrichten in der Warteschlange
      while (CAN_MSGAVAIL == CAN0.checkReceive()) {
        // Empfange und verarbeite Nachrichten
        base_motor_conn.can_receive();
      }
    }

    // Lese aktuelle Positionen
    float pos1 = base_motor_conn.get_pos(MOTOR_ID_1);
    float pos2 = base_motor_conn.get_pos(MOTOR_ID_2);
    float pos3 = base_motor_conn.get_pos(MOTOR_ID_3);

    // Integriere Positionen wenn außerhalb von ±100
    if (pos1 >= 2000 || pos1 <= -2000) {
      float speed1 = base_motor_conn.get_speed(MOTOR_ID_1);  // Umdrehungen pro Sekunde
      //integration: use speed (inverted) to update pos
      //only updates set values, uses no integration value or default start positon

      float speed1LiveMRPS = speed1 / 60.0;
      float speed1LiveRPS = (speed1LiveMRPS / 14.0) / 6.0;
      float deltaTimeS = (currentUpdateTime - lastUpdateTime) * 0.001;

      integratedPos1 = integratedPos1 + ((speed1LiveRPS * deltaTimeS) * 360.0);
    } else {
      integratedPos1 = pos1;
    }

    if (pos2 >= 2000 || pos2 <= -2000) {
      float speed2 = base_motor_conn.get_speed(MOTOR_ID_2);  // Umdrehungen pro Sekunde
      //integration: use speed (inverted) to update pos
      //only updates set values, uses no integration value or default start positon

      float speed2LiveMRPS = speed2 / 60.0;
      float speed2LiveRPS = (speed2LiveMRPS / 14.0) / 6.0;
      float deltaTimeS = (currentUpdateTime - lastUpdateTime) * 0.001;

      integratedPos2 = integratedPos2 + ((speed2LiveRPS * deltaTimeS) * 360.0);
    } else {
      integratedPos2 = pos2;
    }

    if (pos3 >= 2000 || pos3 <= -2000) {
      float speed3 = base_motor_conn.get_speed(MOTOR_ID_3);  // Umdrehungen pro Sekunde
      //integration: use speed (inverted) to update pos
      //only updates set values, uses no integration value or default start positon

      float speed3LiveMRPS = speed3 / 60.0;
      float speed3LiveRPS = (speed3LiveMRPS / 14.0) / 6.0;
      float deltaTimeS = (currentUpdateTime - lastUpdateTime) * 0.001;

      integratedPos3 = integratedPos3 + ((speed3LiveRPS * deltaTimeS) * 360.0);

    } else {
      integratedPos3 = pos3;
    }

    //Serial.println("Positions: ");
    //Serial.print(integratedPos1);
    //Serial.print(", ");
    //Serial.print(integratedPos2);
    //Serial.print(", ");
    //Serial.println(integratedPos3);
    delay(5);
  }

  lastUpdateTime = currentUpdateTime;
}

void stepMotor(int direction) {
  currentStep = (currentStep + direction + 8) % 8;

  digitalWrite(pin1, stepSequence[currentStep][0]);
  digitalWrite(pin2, stepSequence[currentStep][1]);
  digitalWrite(pin3, stepSequence[currentStep][2]);
  digitalWrite(pin4, stepSequence[currentStep][3]);
}

/*void checkForAbort() {
  while (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    if (receivedData.indexOf("ABORT") != -1) {
      abortFlag = true;
      Serial.println("Abort received, stopping all activities.");
      // Leere den Serial-Speicher
      while (Serial.available() > 0) {
        Serial.read();
      }
      break;
    }
  }
}*/

