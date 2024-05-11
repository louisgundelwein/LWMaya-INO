#include "Arduino_LED_Matrix.h"

#define ROWS 8  // Anzahl der Zeilen der Gesamtmatrix
#define COLS 12 // Anzahl der Spalten der Gesamtmatrix

ArduinoLEDMatrix matrix;
uint8_t combinedFrame[ROWS][COLS];

void setup() {
    Serial.begin(9600);
    matrix.begin();
    Serial.println("Arduino bereit, Befehle zu empfangen.");
    clearFrame(); // Initialisiere die Matrix
}

void loop() {
    if (Serial.available() > 0) {
        String receivedData = Serial.readStringUntil('\n');
        if (receivedData.startsWith("LED:")) {
            parseLEDMessage(receivedData.substring(4));
        } else if (receivedData.startsWith("POS:")) {
            parsePOSMessage(receivedData.substring(4));
        }
        matrix.renderBitmap(combinedFrame, ROWS, COLS); // Zeige die Gesamtmatrix
    }
}

// Parse a single LED message with coordinates
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

// Parse POS message containing angle and distance
void parsePOSMessage(String data) {
    int commaIndex = data.indexOf(',');
    int angle = data.substring(0, commaIndex).toInt();
    int distance = data.substring(commaIndex + 1).toInt();

    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(", Distance: ");
    Serial.println(distance);
}

void clearFrame() {
    for (int i = 0; i < ROWS; i++) {
        for (int j = j; j < COLS; j++) {
            combinedFrame[i][j] = 0; // Alle LEDs ausschalten
        }
    }
}
