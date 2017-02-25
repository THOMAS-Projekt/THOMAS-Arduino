#include <TimerOne.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <SoftwareServo.h>

#define STEPPER_DIR_PIN 2
#define STEPPER_STEP_PIN 3

#define SERVO_PIN 5
#define SERVO_HORIZONTAL_HEIGHT 108

LIDARLite lidar;
SoftwareServo servo;

volatile int stepCount = 0;

bool scanRunning = false;
int biasUpdateCount = 0;

void setup() {
	Serial.begin(115200);

	// Handshake-Byte senden
	Serial.write(255);

	// Arduino-ID senden
	uint8_t package[] = { 1 };
	sendPackage(sizeof(package), package);
}

void loop() {
	if (!scanRunning)
		return;

	float angle = ((float)stepCount / 3200.0f) * 360.0f;
	short distance;

	if (biasUpdateCount >= 100) {
		distance = lidar.distance(true);
		biasUpdateCount = 0;
	}
	else {
		distance = lidar.distance(false);
		biasUpdateCount++;
	}

	uint8_t *angleBytes = reinterpret_cast<uint8_t*>(&angle);
	uint8_t *distanceBytes = reinterpret_cast<uint8_t*>(&distance);
	uint8_t event[] = { 100, angleBytes[0], angleBytes[1], angleBytes[2], angleBytes[3], distanceBytes[0], distanceBytes[1] };
	sendPackage(sizeof(event), event);
}

void serialEvent() {
	while (Serial.available()) {
		uint8_t packageLength = Serial.read();
		uint8_t package[packageLength];

		Serial.readBytes(package, packageLength);

		processPackage(packageLength, package);
	}
}

void processPackage(uint8_t packageLength, uint8_t package[]) {
	switch (package[0]) {
		case 1:
			processSetup(package);

			break;

		case 10:
			processScanStart(package);

			break;

		case 11:
			processScanStop(package);

			break;
	}
}

void processSetup(uint8_t request[]) {
	lidar.begin(0, true);
	lidar.configure(0);

	pinMode(STEPPER_DIR_PIN, OUTPUT);
	pinMode(STEPPER_STEP_PIN, OUTPUT);

	digitalWrite(STEPPER_DIR_PIN, HIGH);

	servo.attach(SERVO_PIN);
	servo.write(SERVO_HORIZONTAL_HEIGHT);

	for (int i = 0; i < 500; i++) {
		servo.refresh();
		delay(2);
	}

	servo.detach();

	Timer1.initialize();

	uint8_t response[] = { request[0], 1 };
	sendPackage(sizeof(response), response);
}

void processScanStart(uint8_t request[]) {
	int roundCount = request[1];

	if (roundCount == 0)
		roundCount = 1;

	Timer1.attachInterrupt(onTimerTick, 1000000.0f / (3200 * roundCount));

	uint8_t response[] = { request[0], 1 };
	sendPackage(sizeof(response), response);

	scanRunning = true;
}

void processScanStop(uint8_t request[]) {
	Timer1.detachInterrupt();

	uint8_t response[] = { request[0], 1 };
	sendPackage(sizeof(response), response);

	scanRunning = false;
}

void onTimerTick() {
	digitalWrite(STEPPER_STEP_PIN, HIGH);
	digitalWrite(STEPPER_STEP_PIN, LOW);

	stepCount++;

	if (stepCount >= 3200)
		stepCount = 0;
}