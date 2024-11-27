#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

#define MAX_TARGETS 10000  // Define a maximum number of targets for safety

struct StepperConfig {
    uint8_t enablePin;
    uint8_t stepPin;
    uint8_t directionPin;
    float degPerStep;
    long subdivision;
};

StepperConfig baseStepperConfig = {.enablePin = 23,
                                   .stepPin = 22,
                                   .directionPin = 21,
                                   .degPerStep = 0.9,
                                   .subdivision = 32};

StepperConfig elbowStepperConfig = {.enablePin = 19,
                                    .stepPin = 18,
                                    .directionPin = 5,
                                    .degPerStep = 0.9,
                                    .subdivision = 32};

AccelStepper baseStepper(AccelStepper::DRIVER, baseStepperConfig.stepPin,
                         baseStepperConfig.directionPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, elbowStepperConfig.stepPin,
                          elbowStepperConfig.directionPin);

MultiStepper steppers;

float baseStepperSpeed = 300.0;
float baseStepperAcceleration = 100.0;

float elbowStepperSpeed = 300.0;
float elbowStepperAcceleration = 100.0;
uint8_t targetIndex = 0;

int numTargets;
float targetBuffer[MAX_TARGETS][2];

long radians2steps(float radians, StepperConfig config) {
    float deg = radians * 180.0 / PI;
    return (long) ((deg / config.degPerStep) * config.subdivision);
}

long radiansToStepsBase(float radians, StepperConfig config) {
    radians -= PI / 2;
    return radians2steps(radians, config);
}

long radiansToStepsElbow(float radians, StepperConfig config) {
    radians = -radians;
    return radians2steps(radians, config);
}

int receiveTargets(float targets[MAX_TARGETS][2]) {
    // Check if targets pointer is valid
    if (!targets) return -1;

    Serial.println("Receiving targets");

    // wait until data is available
    while (Serial.available() == 0) {
        delay(100);
    }

    int itemIndex = 0;
    int coordinateIndex = 0;

    while (itemIndex < MAX_TARGETS) {  // Prevent buffer overflow
        if (!Serial.available()) {
            delay(10);  // Small delay if no data available
            continue;
        }


        String line = Serial.readStringUntil('\n');
        line.trim();  // Remove whitespace

        // Check for end of transmission or empty line
        if (line.length() == 0) {
            break;
        }

        // Parse float and check for validity
        float value = line.toFloat();
        if (value == 0 && line.charAt(0) != '0') {  // Basic error check
            Serial.println("Parse error");
            return -3;  // Parse error
        }

        targets[itemIndex][coordinateIndex] = value;

        coordinateIndex++;
        if (coordinateIndex == 2) {
            itemIndex++;
            coordinateIndex = 0;
        }
    }
    Serial.println("Received " + String(itemIndex) + " targets");

    return itemIndex;
}

void setup() {
    // pinMode(baseStepperConfig.enablePin, OUTPUT);
    // pinMode(elbowStepperConfig.enablePin, OUTPUT);
    // pinMode(baseStepperConfig.stepPin, OUTPUT);
    // pinMode(elbowStepperConfig.stepPin, OUTPUT);
    // pinMode(baseStepperConfig.directionPin, OUTPUT);
    // pinMode(elbowStepperConfig.directionPin, OUTPUT);
    Serial.begin(115200);
    baseStepper.setEnablePin(baseStepperConfig.enablePin);
    baseStepper.enableOutputs();
    baseStepper.setMaxSpeed(baseStepperSpeed);
    baseStepper.setAcceleration(baseStepperAcceleration);
    digitalWrite(baseStepperConfig.enablePin, HIGH);

    elbowStepper.setEnablePin(elbowStepperConfig.enablePin);
    elbowStepper.enableOutputs();
    elbowStepper.setMaxSpeed(elbowStepperSpeed);
    elbowStepper.setAcceleration(elbowStepperAcceleration);
    digitalWrite(elbowStepperConfig.enablePin, HIGH);

    steppers.addStepper(baseStepper);
    steppers.addStepper(elbowStepper);

    while (!Serial);
    Serial.println("Serial initialized");

    numTargets = receiveTargets(targetBuffer);

}

void loop() {
    // receive targets from serial

    bool stillMoving = steppers.run();
    if (!stillMoving) {
        long targetSteps[] = {
            radiansToStepsBase(targetBuffer[targetIndex][0], baseStepperConfig),
            radiansToStepsElbow(targetBuffer[targetIndex][1],
                                elbowStepperConfig)};
        Serial.println("Targets: ");
        Serial.println("Base: " + String(targetBuffer[targetIndex][0]) +
                       " deg, " + String(targetSteps[0]) + " steps");
        Serial.println("Elbow: " + String(targetBuffer[targetIndex][1]) +
                       " deg, " + String(targetSteps[1]) + " steps");
        Serial.println();
        Serial.flush();

        steppers.moveTo(targetSteps);

        targetIndex = (targetIndex + 1) % numTargets;
    }
}
