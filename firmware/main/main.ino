#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

struct StepperConfig {
    uint8_t enablePin;
    uint8_t stepPin;
    uint8_t directionPin;
    float degPerStep;
    int subdivision;
};


StepperConfig baseStepperConfig = {
    .enablePin = 23,
    .stepPin = 22,
    .directionPin = 1,
    .degPerStep = 0.9,
    .subdivision = 32
};

StepperConfig elbowStepperConfig = {
    .enablePin = 19,
    .stepPin = 18,
    .directionPin = 5,
    .degPerStep = 0.9,
    .subdivision = 32
};

AccelStepper baseStepper(AccelStepper::DRIVER, baseStepperConfig.stepPin, baseStepperConfig.directionPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, elbowStepperConfig.stepPin, elbowStepperConfig.directionPin);

MultiStepper steppers;

float baseStepperSpeed = 1000.0;
float baseStepperAcceleration = 1.0;

float elbowStepperSpeed = 1000.0;
float elbowStepperAcceleration = 1.0;
uint8_t targetIndex = 0;



float squareTargetsRadians[4][2] = {
    {0.88848543, 1.75941264},
    {0.49369505, 1.75941201},
    {0.08462752, 2.3288367},
    {0.72812876, 2.32883707}
};


long radians2steps(float radians, StepperConfig config) {
    float deg = radians * 180.0 / PI;
    return deg / config.degPerStep * config.subdivision;
}

long radiansToStepsBase(float radians, StepperConfig config) {
    radians -= PI / 2;
    return radians2steps(radians, config);
}

long radiansToStepsElbow(float radians, StepperConfig config) {
    radians = -radians;
    return radians2steps(radians, config);
}


void setup() {
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
}

void loop() {
    bool stillMoving = steppers.run();
    if (!stillMoving) {
        long targetSteps[] = {
            radiansToStepsBase(squareTargetsRadians[targetIndex][0], baseStepperConfig),
            radiansToStepsElbow(squareTargetsRadians[targetIndex][1], elbowStepperConfig)
        };
        Serial.println("Targets: ");
        Serial.println("Base: " + String(squareTargetsRadians[targetIndex][0]) + " deg, " + String(targetSteps[0]) + " steps");
        Serial.println("Elbow: " + String(squareTargetsRadians[targetIndex][1]) + " deg, " + String(targetSteps[1]) + " steps");
        Serial.println();
        Serial.flush();

        steppers.moveTo(targetSteps);

        targetIndex = (targetIndex + 1) % 4;
    }
}
