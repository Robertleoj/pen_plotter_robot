#include <CircularBuffer.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

#define MAX_TARGETS 50  // Define a maximum number of targets for safety

enum class InstructionType {
    PEN_UP,
    PEN_DOWN,
    MOVE_TO,
    HOME
};

struct MoveToInstruction {
    float baseAngle;
    float elbowAngle;
};

struct Instruction {
    InstructionType type;
    union {
        MoveToInstruction moveTo;
    } data;
};

struct StepperConfig {
    uint8_t enablePin;
    uint8_t stepPin;
    uint8_t directionPin;
    float degPerStep;
    long subdivision;
};

StepperConfig baseStepperConfig = {.enablePin = 13,
                                   .stepPin = 12,
                                   .directionPin = 11,
                                   .degPerStep = 1.8,
                                   .subdivision = 256};

StepperConfig elbowStepperConfig = {.enablePin = 10,
                                    .stepPin = 9,
                                    .directionPin = 8,
                                    .degPerStep = 1.8,
                                    .subdivision = 256};

AccelStepper baseStepper(AccelStepper::DRIVER, baseStepperConfig.stepPin,
                         baseStepperConfig.directionPin);
AccelStepper elbowStepper(AccelStepper::DRIVER, elbowStepperConfig.stepPin,
                          elbowStepperConfig.directionPin);

const int buttonPin = 7;
bool isPressed = false;

const int servoPin = 3;
Servo servo;

MultiStepper steppers;

float baseStepperSpeed = 1500.0;

float elbowStepperSpeed = 1500.0;

uint8_t targetIndex = 0;

bool motorOn = false;

CircularBuffer<Instruction, MAX_TARGETS> instructionBuffer;

long radians2steps(float radians, StepperConfig config) {
    float deg = radians * 180.0 / PI;
    return (long)((deg / config.degPerStep) * config.subdivision);
}

long radiansToStepsBase(float radians, StepperConfig config) {
    radians -= PI / 2;
    return radians2steps(radians, config);
}

long radiansToStepsElbow(float radians, StepperConfig config) {
    radians = -radians;
    return radians2steps(radians, config);
}

int receiveInstructions() {
    Serial.println("Receiving instructions");

    instructionBuffer.push(Instruction{InstructionType::PEN_UP});

    // wait until data is available
    while (Serial.available() == 0) {
        delay(100);
    }

    int coordinateIndex = 0;

    int numReceived = 0;

    float coordinates[2];

    while (numReceived < MAX_TARGETS) {  // Prevent buffer overflow
        if (!Serial.available()) {
            delay(10);  // Small delay if no data available
            continue;
        }

        // delay(10);

        String line = Serial.readStringUntil('\n');
        line.trim();  // Remove whitespace

        Serial.println("line:" + line + " Index:" + String(numReceived));

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

        coordinates[coordinateIndex] = value;

        coordinateIndex++;

        if (coordinateIndex == 2) {
            instructionBuffer.push(Instruction{InstructionType::MOVE_TO, {coordinates[0], coordinates[1]}});
            if (numReceived == 1) {
                instructionBuffer.push(Instruction{InstructionType::PEN_DOWN});
            }
            coordinateIndex = 0;
            numReceived ++;
        }

    }
    Serial.println("Received " + String(numReceived) + " instructions");

    instructionBuffer.push(Instruction{InstructionType::PEN_UP});
    instructionBuffer.push(Instruction{InstructionType::HOME});

    return numReceived;
}

void turnOnMotors() {
    digitalWrite(baseStepperConfig.enablePin, LOW);
    digitalWrite(elbowStepperConfig.enablePin, LOW);
    // baseStepper.enableOutputs();
    // elbowStepper.enableOutputs();
    motorOn = true;
}

void turnOffMotors() {
    digitalWrite(baseStepperConfig.enablePin, HIGH);
    digitalWrite(elbowStepperConfig.enablePin, HIGH);
    // baseStepper.disableOutputs();
    // elbowStepper.disableOutputs();
    motorOn = false;
}

void zero() {
    baseStepper.setCurrentPosition(0);
    elbowStepper.setCurrentPosition(0);
}

void getReady() {
    turnOnMotors();
    zero();
    receiveInstructions();
}

void setup() {
    Serial.begin(115200);
    servo.attach(servoPin);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(baseStepperConfig.enablePin, OUTPUT);
    pinMode(elbowStepperConfig.enablePin, OUTPUT);
    pinMode(baseStepperConfig.stepPin, OUTPUT);
    pinMode(elbowStepperConfig.stepPin, OUTPUT);
    pinMode(baseStepperConfig.directionPin, OUTPUT);
    pinMode(elbowStepperConfig.directionPin, OUTPUT);
    // baseStepper.setEnablePin(baseStepperConfig.enablePin);
    // baseStepper.enableOutputs();
    // digitalWrite(baseStepperConfig.enablePin, LOW);
    baseStepper.setMaxSpeed(baseStepperSpeed);

    // elbowStepper.setEnablePin(elbowStepperConfig.enablePin);
    // digitalWrite(elbowStepperConfig.enablePin, LOW);
    // elbowStepper.enableOutputs();
    elbowStepper.setMaxSpeed(elbowStepperSpeed);

    steppers.addStepper(baseStepper);
    steppers.addStepper(elbowStepper);

    turnOffMotors();
    penUp();

    while (!Serial);
    Serial.println("Serial initialized");

}

// void runDrawing() {
//     bool stillMoving = steppers.run();
//     if (!stillMoving) {
//         long targetSteps[] = {
//             radiansToStepsBase(targetBuffer[targetIndex][0], baseStepperConfig),
//             radiansToStepsElbow(targetBuffer[targetIndex][1],
//                                 elbowStepperConfig)};
//         Serial.println("Targets: ");
//         Serial.println("Base: " + String(targetBuffer[targetIndex][0]) +
//                        " deg, " + String(targetSteps[0]) + " steps");
//         Serial.println("Elbow: " + String(targetBuffer[targetIndex][1]) +
//                        " deg, " + String(targetSteps[1]) + " steps");
//         Serial.println();
//         Serial.flush();

//         steppers.moveTo(targetSteps);

//         targetIndex = (targetIndex + 1) % numTargets;
//     }
// }

const int penUpAngle = 0;
const int penDownAngle = 90;


void penUp() {
    Serial.println("Pen up");
    for (int i = penDownAngle; i >= penUpAngle; i--) {
        servo.write(i);
        delay(1);
    }
    Serial.println("Pen up done");
}

void penDown() {
    Serial.println("Pen down");
    for (int i = penUpAngle; i <= penDownAngle; i++) {
        servo.write(i);
        delay(1);
    }
    Serial.println("Pen down done");
}


class InstructionRunner {
public:
    Instruction instruction;
    bool isDone = false;

    bool run() {
        if (isDone) {
            return true;
        }

        switch (instruction.type) {
            case InstructionType::PEN_UP:
                penUp();
                fetchNextInstruction();
                break;

            case InstructionType::PEN_DOWN:
                penDown();
                fetchNextInstruction();
                break;

            case InstructionType::MOVE_TO:
                runTarget();
                break;

            case InstructionType::HOME:
                runTarget();
                break;
        }

        return false;
    }

    void runTarget() {
        bool stillMoving = steppers.run();
        if (!stillMoving) {
            fetchNextInstruction();
        }
    }

    void fetchNextInstruction() {
        if (instructionBuffer.size() > 0) {
            instruction = instructionBuffer.shift();
            Serial.print("Fetched instruction: ");
            switch (instruction.type) {
                case InstructionType::MOVE_TO:
                    Serial.println("Move to " + String(instruction.data.moveTo.baseAngle) + ", " + String(instruction.data.moveTo.elbowAngle));
                    break;
                case InstructionType::HOME:
                    Serial.println("Home");
                    break;
                case InstructionType::PEN_UP:
                    Serial.println("Pen up");
                    break;
                case InstructionType::PEN_DOWN:
                    Serial.println("Pen down");
                    break;
            }
            initInstruction();
        } else {
            isDone = true;
        }

    }

    void initInstruction() {
        switch (instruction.type) {
            case InstructionType::MOVE_TO:
                setTarget(instruction.data.moveTo.baseAngle,
                          instruction.data.moveTo.elbowAngle);
                break;

            case InstructionType::HOME:
                penUp();
                setTarget(PI / 2, 0);
                break;
        }
    }

    void setTarget(float baseAngle, float elbowAngle) {
        Serial.println("Setting target: " + String(baseAngle) + ", " + String(elbowAngle));
        long targetSteps[] = {
            radiansToStepsBase(baseAngle, baseStepperConfig),
            radiansToStepsElbow(elbowAngle, elbowStepperConfig)};

        steppers.moveTo(targetSteps);
    }
};

InstructionRunner instructionRunner;

void mainLoop() {
    if (millis() % 50 == 0) {
        Serial.println(">button_pressed:" + String(isPressed));

        bool wasPressed = isPressed;

        isPressed = digitalRead(buttonPin) == LOW;

        // Serial.println(">button_pressed:" + String(isPressed) +
        //                " wasPressed:" + String(wasPressed));

        if (isPressed && !wasPressed) {
            if (motorOn) {
                turnOffMotors();
                motorOn = false;
            } else {
                getReady();
            }
        }
    }

    if (motorOn) {
        bool finished = instructionRunner.run();
        if (finished) {
            turnOffMotors();
            motorOn = false;
        }
    }
}

void testServoLoop() {
    int delayTime = 10000;
    Serial.println("Pen up");
    penUp();
    delay(delayTime);

    Serial.println("Pen down");
    penDown();
    delay(delayTime);
}

void loop() {
    // receive targets from serial
    // testServoLoop();
    mainLoop();
}