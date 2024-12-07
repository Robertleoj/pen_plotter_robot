#include <CircularBuffer.hpp>
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

const float speedWhenPendown = 1000.0;
const float speedWhenPenup = 2000.0;

uint8_t targetIndex = 0;

bool motorOn = false;

int splitString(String str, String* array, int maxSize, char delimiter) {
    int count = 0;
    int lastIndex = 0;
    int strLength = str.length();
    
    for (int i = 0; i <= strLength; i++) {
        if (i == strLength || str.charAt(i) == delimiter) {
            if (count >= maxSize) {
                break;
            }
            
            array[count] = str.substring(lastIndex, i);
            array[count].trim();
            count++;
            lastIndex = i + 1;
        }
    }
    
    return count;
}


void setSpeed(float speed) {
    baseStepper.setMaxSpeed(speed);
    elbowStepper.setMaxSpeed(speed);
}

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


void turnOnMotors() {
    digitalWrite(baseStepperConfig.enablePin, LOW);
    digitalWrite(elbowStepperConfig.enablePin, LOW);
    motorOn = true;
}

void turnOffMotors() {
    digitalWrite(baseStepperConfig.enablePin, HIGH);
    digitalWrite(elbowStepperConfig.enablePin, HIGH);
    motorOn = false;
}

void zero() {
    baseStepper.setCurrentPosition(0);
    elbowStepper.setCurrentPosition(0);
}

void getReady() {
    turnOnMotors();
    zero();
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
    setSpeed(speedWhenPenup);

    steppers.addStepper(baseStepper);
    steppers.addStepper(elbowStepper);

    turnOffMotors();
    penUp();

    while (!Serial);
    Serial.println("Serial initialized");

}

const int penUpAngle = 0;
const int penDownAngle = 90;


void penUp() {
    Serial.println("Pen up");
    for (int i = penDownAngle; i >= penUpAngle; i--) {
        servo.write(i);
        delay(1);
    }
    Serial.println("Pen up done");
    setSpeed(speedWhenPenup);
}

void penDown() {
    Serial.println("Pen down");
    for (int i = penUpAngle; i <= penDownAngle; i++) {
        servo.write(i);
        delay(1);
    }
    Serial.println("Pen down done");
    setSpeed(speedWhenPendown);
}


class InstructionRunner {
public:
    Instruction instruction;
    CircularBuffer<Instruction, MAX_TARGETS> instructionBuffer;

    bool noInstructions = true;
    long numRuns = 0;

    void run() {
        if (noInstructions) {
            readInstructions();
            return;
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

        numRuns++;
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
            noInstructions = false;
        } else {
            noInstructions = true;
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

    void readInstructions() {

        Serial.println("wfc");

        while (Serial.available() == 0) {
            delay(10);
        }

        // Serial.println("Reading instructions");
        String line = Serial.readStringUntil('\n');
        line.trim();
        Serial.println("Read line: " + line);

        // splitn the string on whitespace
        String values[3];
        int numValues = splitString(line, values, 3, ' ');
        String cmd = values[0];

        if (cmd == "PU") {
            instructionBuffer.push(Instruction{InstructionType::PEN_UP});
        } else if (cmd == "PD") {
            instructionBuffer.push(Instruction{InstructionType::PEN_DOWN});
        } else if (cmd == "M") {
            instructionBuffer.push(Instruction{InstructionType::MOVE_TO, {values[1].toFloat(), values[2].toFloat()}});
        } else if (cmd == "H") {
            instructionBuffer.push(Instruction{InstructionType::HOME});
        }

        noInstructions = false;
    }
};

InstructionRunner instructionRunner;

void mainLoop() {
    if (millis() % 50 == 0) {

        bool wasPressed = isPressed;

        isPressed = digitalRead(buttonPin) == LOW;

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
        instructionRunner.run();
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
    mainLoop();
}