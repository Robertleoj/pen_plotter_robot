#include "MKS_SERVO42.h"

void MKS_SERVO42::initialize(HardwareSerial *serialPort, byte stepperId,
                             FirmwareVersion const &version) {
    this->version = version;
    this->port = serialPort;
    this->stepperId = padStepperId(stepperId);
}

void MKS_SERVO42::flush() { while (port->read() != -1); }

bool MKS_SERVO42::ping() {
    flush();

    bool success = sendMessage(Instruction::STEPPER_PING);

    if (!success) {
        Serial.println("Failed to send");
        return false;
    }

    return receiveStepperStatus();
}

byte MKS_SERVO42::padStepperId(byte const &stepperId) {
    return stepperId + 0xE0;
}

bool MKS_SERVO42::sendMessage(byte const &commandId) {
    int messageSize;

    switch (version) {
        case FirmwareVersion::V1_0:
            messageSize = 2;
            break;

        case FirmwareVersion::V1_1:
            messageSize = 3;
            break;
    }

    byte message[messageSize];
    byte checksum = stepperId + commandId;

    message[0] = stepperId;
    message[1] = commandId;

    if (version == FirmwareVersion::V1_1) {
        message[2] = checksum & 0xFF;
    }

    int numSent = port->write(message, messageSize);
    return numSent == messageSize;
}

int MKS_SERVO42::receiveStepperStatus() {
    int messageSize;

    switch (version) {
        case FirmwareVersion::V1_0:
            messageSize = 2;
            break;

        case FirmwareVersion::V1_1:
            messageSize = 3 + sizeof(uint8_t);
            break;
    }

    byte receivedBytes[messageSize];

    size_t rd = port->readBytes(receivedBytes, messageSize);

    Serial.print("Received bytes: " + String(rd));
    for (size_t i = 0; i < rd; i++) {
        Serial.print(" ");
        Serial.print(receivedBytes[i], HEX);
    }
    Serial.println();

    return receivedBytes[1] == 1;
}

long MKS_SERVO42::getCurrentPosition() {
    // Flush
    flush();

    bool success = sendMessage(Instruction::GET_ENCODER_POS);

    if (!success) {
        Serial.println("Failed to send");
        return -1;
    }

    return receiveEncoderPosition();
}

long MKS_SERVO42::receiveEncoderPosition() {
    uint8_t receivedBytes[4];
    int toRead;

    if (version == FirmwareVersion::V1_1) {
        toRead = 4;
    } else if (version == FirmwareVersion::V1_0) {
        toRead = 2;
    } else {
        Serial.println("Unknown firmware version");
        return -1;
    }

    size_t bytesRead = port->readBytes(receivedBytes, toRead);

    if (bytesRead == toRead && receivedBytes[0] == stepperId) {
        int value = (receivedBytes[1] << 8) | receivedBytes[2];
        return value;
    } else {
        Serial.println("Invalid response from motor controller");
        return -1;
    }
}

/**
 * @param speed if using v1.1, speed is in rpm, between 0 and 2000.
 *             if using v1.0, speed is between 0 and 0x7f.
 */
bool MKS_SERVO42::setTargetPosition(byte direction, uint8_t speed,
                                    uint32_t pulses) {
    if (version == FirmwareVersion::V1_1) {
        if (speed > 2000 || direction > 1) {
            Serial.println("Speed out of range or invalid directionection");
            return false;
        }

        byte message[8];
        message[0] = stepperId;                       // Slave address
        message[1] = Instruction::MOVE_SPEED_PULSES;  // Function code for
                                                      // running the motor
        message[2] =
            (direction << 7) |
            (speed & 0x7F);  // VAL byte, with directionection and speed
        message[3] = (pulses >> 24) & 0xFF;
        message[4] = (pulses >> 16) & 0xFF;
        message[5] = (pulses >> 8) & 0xFF;
        message[6] = pulses & 0xFF;
        message[7] = calculateChecksum(message, 7);
        port->write(message, sizeof(message));

        while (port->read() != -1);
        byte response[3];
        size_t bytesRead = port->readBytes(response, 3);

        Serial.println("Bytes read: " + String(bytesRead));
        Serial.print("Response: ");
        for (size_t i = 0; i < bytesRead; i++) {
            Serial.print(response[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        if (bytesRead == 3 && response[0] == stepperId) {
            if (response[1] == 0) {
                Serial.println("Run command failed");
                return false;
            }
            return true;
        } else {
            Serial.println("Invalid response from motor controller");
            return false;
        }
    } else if (version == FirmwareVersion::V1_0) {
        if (speed > 0x7f || direction > 1) {
            Serial.println("Speed out of range or invalid directionection");
            return false;
        }

        byte message[5];
        message[0] = stepperId;                       // Slave address
        message[1] = Instruction::MOVE_SPEED_PULSES;  // Function code for
                                                      // running the motor
        message[2] =
            (direction << 7) |
            (speed & 0x7F);  // VAL byte, with directionection and speed
        message[3] = (pulses >> 8) & 0xFF;
        message[4] = pulses & 0xFF;
        port->write(message, sizeof(message));

        while (port->read() != -1);

        byte response[2];
        size_t bytesRead = port->readBytes(response, 2);

        Serial.println("Bytes read: " + String(bytesRead));
        Serial.print("Response: ");
        for (size_t i = 0; i < bytesRead; i++) {
            Serial.print(response[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        if (bytesRead == 2 && response[0] == stepperId) {
            if (response[1] == 0) {
                Serial.println("Run command failed");
                return false;
            }
            return true;
        } else {
            Serial.println("Invalid response from motor controller");
            return false;
        }
    }
}

byte MKS_SERVO42::calculateChecksum(const byte *message, int length) {
    if (version == FirmwareVersion::V1_0) {
        Serial.println("V1.0 does not use checksum");
        return 0;
    }

    byte checksum = 0;
    for (int i = 0; i < length; i++) checksum += message[i];
    return checksum & 0xFF;
}

bool MKS_SERVO42::setZeroMode(ZeroMode mode) {
    if (version == FirmwareVersion::V1_0) {
        Serial.println("V1.0 does not support setting zero mode");
        return false;
    }

    uint8_t modeByte = static_cast<uint8_t>(mode);
    Serial.print("Setting zero mode to ");
    Serial.println(modeByte, HEX);

    uint8_t message[4] = {stepperId, Instruction::SET_ZERO_MODE,
                          static_cast<uint8_t>(mode), 0};
    message[3] = calculateChecksum(message, 3);
    port->write(message, sizeof(message));

    uint8_t response[3];
    size_t bytesRead = port->readBytes(response, 3);

    Serial.println("Bytes read: " + String(bytesRead));
    Serial.print("Response: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (bytesRead != 3 || response[0] != stepperId) {
        Serial.println("Invalid response from motor controller");
        return false;
    }

    return true;
}

bool MKS_SERVO42::setZeroPosition() {
    if (version == FirmwareVersion::V1_0) {
        Serial.println("V1.0 does not support setting zero position");
        return false;
    }

    setZeroMode(ZeroMode::NEARMODE);
    Serial.println("Set zero mode to near mode");

    delay(2000);

    Serial.println("Setting zero position");

    uint8_t message[4] = {stepperId, Instruction::SET_ZERO_POS, 0, 0};
    message[3] = calculateChecksum(message, 3);
    port->write(message, sizeof(message));

    delay(1000);

    uint8_t response[3];
    size_t bytesRead = port->readBytes(response, 3);

    Serial.println("Bytes read: " + String(bytesRead));
    Serial.print("Response: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (bytesRead != 3 || response[0] != stepperId) {
        Serial.println("Invalid response from motor controller");
        return false;
    }

    return true;
}

bool MKS_SERVO42::setZeroSpeed(uint8_t speed) {
    if (version == FirmwareVersion::V1_0) {
        Serial.println("V1.0 does not support setting zero speed");
        return false;
    }

    uint8_t message[4] = {stepperId, Instruction::SET_ZERO_SPEED, speed, 0};
    message[3] = calculateChecksum(message, 3);
    port->write(message, sizeof(message));

    flush();

    uint8_t response[3];
    size_t bytesRead = port->readBytes(response, 3);

    Serial.println("Bytes read: " + String(bytesRead));
    Serial.print("Response: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (bytesRead != 3 || response[0] != stepperId) {
        Serial.println("Invalid response from motor controller");
        return false;
    }

    return true;
}

bool MKS_SERVO42::setSpeedDirection(uint8_t speed, byte direction) {
    int messageSize;
    int responseSize;
    if (version == FirmwareVersion::V1_0) {
        messageSize = 3;
        responseSize = 2;
    } else if (version == FirmwareVersion::V1_1) {
        messageSize = 4;
        responseSize = 3;
    } else {
        Serial.println("Unknown firmware version");
        return false;
    }

    byte message[messageSize];
    message[0] = stepperId;
    message[1] = Instruction::SET_SPEED_DIRECTION;
    message[2] = direction << 7 | speed;

    if (version == FirmwareVersion::V1_1) {
        message[3] = calculateChecksum(message, 3);
    }

    port->write(message, messageSize);

    byte response[responseSize];
    size_t bytesRead = port->readBytes(response, responseSize);

    Serial.println("Bytes read: " + String(bytesRead));
    Serial.print("Response: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (bytesRead != responseSize || response[0] != stepperId) {
        Serial.println("Invalid response from motor controller");
        return false;
    }

    if (response[1] == 0) {
        Serial.println("Set speed direction failed");
        return false;
    }

    return true;
}

bool MKS_SERVO42::setSubdivision(uint8_t subdivision) {
    if (version == FirmwareVersion::V1_0) {
        Serial.println("V1.0 does not support setting subdivision");
        return false;
    }

    uint8_t message[4] = {stepperId, Instruction::SET_SUBDIVISION, subdivision, 0};
    message[3] = calculateChecksum(message, 3);
    port->write(message, sizeof(message));

    uint8_t response[3];
    size_t bytesRead = port->readBytes(response, 3);

    Serial.println("Bytes read: " + String(bytesRead));
    Serial.print("Response: ");
    for (size_t i = 0; i < bytesRead; i++) {
        Serial.print(response[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    if (bytesRead != 3 || response[0] != stepperId) {
        Serial.println("Invalid response from motor controller");
        return false;
    }

    if (response[1] == 0) {
        Serial.println("Set subdivision failed");
        return false;
    }

    return true;
}
