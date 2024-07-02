/**
 * @file KincoMotor.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "drivers/KincoMotor.h"

KincoMotorClient::KincoMotorClient(const std::string &host, int port) {
    // connect client to an open server
	std::cout << "Host : " << host << " & Port : " << port << std::endl;
    for (int i=0; i<3; i++) {
        pipe_ret_t connectRet = connectTo(host, port);
        _connected = connectRet.isSuccessful();
        if (_connected) {
            std::cout << "KincoMotorClient connected successfully\n";
            break;
        } else {
            std::cout << "KincoMotorClient failed to connect: " << connectRet.message() << "\n"
                      << "Make sure the server is open and listening\n\n";
            sleep(2);
            std::cout << "Retrying to connect...\n";
        }
	}
}

/**
 * @brief Construct a new Kinco Motor Client:: Kinco Motor Client object
 * 
 */
KincoMotorClient::KincoMotorClient() {
}

/**
 * @brief Destroy the Kinco Motor Client:: Kinco Motor Client object
 * 
 */
KincoMotorClient::~KincoMotorClient() {
    std::cout << "KincoMotorClient Disconnected\n";
    close();
}

void KincoMotorClient::initAll() {
	for (int i=0; i<motors.size(); i++) {
		setDeviceState(motors[i].nodeId, OPERATIONAL);
		setMode(motors[i].nodeId, SPEED_CTRL);
	}
}

void KincoMotorClient::stopAll() {
	for (int i=0; i<motors.size(); i++) {
		setMode(motors[i].nodeId, MOTOR_OFF);
		setDeviceState(motors[i].nodeId, PRE_OPERATIONAL);
	}
}

void KincoMotorClient::resetErrors() {
	for (int i=0; i<motors.size(); i++) {
		resetError(motors[i].nodeId);
	}
}

void KincoMotorClient::resetError(uint8_t nodeId) {
	uint8_t data[8];
	data[0] = ERROR_RESET;
	data[1] = 0;
	data[2] = POSITION_CTRL;
	sendCANPacket((0x200 | nodeId), 3, data);
}

/**
 * @brief 
 * 
 * @param id 
 * @param name 
 */
void KincoMotorClient::addMotor(int id, std::string name) {
	Motor m = {.nodeId=id, .name = name};
	motors.push_back(m);
}

/**
 * @brief 
 * 
 * @param id 
 */
void KincoMotorClient::addMotor(int id) {
	addMotor(id, "M" + id);
}

/**
 * @brief 
 * 
 * @param vel 
 */
void KincoMotorClient::setVelocities(std::vector<int32_t> vels) {
	for (int i=0; i<motors.size(); i++) {
		setVelocity(motors[i].nodeId, vels[i]);
	}
}

/**
 * @brief 
 * 
 * @param vel 
 */
void KincoMotorClient::setVelocitiesRPM(std::vector<float> vels) {
	for (int i=0; i<motors.size(); i++) {
		// std::cout << vels[i]*RPM_FACTOR << std::endl;
		setVelocity(motors[i].nodeId, vels[i]*RPM_FACTOR);
	}
}

/**
 * @brief 
 * 
 * @param modes 
 */
void KincoMotorClient::setModes(std::vector<OpMode> modes) {
	for (int i=0; i<motors.size(); i++) {
		setMode(motors[i].nodeId, modes[i]);
	}
}

/**
 * @brief 
 * 
 * @param modes 
 */
void KincoMotorClient::setDeviceStates(std::vector<CanMode> modes) {
	for (int i=0; i<motors.size(); i++) {
		setDeviceState(motors[i].nodeId, modes[i]);
	}
}

/**
 * @brief 
 * 
 * @return std::vector<int32_t> 
 */
std::vector<int32_t> KincoMotorClient::getEncodersData(){
    std::lock_guard<std::mutex> lock(mMutex);
	std::vector<int32_t> ticks;
	for (int i=0; i<motors.size(); i++) {
		ticks.push_back(motors[i].position);
	}
	// std::cout << "left Encoder Data: >" << ticks[0] << std::endl;
	// std::cout << "Right Endoder Data: >" << ticks[1] << std::endl;
	return ticks;
}

std::vector<float> KincoMotorClient::getRPMData(){
    std::lock_guard<std::mutex> lock(mMutex);
	std::vector<float> rpms;
	for (int i=0; i<motors.size(); i++) {
		rpms.push_back(motors[i].speed / RPM_FACTOR);
	}
	return rpms;
}

/**
 * @brief 
 * 
 * @param msg 
 * @param msgSize 
 */
void KincoMotorClient::receiveMsg(const char *msg, size_t msgSize){
	// std::cout << "Received: " << msgSize << " data: "; // << int(msg[0]) << std::endl;
	// for (int i=0;i<msgSize;i++) {
	// 	std::cout << (int)uint8_t(msg[i]) << ", ";
	// }
	// std::cout << std::endl;

	int canSize = 13; // dlc,ide,rtr,len(2),data(8)
	if (msgSize % canSize == 0) {
		// uint16_t cobId = uint8_t(msg[3]) << 8 | uint8_t(msg[4]);
		int numPackets = msgSize / canSize;
		// std::cout << "NumPackets " << numPackets << std::endl;
		// std::cout << "COBID: " << int(uint16_t(cobId)) << std::endl;
		for (int i=0;i<numPackets;i++) {
			COReceive((uint8_t*)msg+(canSize*i), canSize);
		}
	}
}

/**
 * @brief 
 * 
 * @param data 
 * @param len 
 */
void KincoMotorClient::COReceive(const uint8_t * data, uint8_t len) {
    std::lock_guard<std::mutex> lock(mMutex);
	uint8_t dlc = data[0];
	uint8_t std = data[1];
	uint8_t rtr = data[2];
	uint32_t cobId = uint8_t(data[3]) << 8 | uint8_t(data[4]);
	// std::cout << int32_t(cobId) << std::endl;

	uint32_t nodeId = cobId & 0x7F;
	uint32_t pdoId = cobId - nodeId;
	switch (pdoId) {
	case 0x180: {		// TPDO1
		int32_t motorSpd = data[8] << 24 | data[7] << 16 | data[6] << 8 | data[5];
		int32_t motorPos = data[12] << 24 | data[11] << 16 | data[10] << 8 | data[9];
		float motorSpdF = motorSpd / 2730.666667;
		posSpeedFB(nodeId, motorPos, motorSpd);
		// std::cout << "Speed: " << motorSpdF << " Pos: " << motorPos << std::endl;
		break;
	}
	case 0x280:	{	// TPDO2
		int16_t motorCur = data[6] << 8 | data[5];
		float motorCurF = motorCur / 36.2038;
		currentFB(nodeId, motorCur);
		// std::cout << "Current: " << motorCurF << std::endl;
		break;
	}
	case 0x700:
		break;
	default:
		break;
	}
}

void KincoMotorClient::posSpeedFB(uint8_t nodeId, int32_t motorPos, int32_t motorSpd) {
	for (int i=0; i<motors.size(); i++) {
		if (motors[i].nodeId == nodeId) {
			motors[i].position = motorPos;
			motors[i].speed = motorSpd;
		}
	}
}
    

void KincoMotorClient::currentFB(uint8_t nodeId, int16_t motorCur) {
	for (int i=0; i<motors.size(); i++) {
		if (motors[i].nodeId == nodeId) {
			motors[i].current = motorCur;
		}
	}
}

/**
 * @brief 
 * 
 * @param id 
 * @param dlc 
 * @param TxData 
 * @return true 
 * @return false 
 */
bool KincoMotorClient::sendCANPacket(uint32_t id, uint8_t dlc, uint8_t *TxData) {
	char data[13];
	data[0] = dlc;
	data[1] = 0;
	data[2] = 0;
	data[3] = id >> 8;
	data[4] = id;
	for (int i=0; i<dlc ; i++){
		data[5+i] = TxData[i];
	}
	sendMsg(data, 13);
	return true;
}

/**
 * @brief 
 * 
 * @param nodeId 
 * @param mode 
 */
void KincoMotorClient::setMode(uint8_t nodeId, OpMode mode) {
	if (mode == MOTOR_OFF) {
		setMotorOff(nodeId);
		return;
	}

	uint8_t data[8];
	data[0] = POWER_ON;
	data[1] = 0;
	data[2] = mode;
	sendCANPacket((0x200 | nodeId), 3, data);
}

/**
 * @brief 
 * 
 * @param nodeId 
 */
void KincoMotorClient::setMotorOff(uint8_t nodeId) {
	uint8_t data[8];
	data[0] = POWER_OFF;
	data[1] = 0;
	data[2] = POSITION_CTRL;
	sendCANPacket((0x200 | nodeId), 3, data);
}

/**
 * @brief 
 * 
 * @param nodeId 
 * @param vel 
 */
void KincoMotorClient::setVelocity(uint8_t nodeId, int32_t vel) {
	sendCANPacket((0x300 | nodeId), 4, (uint8_t*) &vel);
}

/**
 * @brief 
 * 
 * @param nodeId 
 * @param state 
 * @return true 
 * @return false 
 */
bool KincoMotorClient::setDeviceState(uint8_t nodeId, CanMode state) {
	uint8_t data[8];
	data[0] = state;
	data[1] = nodeId;
	return sendCANPacket(0, 2, (uint8_t*) &data);
}

float KincoMotorClient::getEncoderResolution() {
	return encoderResolution;
}