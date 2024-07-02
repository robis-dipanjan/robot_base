
/**
 * @file DeviceManager.cpp
 * @author your name (dipanjan.maji@motherson.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-27
 */

#include <DeviceManager.h>

DeviceManager::DeviceManager() {
    motorDriver = std::make_shared<KincoMotorClient>("192.168.0.7", 20001);

    motorDriver->addMotor(0x06, "LEFT");

    motorDriver->addMotor(0x07, "RIGHT");

    diffDrive = std::make_shared<DifferentialDrive>(&motorDriver, 0.40, 0.20, 1.0);

}

DeviceManager::~DeviceManager() {
    
}