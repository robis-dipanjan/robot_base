/**
 * @file DeviceManager.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "drivers/KincoMotor.h"
#include "drivers/DifferentialDrive.h"
#include "drivers/external/tcp_client.h"

class DeviceManager {
    public:
        DeviceManager();
        ~DeviceManager();

    private:
        std::string _name;

    public:
        std::shared_ptr<KincoMotorClient> motorDriver;
        std::shared_ptr<DifferentialDrive> diffDrive;
    };