/**
 * @file IMU.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-11-30
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "drivers/IMU.h"

IMUClient::IMUClient(const std::string &host, int port)
{
    for (int i = 0; i < 3; i++)
    {
        pipe_ret_t connectRet = connectTo(host, port);
        _connected = connectRet.isSuccessful();
        if (_connected)
        {
            std::cout << "IMUClient connected successfully\n";
            break;
        }
        else
        {
            std::cout << "IMUClient failed to connect: " << connectRet.message() << "\n"
                      << "Make sure the server is open and listening\n\n";
            sleep(2);
            std::cout << "Retrying to connect...\n";
        }
    }
}

IMUClient::IMUClient()
{
}

IMUClient::~IMUClient()
{
    std::cout << "IMUClient Disconnected\n";
    close();
}

float IMUClient::toFloat(uint8_t *data)
{
    uint8_t lData[4];
    // big endian to little endian
    for (int i = 0; i < 4; i++)
    {
        lData[i] = data[3 - i];
    }
    // cast as float
    return *(float *)lData;
}

void IMUClient::parseType(uint8_t *data, uint8_t *index)
{
    std::lock_guard<std::mutex> lock(mMutex);
    int dType = data[0] << 8 | data[1];
    int dLen = data[2];
    *index += 3 + dLen;
    imuData.time = std::chrono::system_clock::now();

    switch (dType)
    {
    case PACKET_COUNTER:
        imuData.counter = data[3] << 8 | data[4];
        break;

    case SAMPLE_TIME_FINE:
        break;

    case QUATERNION:
        break;

    case EULER_ANGLE:
        imuData.orientation.roll = toFloat(data + 3);
        imuData.orientation.pitch = toFloat(data + 3 + 4);
        imuData.orientation.yaw = toFloat(data + 3 + 8);
        // std::cout << "Yaw: " << imuData.orientation.yaw
        //         << " Pitch: " << imuData.orientation.pitch
        //         << " Roll: " << imuData.orientation.roll
        //         << std::endl;
        break;

    case ACCELERATION:
        imuData.accel.x = toFloat(data + 3);
        imuData.accel.y = toFloat(data + 3 + 4);
        imuData.accel.z = toFloat(data + 3 + 8);
        break;

    case RATE_OF_TURN:
        imuData.gyro.x = toFloat(data + 3);
        imuData.gyro.y = toFloat(data + 3 + 4);
        imuData.gyro.z = toFloat(data + 3 + 8);
        break;

    case MAGNETIC_FIELD:
        imuData.mag.x = toFloat(data + 3);
        imuData.mag.y = toFloat(data + 3 + 4);
        imuData.mag.z = toFloat(data + 3 + 8);
        break;

    case STATUSWORD:
        imuData.status = data[3] << 8 | data[4];
        break;

    default:
        break;
    }
}

void IMUClient::processXsensData(uint8_t *data, uint8_t len)
{
    uint8_t numItems = 7;
    uint8_t index = 0;
    for (int i = 0; i < numItems; i++)
    {
        parseType(data + index, &index);
    }
}

IMUData IMUClient::getIMUData()
{
    std::lock_guard<std::mutex> lock(mMutex);
    return imuData;
}

void IMUClient::receiveMsg(const char *msg, size_t msgSize)
{
    // std::cout << "Received: " << msgSize << " data: " << int(msg[2]) << std::endl;
    if (uint8_t(msg[0]) == 0xFA and uint8_t(msg[1]) == 0xFF and uint8_t(msg[2]) == 0x36)
    {
        processXsensData((uint8_t *)msg + 4, msgSize);
    }
}