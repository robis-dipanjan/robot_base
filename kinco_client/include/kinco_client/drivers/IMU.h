/**
 * @file IMU.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <string>

#include "drivers/external/tcp_client.h"
// #include "ringBuffer.h"
#include "DataStructures.h"

enum {
    PACKET_COUNTER=0x1020,
    SAMPLE_TIME_FINE=0x1060,
    QUATERNION=0x2010, 
    EULER_ANGLE=0x2030,
    ACCELERATION=0x4020,
    RATE_OF_TURN=0x8020,
    MAGNETIC_FIELD=0xc020,
    STATUSWORD=0xe020,
};

// typedef struct pos3D
// {
//     float x;
//     float y;
//     float z;
// } pos3D;

// typedef struct Euler
// {
//     float roll;
//     float pitch;
//     float yaw;
// } Euler;

// typedef struct IMUData
// {
//     int counter;
//     int status;
//     Euler orientation;
//     pos3D accel;
//     pos3D gyro;
//     pos3D mag;
// } IMUData;


/// @brief 
class IMUClient : public TcpClient {
public:
    IMUClient(const std::string &host, int port);
    IMUClient();
    ~IMUClient();

private:
    std::string _host;
    int _port;
    bool _connected;
    IMUData imuData;
    bool once = false;
    std::mutex mMutex;
    // Ringbuffer<uint8_t*, 256> dataBuffer;

    float toFloat(uint8_t *data);
    void parseType(uint8_t *data, uint8_t *index);
    void processXsensData(uint8_t *data, uint8_t len);

public:
    IMUData getIMUData();
    void receiveMsg(const char * msg, size_t msgSize) override;
};