#include <iostream>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#define P_MIN -12.5
#define P_MAX 12.5
#define V_MIN -65.0
#define V_MAX 65.0
#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0
#define T_MIN -18.0
#define T_MAX 18.0

#define BITS_POSITION 16
#define BITS_VELOCITY 12
#define BITS_KP 12
#define BITS_KD 12
#define BITS_TORQUE 12

class canbus {
public: 
    bool sendCustomMessage(int socket, const std::string& ifname, std::vector<uint8_t> data, uint32_t id);

    int pullMessage(uint32_t id, std::vector<can_frame>& rxFrames);

    uint doubleToUint(double x, double xMin, double xMax, int bits);

    double uintToDouble(uint32_t xInt, double xMin, double xMax, int bits);

    std::vector<uint8_t> createCtrlMessage(double position, double velocity, double torque, double kp, double kd);
 

    void unpackCanFrame(struct can_frame frame, std::vector<double>& listVal);
    struct can_frame packCanFrame(const std::vector<uint8_t> canMessage);

};