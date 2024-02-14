#include "canbus.h"


bool canbus::sendCustomMessage(int socket, const std::string& ifname, std::vector<uint8_t> data, uint32_t id) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 8;
    memcpy(frame.data, data.data(), 8);
    
    if (write(socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return false;
    }
    
    std::cout << "Message sent on " << ifname << " with id " << id << " and data ";
    for (int i = 0; i < 8; ++i) {
        printf("%02X ", frame.data[i]);
    }
    std::cout << std::endl;
    
    return true;
}


//<int canbus::pullMessage(uint32_t id, std::vector<can_frame>& rxFrames) {
//     int ret = 0;
//     if (_receiveMemory[id].size() > 0) {
//         std::copy(_receiveMemory[id].begin(), _receiveMemory[id].end(), std::back_inserter(rxFrames));
//         ret = 1;
//     }
//     _receiveMemory[id].clear();
//     std::cout << "pulled";
//     return ret;
// }

uint canbus::doubleToUint(double x, double xMin, double xMax, int bits)
{
    // Converts a float to an unsigned int, given range and number of bits
    double span = xMax - xMin;
    return (uint)((x - xMin) * ((double)((1 << bits) - 1)) / span);
}

double canbus::uintToDouble(uint32_t xInt, double xMin, double xMax, int bits) {
    double span = xMax - xMin;
    return ((double)xInt) * span / ((double)((1 << bits) - 1)) + xMin;
}

std::vector<uint8_t> canbus::createCtrlMessage(double position, double velocity, double torque, double kp, double kd)
{
    uint pInt = doubleToUint(position, P_MIN, P_MAX, BITS_POSITION);
    uint vInt = doubleToUint(velocity, V_MIN, V_MAX, BITS_VELOCITY);
    uint kpInt = doubleToUint(kp, KP_MIN, KP_MAX, BITS_KP);
    uint kdInt = doubleToUint(kd, KD_MIN, KD_MAX, BITS_KD);
    uint tInt = doubleToUint(torque, T_MIN, T_MAX, BITS_TORQUE);

    // Creation of the vector of 8 bytes for the CAN message
    std::vector<uint8_t> canMessage(8, 0);

    // Filling the vector with the converted values
    canMessage[0] = (pInt >> 8) & 0xFF;
    canMessage[1] = pInt & 0xFF;

    canMessage[2] = (vInt >> 4) & 0xFF;
    canMessage[3] = ((vInt & 0xF) << 4) | ((kpInt >> 8) & 0xF);

    canMessage[4] = kpInt & 0xFF;

    canMessage[5] = (kdInt >> 4) & 0xFF;
    canMessage[6] = ((kdInt & 0xF) << 4) | ((tInt >> 8) & 0xF);

    canMessage[7] = tInt & 0xFF;

    // Displaying the CAN message in binary
    return canMessage;
}

void canbus::unpackCanFrame(struct can_frame frame, std::vector<double>& listVal) {
    uint32_t pInt = frame.data[1] << 8 | frame.data[2];
    uint32_t vInt = frame.data[3] << 4 | frame.data[4] >> 4;
    uint32_t tInt = (frame.data[4] & 0xF) << 8 | frame.data[5];

    listVal[0] = uintToDouble(pInt, P_MIN, P_MAX, BITS_POSITION);
    listVal[1] = uintToDouble(vInt, V_MIN, V_MAX, BITS_VELOCITY);
    listVal[2] = uintToDouble(tInt, -T_MAX, T_MAX, BITS_TORQUE);
}

struct can_frame packCanFrame(const std::vector<uint8_t> canMessage)
{
    struct can_frame frame;
    frame.can_id = 1;
    frame.can_dlc = canMessage.size();
    for (uint i = 0; i < uint(canMessage.size()); ++i)
    {
        frame.data[i] = canMessage[i];
    }

    return frame;
}
