#include <iostream>
#include <string.h>
#include <unistd.h>
#include <net/if.h>

#include "canbus.h"
#include "./moteus/moteus.h"


using namespace mjbots;



int main() {
    canbus CanBus;

    // initialisation du can pour les modules canable
    system("sudo modprobe can_raw");
    system("sudo modprobe can-dev");
    system("sudo slcand -o -c -s8 /dev/canable0 can0");
    system("sudo ifconfig can0 up");
    system("sudo slcand -o -c -s8 /dev/canable1 can1");
    system("sudo ifconfig can1 up");

    int socket0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket0 < 0) {
        perror("Socket CAN0");
        return 1;
    }

    int socket1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket1 < 0) {
        perror("Socket CAN1");
        return 1;
    }

    struct ifreq ifr;
    struct sockaddr_can addr;

    strcpy(ifr.ifr_name, "can0");
    ioctl(socket0, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(socket0, (struct sockaddr *)&addr, sizeof(addr));

    strcpy(ifr.ifr_name, "can1");
    ioctl(socket1, SIOCGIFINDEX, &ifr);
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(socket1, (struct sockaddr *)&addr, sizeof(addr));


    // initialisation du can pour les fdcanusbs
    auto transport1 = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_B4CCDA38-if00"); // jaune 
    auto transport2 = std::make_shared<moteus::Fdcanusb>("/dev/serial/by-id/usb-mjbots_fdcanusb_ECE784D0-if00"); // gris

    moteus::Controller::Options options1;
    options1.transport = transport1;
    options1.id = 2;
    moteus::Controller controller1(options1);
    
    moteus::Controller::Options options2;
    options2.transport = transport2;
    options2.id = 1;
    moteus::Controller controller2(options2);

    char input;

    float val = 0.0;
    float kP = 0.0;
    float kD = 0.0;

    while (true) {
        input = getchar(); // Lire un caractère
        getchar(); // Lire le caractère newline pour nettoyer le buffer d'entrée

        if (input == 'e') {
            break; // Quitter la boucle
        } else if (input == 'a') {
            std::vector<uint8_t> messageA = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            if (!CanBus.sendCustomMessage(socket0, "can0", messageA, 0x002)) {
                std::cerr << "Failed to send message on can0" << std::endl;
            }
        } else if (input == 's') {
            std::vector<uint8_t> messageS = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
            if (!CanBus.sendCustomMessage(socket0, "can0", messageS, 0x002)) {
                std::cerr << "Failed to send message on can0" << std::endl;
            }
        }else if (input == 'q') {
            std::vector<uint8_t> messageQ = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            if (!CanBus.sendCustomMessage(socket1, "can1", messageQ, 0x002)) {
                std::cerr << "Failed to send message on can0" << std::endl;
            }
        }else if (input == 'w') {
            std::vector<uint8_t> messageW = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
            if (!CanBus.sendCustomMessage(socket1, "can1", messageW, 0x002)) {
                std::cerr << "Failed to send message on can0" << std::endl;
            }
        }else if (input == 'z'){
            moteus::PositionMode::Command cmd1;
            cmd1.kp_scale = kP;
            cmd1.kd_scale = kD;
            cmd1.position = val;
            cmd1.velocity = 0.0;
            controller1.SetPosition(cmd1);
        }else if (input == 'u'){
            controller1.SetStop();
        }else if (input == 'h'){
            moteus::PositionMode::Command cmd2;
            cmd2.kp_scale = kP;
            cmd2.kd_scale = kD;
            cmd2.position = val;
            cmd2.velocity = 0.0;
            controller2.SetPosition(cmd2);
        }else if (input == 'j'){
            controller2.SetStop();
        }
        else if (input == 'p')
        {
            std::cout << "valeur:";
            std::cin >> val;
            std::cout << "kp:";
            std::cin >> kP;
            std::cout << "kd:";
            std::cin >> kD;
        }

        else if(input == 'v'){
        std::vector<uint8_t> messageA = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        if (!CanBus.sendCustomMessage(socket0, "can0", messageA, 0x002)) {
            std::cerr << "Failed to send message on can0" << std::endl;
        }
        struct can_frame frame;
        int nbytes = read(socket0, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can raw socket read");
            return 0;
        } 
        std::vector<double> listVal(3);
        CanBus.unpackCanFrame(frame, listVal);
        std::cout << listVal[0];
        }

        else if (input == 'c'){
            std::vector<uint8_t> canMessage = CanBus.createCtrlMessage(val, 0.0, 0.0, kP, kD);
            if (!CanBus.sendCustomMessage(socket0, "can0", canMessage, 0x001)) {
            std::cerr << "Failed to send message on can0" << std::endl;
            }
        }
    // Fermeture du socket
    }

    close(socket0);
    close(socket1);
    
    return 0;
}

// g++ -o FourBus FourBus.cpp canbus.cpp 