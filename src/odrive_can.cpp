#include <cstring>

#include <unistd.h>

#include <linux/can/raw.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include "odrive_can.hpp"

namespace ODrive_CAN{

void Clear_CAN_Bus(int can_socket){
    can_frame frame;
    while (read(can_socket, &frame, sizeof(frame)));
}
CAN_Frame Read_CAN_Bus1(int can_socket){
    can_frame raw_frame;
    ssize_t bytes_read;
    CAN_Frame frame;

    // TODO: What do we do if the entire frame wasn't read? Is that a possible case?
    // Attempt to read more data? Wouldn't this potentially also read part of the next frame?
    bytes_read = read(can_socket, &raw_frame, sizeof(raw_frame));
    if (bytes_read != sizeof(frame)) return frame;

    frame.node_id = (raw_frame.can_id & 0x3f) >> 5;
    frame.cmd_id = raw_frame.can_id & 0x1f;
    frame.len = raw_frame.len;
    std::memcpy(&frame.cmd, raw_frame.data, raw_frame.len);
    return frame;
}
std::vector<CAN_Frame> Read_CAN_Bus(int can_socket, std::size_t max){
    std::vector<CAN_Frame> out;
    while (true){
        CAN_Frame frame = Read_CAN_Bus1(can_socket);
        if ((COMMAND)frame.cmd_id == COMMAND::NONE) break;
        out.push_back(frame);
    }
    return out;
}
ssize_t Write_CAN_Bus1(int can_socket, const CAN_Frame& frame){
    can_frame raw_frame;
    raw_frame.can_id = (frame.node_id << 5) | frame.cmd_id;
    std::memcpy(raw_frame.data, &frame.cmd, sizeof(raw_frame.data));
    raw_frame.len = frame.len;
    // TODO: What do we do if the full frame wasn't sent? Send it again? Is that a possible case?
    // Sending the remainder of the packet would cause it to not be read correctly.
    return write(can_socket, &raw_frame, sizeof(raw_frame));
}
std::vector<ssize_t> Write_Can_Bus(int can_socket, const std::vector<CAN_Frame>& frames){
    std::vector<ssize_t> out;
    for (const CAN_Frame& frame : frames) out.push_back(Write_CAN_Bus1(can_socket, frame));
    return out;
}

int Create_CAN_Socket(){
    return socket(PF_CAN, SOCK_RAW, CAN_RAW);
}
int Bind_CAN_Socket(int can_socket, const std::string& can_id, sockaddr_can* out_addr){
    sockaddr_can addr;
    ifreq if_req;

    std::strcpy(if_req.ifr_name, can_id.c_str());
    ioctl(can_socket, SIOCGIFINDEX, &if_req);

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_req.ifr_ifindex;

    if (out_addr) *out_addr = addr;
    return bind(can_socket, (sockaddr*)&addr, sizeof(addr));
}
int Close_CAN_Socket(int can_socket){
    return close(can_socket);
}

}
