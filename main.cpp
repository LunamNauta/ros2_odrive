#include <iostream>
#include <stdfloat>
#include <cstring>
#include <cstdint>
#include <string>
#include <vector>
#include <cmath>

#include <unistd.h>

#include <linux/can/raw.h>
#include <linux/can.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

namespace ODrive_CAN{

enum class COMMAND{
    GET_VERSION             = 0x000,
    HEARTBEAT               = 0x001,
    ESTOP                   = 0x002,
    GET_ERROR               = 0x003,
    RXSDO                   = 0x004,
    TXSDO                   = 0x005,
    ADDRESS                 = 0x006,
    SET_AXIS_STATE          = 0x007,
    GET_ENCODER_ESTIMATES   = 0x009,
    SET_CONTROLLER_MODE     = 0x00b,
    SET_INPUT_POS           = 0x00c,
    SET_INPUT_VEL           = 0x00d,
    SET_INPUT_TORQUE        = 0x00e,
    SET_LIMITS              = 0x00f,
    SET_TRAJ_VEL_LIMIT      = 0x011,
    SET_TRAJ_ACCEL_LIMITS   = 0x012,
    SET_TRAJ_INTERTIA       = 0x013,
    GET_IQ                  = 0x014,
    GET_TEMPERATURE         = 0x015,
    REBOOT                  = 0x016,
    GET_BUS_VOLTAGE_CURRENT = 0x017,
    CLEAR_ERRORS            = 0x018,
    SET_ABSOLUTE_POSITION   = 0x019,
    SET_POS_GAIN            = 0x01a,
    SET_VEL_GAINS           = 0x01b,
    GET_TORQUES             = 0x01c,
    GET_POWERS              = 0x01d,
    ENTER_DFU_MODE          = 0x01f,
    NONE
};

struct Heartbeat{
    std::uint32_t axis_error;
    std::uint8_t axis_state;
    std::uint8_t procedure_result;
    std::uint8_t trajectory_done_flag;
};
struct Get_Version{
    std::uint8_t protocol_version;
    std::uint8_t hw_version_major;
    std::uint8_t hw_version_minor;
    std::uint8_t hw_version_variant;
    std::uint8_t fw_version_major;
    std::uint8_t fw_version_minor;
    std::uint8_t fw_version_revision;
    std::uint8_t fw_version_unreleased;
};
struct Estop{};
struct Get_Error{
    std::uint32_t active_errors;
    std::uint32_t disarm_reason;
};
struct RxSdo{
    std::uint8_t opcode;
    std::uint16_t endpoint_id;
    std::uint8_t reserved;
    std::uint32_t value;
};
struct TxSdo{
    std::uint8_t reserved0;
    std::uint16_t endpoint_id;
    std::uint8_t reserved1;
    std::uint32_t value;
};
struct Address{
    std::uint8_t node_id;
    std::uint64_t serial_number : 48;
};
struct Set_Axis_State{
    std::uint32_t axis_requested_state;
};
struct Get_Encoder_Estimates{
    std::float32_t pos_estimate;
    std::float32_t vel_estimate;
};
struct Set_Controller_Mode{
    std::uint32_t control_mode;
    std::uint32_t input_mode;
};
struct Set_Input_Pos{
    std::float32_t input_pos;
    std::int16_t vel_ff;
    std::int16_t torque_ff;
};
struct Set_Input_Vel{
    std::float32_t input_vel;
    std::float32_t input_torque_ff;
};
struct Set_Input_Torque{
    std::float32_t input_torque;
};
struct Set_Limits{
    std::float32_t velocity_limit;
    std::float32_t current_limit;
};
struct Set_Traj_Vel_Limit{
    std::float32_t traj_accel_limit;
    std::float32_t traj_decel_limit;
};
struct Set_Traj_Accel_Limits{
    std::float32_t traj_accel_limit;
    std::float32_t traj_decel_limit;
};
struct Set_Traj_Inertia{
    std::float32_t traj_inertia;
};
struct Get_Iq{
    std::float32_t iq_setpoint;
    std::float32_t iq_measured;
};
struct Get_Temperature{
    std::float32_t fet_temperature;
    std::float32_t motor_temperature;
};
struct Reboot{
    std::uint8_t action;
};
struct Get_Bus_Voltage_Current{
    std::float32_t bus_voltage;
    std::float32_t bus_current;
};
struct Clear_Errors{
    std::uint8_t identify;
};
struct Set_Absolute_Position{
    std::float32_t position;
};
struct Set_Pos_Gain{
    std::float32_t pos_gain;
};
struct Set_Vel_Gains{
    std::float32_t vel_gain;
    std::float32_t vel_integrator_gain;
};
struct Get_Torques{
    std::float32_t torque_target;
    std::float32_t torque_estimate;
};
struct Get_Powers{
    std::float32_t electrical_power;
    std::float32_t mechanical_power;
};
struct Enter_DFU_Mode{};

struct CAN_Frame{
    std::uint8_t node_id;
    std::uint8_t cmd_id;
    std::uint8_t len;
    union{
        Get_Version get_version;
        Heartbeat heartbeat;
        Estop estop;
        RxSdo rxsdo;
        TxSdo txsdo;
        Address address;
        Set_Axis_State set_axis_state;
        Get_Encoder_Estimates get_encoder_estimates;
        Set_Controller_Mode set_controller_mode;
        Set_Input_Pos set_input_pos;
        Set_Input_Vel set_input_vel;
        Set_Input_Torque set_input_torque;
        Set_Limits set_limits;
        Set_Traj_Vel_Limit set_traj_vel_limit;
        Set_Traj_Accel_Limits set_traj_accel_limits;
        Set_Traj_Inertia set_traj_inertia;
        Get_Iq get_iq;
        Get_Temperature get_temperature;
        Reboot reboot;
        Get_Bus_Voltage_Current get_bus_voltage_current;
        Clear_Errors clear_errors;
        Set_Absolute_Position set_absolute_position;
        Set_Pos_Gain set_pos_gain;
        Set_Vel_Gains set_vel_gains;
        Get_Torques get_torques;
        Get_Powers get_powers;
        Enter_DFU_Mode enter_dfu_mode;
    } cmd;

    CAN_Frame() : node_id(-1), cmd_id((std::uint8_t)COMMAND::NONE){}
};

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

    frame.node_id = (raw_frame.can_id & 0x3f) >> 6;
    frame.cmd_id = raw_frame.can_id & 0x1f;
    frame.len = raw_frame.len;
    std::memcpy(&frame.cmd, raw_frame.data, raw_frame.len);
    return frame;
}
std::vector<CAN_Frame> Read_CAN_Bus(int can_socket, std::size_t max = 1){
    std::vector<CAN_Frame> out;
    while (true){
        CAN_Frame frame = Read_CAN_Bus1(can_socket);
        if ((COMMAND)frame.cmd_id == COMMAND::NONE) break;
        out.push_back(frame);
    }
    return out;
}
void Write_CAN_Bus1(int can_socket, const CAN_Frame& frame){
    can_frame raw_frame;
    raw_frame.can_id = (frame.node_id << 6) | frame.cmd_id;
    std::memcpy(raw_frame.data, frame.cmd, sizeof(frame.cmd));
    raw_frame.len = frame.len;
    // TODO: What do we do if the full frame wasn't sent? Send it again? Is that a possible case?
    // Sending the remainder of the packet would cause it to not be read correctly.
    return write(can_socket, &raw_frame, sizeof(raw_frame)) != sizeof(can_frame));
}
std::vector<std::size_t> Write_Can_Bus(int can_socket, const std::vector<CAN_Frame>& frames){
    std::vector<std::size_t> out;
    for (const CAN_Frame& frame : frames) out.push_back(Write_CAN_Bus1(can_socket, frame));
    return out;
}

int Create_CAN_Socket(){
    int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    return can_socket;
}
int Bind_CAN_Socket(int can_socket, const std::string& can_id, sockaddr_can* out_addr = nullptr){
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

}

int main(){
    // Set this to the node ID of the ODrive controller
    const std::size_t ODRIVE_NODE_ID = 0x00;

    // Open socket as CAN socket
    int can_socket = ODrive_CAN::Create_CAN_Socket();
    if (can_socket < 0){
        std::cerr << "Error: Failed to create CAN socket\n";
        return -1;
    }

    // Bind socket to CAN id
    if (ODrive_CAN::Bind_CAN_Socket(can_socket, "can0") < 0){
        std::cerr << "Error: Failed to bind CAN socket\n";
        return -1;
    }

    // Clear CAN buffer
    ODrive_CAN::Clear_CAN_Bus(can_socket);

    // Read all heartbeat signals
    while (true){
        ODrive_CAN::CAN_Frame frame = ODrive_CAN::Read_CAN_Bus1(can_socket);
        if ((ODrive_CAN::COMMAND)frame.cmd_id != ODrive_CAN::COMMAND::HEARTBEAT) continue;
        ODrive_CAN::Heartbeat heartbeat = frame.cmd.heartbeat;
        std::cout << "Axis_Error: " << heartbeat.axis_error << "\n";
        std::cout << "Axis_State: " << heartbeat.axis_state << "\n";
        std::cout << "Procedure_Result: " << heartbeat.procedure_result << "\n";
        std::cout << "Trajectory_Done_Flag:" << heartbeat.trajectory_done_flag << "\n";
        std::cout << "\n";
    }

    // Close the CAN socket
    if (close(can_socket) < 0){
        std::cerr << "Error: Failed to close can socket\n";
        return -1;
    }

    return 0;
}
