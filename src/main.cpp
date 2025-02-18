#include <iostream>

#include "odrive_can.hpp"

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
    if (ODrive_CAN::Close_CAN_Socket(can_socket)){
        std::cerr << "Error: Failed to close can socket\n";
        return -1;
    }

    return 0;
}
