import struct
import can

# Open CAN bus
bus = can.interface.Bus("can0", bustype="socketcan")

# Flush CAN bus
while not (bus.recv(timeout=0) is None): pass

node_id = 0                          # ODrive node ID (set in ODrive config)
cmd_id = 0x01                        # ODrive CAN cmd_id for 'Heartbeat'
message_id = (node_id << 5) | cmd_id # Message ID (mix of node_id and cmd_id

for msg in bus:
    if msg.arbitration_id == message_id:
        error, state, result, traj_done = struct.unpack("<IBBB", bytes(msg.data[:7]))
        break
    print(error, state, result, traj_done)
