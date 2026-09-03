# World-frame TCP pose. RPY uses fixed XYZ angles: Rz(yaw) * Ry(pitch) * Rx(roll).
std_msgs/Header header
uint64 sequence_id
uint64 source_timestamp_ns
uint64 receive_steady_ns
bool valid
string side
string source
float64[3] xyz_m
float64[3] rpy_rad
