# One independently timestamped track measurement (not a synchronized pair).
# header.stamp is ROS time corresponding to host receipt of this VESC reply.
std_msgs/Header header
# Changes on every kolesa_control restart. No silent zero-reset is permitted.
string session_id
# Signed, unwrapped, encoder-inversion-corrected hardware tachometer ticks.
int64 position_ticks
# Unwrapped tachometer_abs (includes forward and reverse motion).
uint64 travel_ticks
# Ground-distance calibration including this track's individual scale.
float64 meters_per_tick
# False on missing/stale telemetry, counter discontinuity or VESC fault.
bool valid
