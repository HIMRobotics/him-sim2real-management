# Complete frame in configured joint order. Limits remain dry-run placeholders.
string<=128 session_id
uint64 sequence
uint64 observation_id
float64[<=35] q
float64[<=35] dq
float64[<=35] kp
float64[<=35] kd
float64[<=35] tau
