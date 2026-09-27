# Bridge stamps local monotonic receipt time. Checker assigns observation_id.
string<=128 host_boot_id
float64 received_at_secs
uint64 observation_id
float64[<=35] q
float64[<=35] dq
bool start_button_pressed
