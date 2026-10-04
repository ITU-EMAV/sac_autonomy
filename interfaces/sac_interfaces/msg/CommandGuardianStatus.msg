uint8 STATE_DISABLED=0
uint8 STATE_ARMED=1
uint8 STATE_ACTIVE=2
uint8 STATE_STOPPING=3
uint8 STATE_FAULT=4

builtin_interfaces/Time stamp
uint8 state
bool healthy
bool transport_enabled
string fault_reason
float64 command_age_ms
uint64 last_accepted_sequence
uint64 rejected_command_count
