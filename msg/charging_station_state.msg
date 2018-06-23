# State of a charging station

uint8 BASE_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
uint8 SYSTEM_STATUS_CRITICAL = 5
uint8 SYSTEM_STATUS_EMERGENCY = 6
uint8 CUSTOM_MODE_UNKNOWN = 0
uint8 CUSTOM_MODE_OPEN = 1
uint8 CUSTOM_MODE_OPENING = 2
uint8 CUSTOM_MODE_CLOSED = 3
uint8 CUSTOM_MODE_CLOSING = 4

uint8 id
uint8 base_mode
uint8 system_status
uint32 custom_mode
