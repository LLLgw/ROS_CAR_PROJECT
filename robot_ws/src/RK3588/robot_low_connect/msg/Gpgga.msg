# Message from GPGGA NMEA String
std_msgs/Header header

string message_id

# UTC seconds from midnight
uint32 day
uint32 hour
uint32 min
float64 seconds

float64 lat
float64 lon

uint32 gps_qual

float32 hdop 
float32 alt
