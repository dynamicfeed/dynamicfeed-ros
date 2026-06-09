# GNSS interference (jamming/spoofing) risk near the robot — derived from ADS-B NIC/NACp
# (the GPSJam method). A nav stack can degrade trust in its own GNSS fix when level is high.
uint8 LEVEL_NONE=0
uint8 LEVEL_LOW=1
uint8 LEVEL_MEDIUM=2
uint8 LEVEL_HIGH=3
std_msgs/Header header
float64 latitude
float64 longitude
uint8 level
float64 confidence         # 0..1
string method              # e.g. "adsb-nic-nacp"
Provenance provenance
