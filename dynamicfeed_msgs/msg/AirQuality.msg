# Air quality at the robot's location.
std_msgs/Header header
float64 us_aqi
string category            # "Good" | "Moderate" | ...
float64 pm2_5              # ug/m^3 (NaN if unknown)
float64 pm10               # ug/m^3 (NaN if unknown)
Provenance provenance
