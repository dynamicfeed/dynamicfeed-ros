# A geolocated hazard near the robot. One stream for all hazard feeds (earthquake, wildfire,
# weather alert, volcano, disaster) so a planner subscribes once.
std_msgs/Header header
string category            # "earthquake" | "wildfire" | "weather_alert" | "volcano" | "disaster"
string severity            # source-reported severity/level (free text)
string headline
geographic_msgs/GeoPoint location
float64 distance_km        # distance from the robot (NaN if unknown)
float64 magnitude          # quake magnitude / fire-size proxy / NaN
Provenance provenance
