# DF-VERIFY/1 trust envelope — attached to every Dynamic Feed message so a robot can decide
# whether to trust the datum: where it came from, how fresh it is, and whether its Ed25519
# signature verified ON THIS ROBOT. A safety-critical stack should refuse to act when
# signature_valid is false or stale is true.
string source              # e.g. "Open-Meteo", "USGS", "NOAA SWPC"
string source_url
float64 age_seconds        # age of the datum at publish time
bool stale                 # true if older than the feed's freshness budget
bool signature_valid       # true iff the awareness envelope's Ed25519 signature verified locally
string key_id              # signing key id (see https://dynamicfeed.ai/.well-known/keys)
string snapshot_id         # the signed snapshot this datum came from
string measured_at         # ISO 8601 timestamp the datum was observed at source
