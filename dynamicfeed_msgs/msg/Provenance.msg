# DF-VERIFY/1 trust envelope — attached to every Dynamic Feed message so a robot can decide
# whether to trust the datum. A safety-critical stack should gate on it:
#   if prov.freshness_state != prov.FRESHNESS_LIVE or not prov.signature_valid: do not act on it.
uint8 FRESHNESS_LIVE=0
uint8 FRESHNESS_STALE=1
uint8 FRESHNESS_UNAVAILABLE=2

string source                       # e.g. "Open-Meteo", "USGS", "NOAA SWPC"
string source_url
builtin_interfaces/Time reported_at # when the source reported it (zero if unknown)
builtin_interfaces/Time measured_at # when Dynamic Feed observed/ingested it (zero if unknown)
float64 age_seconds                 # age of the datum at publish time
uint8 freshness_state               # LIVE | STALE | UNAVAILABLE
float64 max_age_seconds             # freshness budget; stale beyond this (NaN if unknown)
bool signature_valid                # true iff the envelope's Ed25519 signature verified ON THIS ROBOT
string key_id                       # signing key id (https://dynamicfeed.ai/.well-known/keys)
string snapshot_id                  # the signed snapshot this datum came from
string canonicalization             # "json-sorted-compact"
