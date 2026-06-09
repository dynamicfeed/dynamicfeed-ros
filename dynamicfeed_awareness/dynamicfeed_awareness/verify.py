"""
DF-VERIFY/1 awareness verifier for ROS 2 — pure Python, no ROS dependencies.

A safety-critical robot must not act on world-state it cannot prove is authentic and unaltered.
This verifies a Dynamic Feed awareness/receipt envelope: strip the `signature` field, canonicalize
json-sorted-compact (keys sorted recursively, compact separators, ensure_ascii), and check the
detached Ed25519 signature against the issuer's key. Identical to the reference verifiers at
https://dynamicfeed.ai/standard.

For safety-critical use, PIN the key (pass `trusted_keys=`) rather than fetching it every boot —
a pinned key means a network attacker can't substitute one. `fetch_keys()` is offered for
convenience / key discovery.
"""
import base64
import json
import urllib.request

from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

DEFAULT_BASE = "https://dynamicfeed.ai"
USER_AGENT = "dynamicfeed-ros/0.1 (+https://dynamicfeed.ai; hello@dynamicfeed.ai)"

# Dynamic Feed's published signing key (pin this for safety-critical robots; verify against
# https://dynamicfeed.ai/.well-known/keys before trusting). key_id -> base64url raw 32-byte pubkey.
PINNED_KEYS = {
    "df-ed25519-4cb32e72f333": "O4Kw2r-BjuDRL_Uyj3Vs8i-SnqHZUtPfARfj27NKEfk=",
}


def _b64(s: str) -> bytes:
    return base64.urlsafe_b64decode(s + "=" * (-len(s) % 4))


def canonical(payload: dict) -> bytes:
    """The exact bytes that were signed: payload (minus `signature`) as json-sorted-compact."""
    return json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")


def fetch_keys(base: str = DEFAULT_BASE, timeout: float = 10.0) -> dict:
    """Fetch the issuer JWKS (key_id -> base64url pubkey). Prefer a pinned key for safety-critical use."""
    req = urllib.request.Request(base.rstrip("/") + "/.well-known/keys", headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.load(r)


def verify(env: dict, trusted_keys: dict) -> tuple:
    """
    Verify a DF-VERIFY/1 envelope against trusted_keys ({key_id: base64url pubkey}).
    Returns (ok: bool, detail: str). Fails closed on anything unexpected.
    """
    sig = (env or {}).get("signature") or {}
    key_id, sig_b64 = sig.get("key_id"), sig.get("sig")
    if not key_id or not sig_b64:
        return False, "no signature block"
    if key_id not in trusted_keys:
        return False, "key_id %s not in trusted keys" % key_id
    payload = {k: v for k, v in env.items() if k != "signature"}
    try:
        Ed25519PublicKey.from_public_bytes(_b64(trusted_keys[key_id])).verify(_b64(sig_b64), canonical(payload))
    except Exception as e:  # any failure => not verified
        return False, "signature INVALID: %s" % e
    return True, "valid (%s)" % key_id
