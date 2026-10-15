"""Loopback-only check shared by the sim processes."""

import socket
from pathlib import Path

# The kernel creates these, down and unaddressed, in every network namespace
# once the matching tunnel module is loaded (Docker Desktop's VM does). They
# carry no traffic unless configured with CAP_NET_ADMIN.
FALLBACK_TUNNELS = frozenset({
    "erspan0", "gre0", "gretap0", "ip6_vti0", "ip6gre0", "ip6tnl0", "ip_vti0",
    "sit0", "tunl0",
})


def interface_up(name):
    try:
        return bool(int(Path("/sys/class/net", name, "flags").read_text(), 16) & 0x1)
    except (OSError, ValueError):
        return True


def loopback_only():
    names = {name for _, name in socket.if_nameindex()}
    idle = {name for name in names & FALLBACK_TUNNELS if not interface_up(name)}
    return names - idle == {"lo"}
