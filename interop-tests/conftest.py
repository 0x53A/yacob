"""Fixtures for CANopen interop tests.

Uses UDP multicast on loopback by default (no root required).
Set CAN_TRANSPORT=socketcan to use vcan instead.

Run with:
  cd interop-tests
  uv run pytest -v
"""

import os
import subprocess
import time
from pathlib import Path

import can
import canopen
import pytest

from loopback_mcast import LoopbackMulticastBus

NODE_ID = 1
EDS_PATH = Path(__file__).parent / "vcan_node.eds"
WORKSPACE_ROOT = Path(__file__).parent.parent
VCAN_NODE_BIN = WORKSPACE_ROOT / "target" / "debug" / "examples" / "vcan_node"

# Transport selection: "udp" (default, no root) or "socketcan" (needs vcan)
CAN_TRANSPORT = os.environ.get("CAN_TRANSPORT", "udp")
VCAN_IFACE = os.environ.get("CAN_IFACE", "vcan_test0")


def _vcan_exists(iface: str) -> bool:
    result = subprocess.run(
        ["ip", "link", "show", iface],
        capture_output=True,
    )
    return result.returncode == 0


@pytest.fixture(scope="session")
def vcan_node_bin():
    """Build and return path to the vcan_node binary."""
    if not VCAN_NODE_BIN.exists():
        subprocess.run(
            ["cargo", "build", "--example", "vcan_node", "-p", "canopen-linux"],
            cwd=WORKSPACE_ROOT,
            check=True,
        )
    assert VCAN_NODE_BIN.exists(), f"Binary not found: {VCAN_NODE_BIN}"
    return VCAN_NODE_BIN


@pytest.fixture()
def node_process(vcan_node_bin):
    """Start the vcan_node process and yield it. Kill on cleanup."""
    env = os.environ.copy()
    if CAN_TRANSPORT == "socketcan":
        if not _vcan_exists(VCAN_IFACE):
            pytest.skip(f"vcan interface '{VCAN_IFACE}' not available")
        env["CAN_TRANSPORT"] = "socketcan"
        env["CAN_IFACE"] = VCAN_IFACE
    else:
        env["CAN_TRANSPORT"] = "udp"

    proc = subprocess.Popen(
        [str(vcan_node_bin)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    # Give it time to start and send boot heartbeat
    time.sleep(0.5)
    assert proc.poll() is None, f"vcan_node died on startup: {proc.stderr.read()}"
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def _make_bus():
    """Create a python-can bus for the configured transport."""
    if CAN_TRANSPORT == "socketcan":
        return can.Bus(interface="socketcan", channel=VCAN_IFACE)
    else:
        return LoopbackMulticastBus()


@pytest.fixture()
def network(node_process):
    """Create a python-canopen network connected via the configured transport."""
    net = canopen.Network()
    net.bus = _make_bus()
    net.notifier = can.Notifier(net.bus, net.listeners, timeout=1.0)
    node = canopen.RemoteNode(NODE_ID, str(EDS_PATH))
    net.add_node(node)
    yield net
    net.disconnect()


@pytest.fixture()
def raw_bus(node_process):
    """Raw python-can bus (for direct frame send/receive without canopen stack)."""
    bus = _make_bus()
    yield bus
    bus.shutdown()


@pytest.fixture()
def node(network):
    """Shortcut to the remote node object."""
    return network[NODE_ID]
