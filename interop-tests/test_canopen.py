"""CANopen interop tests using python-canopen against our vcan_node.

These tests exercise the protocol over a real (virtual) CAN bus,
validating interoperability with the reference python-canopen stack.
"""

import struct
import time

import can
import canopen
import pytest

from conftest import NODE_ID


# ---------------------------------------------------------------------------
# Heartbeat
# ---------------------------------------------------------------------------


class TestHeartbeat:
    def test_heartbeat_received(self, raw_bus):
        """Node should send heartbeats (PreOperational state = 0x7F)."""
        hb_cob = 0x700 + NODE_ID
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == hb_cob:
                # Accept either boot-up (0x00) or PreOp (0x7F)
                assert msg.data[0] in (0x00, 0x7F), f"Unexpected state: 0x{msg.data[0]:02X}"
                return  # pass
        pytest.fail("No heartbeat received within 2s")

    def test_periodic_heartbeat(self, network, node):
        """After boot, node should send periodic heartbeats in PreOperational."""
        node.nmt.wait_for_heartbeat(timeout=2)
        state = node.nmt.state
        assert state == "PRE-OPERATIONAL"

    def test_heartbeat_interval(self, raw_bus):
        """Heartbeats should arrive approximately every 500ms."""
        hb_cob = 0x700 + NODE_ID
        timestamps = []
        deadline = time.time() + 3.0
        while time.time() < deadline and len(timestamps) < 5:
            msg = raw_bus.recv(timeout=1.0)
            if msg and msg.arbitration_id == hb_cob and msg.data[0] != 0x00:
                timestamps.append(msg.timestamp)

        assert len(timestamps) >= 3, f"Only got {len(timestamps)} heartbeats"
        intervals = [
            timestamps[i + 1] - timestamps[i]
            for i in range(len(timestamps) - 1)
        ]
        for dt in intervals:
            assert 0.3 < dt < 0.8, f"Heartbeat interval {dt:.3f}s out of range"


# ---------------------------------------------------------------------------
# NMT
# ---------------------------------------------------------------------------


class TestNmt:
    def test_nmt_start(self, network, node):
        """NMT Start should transition node to Operational."""
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "PRE-OPERATIONAL"

        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "OPERATIONAL"

    def test_nmt_stop(self, network, node):
        """NMT Stop should transition node to Stopped."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)
        node.nmt.state = "STOPPED"
        time.sleep(0.2)
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "STOPPED"

    def test_nmt_preoperational(self, network, node):
        """NMT Enter PreOperational from Operational."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)
        node.nmt.state = "PRE-OPERATIONAL"
        time.sleep(0.2)
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "PRE-OPERATIONAL"

    def test_nmt_reset_node(self, network, node):
        """NMT Reset Node should cause a boot-up heartbeat then PreOp."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)
        node.nmt.state = "RESET"
        # After reset, node should send boot-up (0x00) then go to PreOp
        time.sleep(1.0)
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "PRE-OPERATIONAL"

    def test_nmt_reset_communication(self, network, node):
        """NMT Reset Communication should cause boot-up then PreOp."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)
        node.nmt.state = "RESET COMMUNICATION"
        time.sleep(1.0)
        node.nmt.wait_for_heartbeat(timeout=2)
        assert node.nmt.state == "PRE-OPERATIONAL"


# ---------------------------------------------------------------------------
# SDO — Expedited
# ---------------------------------------------------------------------------


class TestSdoExpedited:
    def test_read_device_type(self, network, node):
        """Read 0x1000:0 (Device Type) — should be 0x191."""
        val = node.sdo[0x1000].raw
        assert val == 0x191

    def test_read_error_register(self, network, node):
        """Read 0x1001:0 (Error Register) — should be 0."""
        val = node.sdo[0x1001].raw
        assert val == 0

    def test_read_identity_vendor_id(self, network, node):
        """Read 0x1018:1 (Vendor ID)."""
        val = node.sdo[0x1018][1].raw
        assert val == 0xCAFE

    def test_read_identity_product_code(self, network, node):
        """Read 0x1018:2 (Product Code)."""
        val = node.sdo[0x1018][2].raw
        assert val == 0x0001

    def test_write_read_u8(self, network, node):
        """Write and read back 0x6200:1 (output1, u8)."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)
        node.sdo[0x6200][1].raw = 0x42
        val = node.sdo[0x6200][1].raw
        assert val == 0x42

    def test_write_read_u16(self, network, node):
        """Write and read back 0x6200:2 (output2, u16)."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)
        node.sdo[0x6200][2].raw = 0xBEEF
        val = node.sdo[0x6200][2].raw
        assert val == 0xBEEF

    def test_write_readonly_rejected(self, network, node):
        """Writing to read-only 0x1000:0 should be aborted."""
        with pytest.raises(canopen.SdoAbortedError) as exc_info:
            node.sdo[0x1000].raw = 0
        # 0x06010002 = Attempt to write a read only object
        assert exc_info.value.code == 0x06010002

    def test_read_nonexistent_object(self, raw_bus):
        """Reading a non-existent object should return SDO abort."""
        # python-canopen rejects unknown indices client-side, so send raw SDO
        sdo_req = can.Message(
            arbitration_id=0x601,
            data=bytes([0x40, 0xFF, 0xFF, 0x00, 0, 0, 0, 0]),
            is_extended_id=False,
        )
        raw_bus.send(sdo_req)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x581 and len(msg.data) == 8:
                cs = (msg.data[0] >> 5) & 0x07
                if cs == 4:  # Abort
                    code = struct.unpack_from("<I", msg.data, 4)[0]
                    assert code == 0x06020000  # Object does not exist
                    return
        pytest.fail("No SDO abort response received")


# ---------------------------------------------------------------------------
# SDO — Block transfer
# ---------------------------------------------------------------------------


class TestSdoBlockTransfer:
    def test_block_download_roundtrip(self, network, node):
        """Block download via python-canopen should write a large octet string."""
        payload = bytes(range(32))

        # size must be declared: without it python-canopen cannot flag the
        # final segment and silently truncates the transfer.
        with node.sdo[0x2001].open(
            "wb", buffering=7, size=len(payload), block_transfer=True
        ) as fp:
            fp.write(payload)

        with node.sdo[0x2001].open("rb", buffering=7, block_transfer=True) as fp:
            assert fp.read(len(payload)) == payload


# ---------------------------------------------------------------------------
# SDO — Identity record
# ---------------------------------------------------------------------------


class TestSdoIdentityRecord:
    def test_read_all_identity_subindices(self, network, node):
        """Read all subindices of 0x1018."""
        assert node.sdo[0x1018][1].raw == 0xCAFE
        assert node.sdo[0x1018][2].raw == 0x0001
        assert node.sdo[0x1018][3].raw == 0x00010000
        assert node.sdo[0x1018][4].raw == 0x00000001


# ---------------------------------------------------------------------------
# PDO — Config protection
# ---------------------------------------------------------------------------


class TestPdoConfigProtection:
    def test_pdo_write_rejected_in_operational(self, raw_bus):
        """Writing PDO config (0x1800:1) in Operational should be rejected."""
        # First send NMT Start to go Operational
        nmt_start = can.Message(
            arbitration_id=0x000,
            data=bytes([0x01, 0x01]),  # Start node 1
            is_extended_id=False,
        )
        raw_bus.send(nmt_start)
        time.sleep(0.3)

        # Send raw SDO expedited download to 0x1800:01
        sdo_req = can.Message(
            arbitration_id=0x601,
            data=bytes([0x23, 0x00, 0x18, 0x01, 0x81, 0x01, 0x00, 0x80]),
            is_extended_id=False,
        )
        raw_bus.send(sdo_req)

        # Wait for SDO response
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x581 and len(msg.data) == 8:
                cs = (msg.data[0] >> 5) & 0x07
                if cs == 4:  # Abort
                    code = struct.unpack_from("<I", msg.data, 4)[0]
                    assert code == 0x08000022
                    return
        pytest.fail("No SDO abort response received")


# ---------------------------------------------------------------------------
# PDO — Data exchange
# ---------------------------------------------------------------------------


class TestPdo:
    def test_rpdo_writes_to_od(self, network, node):
        """Sending an RPDO frame should update the OD, readable via SDO."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)

        # Send RPDO1 (COB-ID 0x201 for node 1): output1=0xAA, output2=0x1234
        rpdo_data = struct.pack("<BH", 0xAA, 0x1234)
        msg = can.Message(
            arbitration_id=0x201,
            data=rpdo_data,
            is_extended_id=False,
        )
        network.bus.send(msg)
        time.sleep(0.1)

        # Read back via SDO
        assert node.sdo[0x6200][1].raw == 0xAA
        assert node.sdo[0x6200][2].raw == 0x1234

    def test_tpdo_received_after_change(self, network, node):
        """After writing outputs via RPDO, the node should echo them back
        as TPDO1 (since vcan_node mirrors outputs->inputs)."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.2)

        # Send RPDO1 with known values
        rpdo_data = struct.pack("<BH", 0x55, 0xABCD)
        msg = can.Message(
            arbitration_id=0x201,
            data=rpdo_data,
            is_extended_id=False,
        )
        network.bus.send(msg)

        # Listen for TPDO1 (COB-ID 0x181 for node 1)
        tpdo_cob = 0x181
        deadline = time.time() + 3.0
        received = False
        while time.time() < deadline:
            rx = network.bus.recv(timeout=0.5)
            if rx and rx.arbitration_id == tpdo_cob and len(rx.data) >= 3:
                val1 = rx.data[0]
                val2 = struct.unpack_from("<H", rx.data, 1)[0]
                if val1 == 0x55 and val2 == 0xABCD:
                    received = True
                    break

        assert received, "Did not receive expected TPDO1 with echoed values"


# ---------------------------------------------------------------------------
# EMCY
# ---------------------------------------------------------------------------


class TestEmcy:
    def test_emcy_on_error(self, raw_bus):
        """Writing 0xEE to output1 should trigger an EMCY frame."""
        # NMT Start
        raw_bus.send(can.Message(arbitration_id=0x000, data=bytes([0x01, 0x01]), is_extended_id=False))
        time.sleep(0.3)

        # SDO write 0xEE to 0x6200:1 (expedited download)
        raw_bus.send(can.Message(
            arbitration_id=0x601,
            data=bytes([0x2F, 0x00, 0x62, 0x01, 0xEE, 0x00, 0x00, 0x00]),
            is_extended_id=False,
        ))
        time.sleep(0.3)

        # Listen for EMCY (COB-ID 0x081 for node 1)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x081 and len(msg.data) >= 3:
                error_code = struct.unpack_from("<H", msg.data, 0)[0]
                if error_code == 0x1000:
                    return
        pytest.fail("No EMCY frame received")

    def test_emcy_reset(self, raw_bus):
        """Writing 0x00 to output1 after error should send error-reset EMCY."""
        # NMT Start
        raw_bus.send(can.Message(arbitration_id=0x000, data=bytes([0x01, 0x01]), is_extended_id=False))
        time.sleep(0.3)

        # Trigger error
        raw_bus.send(can.Message(
            arbitration_id=0x601,
            data=bytes([0x2F, 0x00, 0x62, 0x01, 0xEE, 0x00, 0x00, 0x00]),
            is_extended_id=False,
        ))
        time.sleep(0.3)

        # Clear error
        raw_bus.send(can.Message(
            arbitration_id=0x601,
            data=bytes([0x2F, 0x00, 0x62, 0x01, 0x00, 0x00, 0x00, 0x00]),
            is_extended_id=False,
        ))
        time.sleep(0.3)

        # Listen for error-reset EMCY (code 0x0000)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x081 and len(msg.data) >= 3:
                error_code = struct.unpack_from("<H", msg.data, 0)[0]
                if error_code == 0x0000:
                    return
        pytest.fail("No error-reset EMCY frame received")


# ---------------------------------------------------------------------------
# SDO stress
# ---------------------------------------------------------------------------


class TestSdoStress:
    def test_rapid_read_write_cycles(self, network, node):
        """Perform many rapid SDO read/write cycles without errors."""
        node.nmt.state = "OPERATIONAL"
        time.sleep(0.1)

        for i in range(20):
            val = i & 0xFF
            node.sdo[0x6200][1].raw = val
            readback = node.sdo[0x6200][1].raw
            assert readback == val, f"Mismatch at cycle {i}: wrote {val}, read {readback}"

    def test_read_all_identity_fields_rapidly(self, network, node):
        """Read identity fields in rapid succession."""
        for _ in range(5):
            assert node.sdo[0x1018][1].raw == 0xCAFE
            assert node.sdo[0x1018][2].raw == 0x0001
            assert node.sdo[0x1018][3].raw == 0x00010000
            assert node.sdo[0x1018][4].raw == 0x00000001


# ---------------------------------------------------------------------------
# LSS
# ---------------------------------------------------------------------------


class TestLss:
    def test_lss_inquire_vendor_id(self, raw_bus):
        """Switch to LSS configuration mode and inquire vendor ID."""
        # Switch mode global → configuration
        switch = can.Message(
            arbitration_id=0x7E5,
            data=bytes([0x04, 0x01, 0, 0, 0, 0, 0, 0]),
            is_extended_id=False,
        )
        raw_bus.send(switch)
        time.sleep(0.1)

        # Inquire vendor ID
        inquire = can.Message(
            arbitration_id=0x7E5,
            data=bytes([0x5A, 0, 0, 0, 0, 0, 0, 0]),
            is_extended_id=False,
        )
        raw_bus.send(inquire)

        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x7E4 and len(msg.data) >= 5:
                if msg.data[0] == 0x5A:
                    vendor = struct.unpack_from("<I", msg.data, 1)[0]
                    assert vendor == 0xCAFE
                    # Switch back to waiting
                    raw_bus.send(can.Message(
                        arbitration_id=0x7E5,
                        data=bytes([0x04, 0x00, 0, 0, 0, 0, 0, 0]),
                        is_extended_id=False,
                    ))
                    return
        pytest.fail("No LSS vendor ID response received")

    def test_lss_selective_switch(self, raw_bus):
        """Use selective switch (by identity) to enter configuration mode."""
        identity = [
            (0x40, 0xCAFE),      # vendor
            (0x41, 0x0001),      # product
            (0x42, 0x00010000),  # revision
            (0x43, 0x00000001),  # serial
        ]

        for cs, val in identity:
            data = bytearray([cs, 0, 0, 0, 0, 0, 0, 0])
            struct.pack_into("<I", data, 1, val)
            raw_bus.send(can.Message(
                arbitration_id=0x7E5,
                data=bytes(data),
                is_extended_id=False,
            ))
            time.sleep(0.05)

        # Should get switch state response (0x44)
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = raw_bus.recv(timeout=0.5)
            if msg and msg.arbitration_id == 0x7E4:
                if msg.data[0] == 0x44:
                    # Switch back to waiting
                    raw_bus.send(can.Message(
                        arbitration_id=0x7E5,
                        data=bytes([0x04, 0x00, 0, 0, 0, 0, 0, 0]),
                        is_extended_id=False,
                    ))
                    return
        pytest.fail("No LSS switch state response received")
