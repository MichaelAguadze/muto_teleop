"""Frame-level tests for the board protocol.

The point of these is that this package builds its own serial frames rather than
calling the vendor library. That is only safe while the bytes stay identical to
what MutoLibCore.__send produces, so the expected frames below were captured
from the vendor implementation and are hard-coded rather than recomputed -- a
test that recalculates the checksum the same way the code does would pass even
if both were wrong.
"""

from muto_teleop.muto_board import (
    ADDR_BUZZER, ADDR_FORWARD, ADDR_PWM_SERVO, ADDR_RUN_TIME, ADDR_STAY_PUT,
    GIMBAL_HOLD, build_frame, clamp,
)


def test_gimbal_frame_matches_vendor():
    # MutoLibCore.Gimbal_1_2(90, 57)
    assert build_frame(ADDR_PWM_SERVO, [90, 57]) == bytes(
        [0x55, 0x00, 0x0A, 0x01, 0x25, 0x5A, 0x39, 0x3C, 0x00, 0xAA])


def test_gimbal_hold_byte():
    # -1 masks to 0xFF on the wire; the board reads that as "leave this axis".
    frame = build_frame(ADDR_PWM_SERVO, [GIMBAL_HOLD & 0xFF, 57])
    assert frame[5] == 0xFF


def test_single_byte_payload_frame_length():
    # Payload of one gives length 0x09, not the gimbal's 0x0A.
    frame = build_frame(ADDR_FORWARD, [25])
    assert frame[2] == 0x09
    assert frame[4] == ADDR_FORWARD


def test_zero_payload_command():
    frame = build_frame(ADDR_STAY_PUT, [0])
    assert frame[:5] == bytes([0x55, 0x00, 0x09, 0x01, 0x11])
    assert frame[-2:] == bytes([0x00, 0xAA])


def test_checksum_wraps():
    # A payload large enough to push the sum past 255 must still produce a
    # single byte -- an unwrapped checksum would raise here.
    frame = build_frame(ADDR_BUZZER, [255])
    assert 0 <= frame[-3] <= 255
    assert len(frame) == 9


def test_speed_inversion_is_the_vendor_direction():
    # speed(5) is fastest and transmits 0, because the board wants a cycle time.
    from muto_teleop.muto_board import SPEED_MAX
    assert SPEED_MAX - 5 == 0
    assert build_frame(ADDR_RUN_TIME, [SPEED_MAX - 5])[5] == 0


def test_clamp_bounds():
    assert clamp(-10, 0, 180) == 0
    assert clamp(200, 0, 180) == 180
    assert clamp(90, 0, 180) == 90
