//! Offline SocketCAN conversion/error and serial-open regressions.
//! Private conversions are exercised without widening production visibility.
//! See `_Tasks/review-2026-09-07.md` and `_Tasks/review-iterations-2026-09-08.md`.
#[test]
fn serial_open_rejects_non_terminal() {
    assert!(canopen_linux::slcan::UnixSerialPort::open("/dev/null").is_err());
}

#[allow(dead_code)]
mod socketcan_conversion {
    include!("../src/socketcan_transport.rs");

    #[test]
    fn transmit_queue_full_is_retryable_but_interface_down_is_not() {
        for errno in [libc::ENOBUFS, libc::EAGAIN] {
            assert!(matches!(
                map_transmit_error(std::io::Error::from_raw_os_error(errno)),
                nb::Error::WouldBlock
            ));
        }
        for errno in [libc::ENETDOWN, libc::ENODEV, libc::EINVAL] {
            assert!(matches!(
                map_transmit_error(std::io::Error::from_raw_os_error(errno)),
                nb::Error::Other(CanError::BusError)
            ));
        }
    }

    #[test]
    fn extended_frame_must_not_become_standard_command() {
        let id = socketcan::ExtendedId::new(0x1201).unwrap();
        let data = socketcan::CanDataFrame::new(id, &[42]).unwrap();
        let frame = socketcan::CanFrame::from(data);
        let converted = sc_frame_to_canframe(&frame);
        assert!(converted.is_none(), "extended 0x1201 became {converted:?}");
    }
}
