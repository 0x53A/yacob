use canopen_core::cobid::NodeId;
use canopen_core::nmt::NmtState;
use canopen_core::od::*;
use canopen_core::sdo::driver::AsyncCan;
use canopen_core::sdo::server::SdoServer;
use canopen_core::transport::CanFrame;
use canopen_derive::{object_dictionary, sdo_client_from_eds};

// Server-side OD (what the remote node has)
object_dictionary! {
    pub struct ServerOd {
        [0x1000] device_type: u32 = 0x191, ro;
        [0x1001] error_register: u8 = 0, ro;
        [0x1008] device_name: visible_string<32>, ro;
        [0x1017] producer_heartbeat_time: u16 = 500, rw;
        [0x6040] controlword: u16 = 0, rw, pdo;
        [0x6041] statusword: u16 = 0, ro, pdo;
    }
}

// Client generated from EDS — talks to the server
sdo_client_from_eds! {
    pub struct TestClient = "tests/test_pdo_device.eds"
    with {
        [0x1008] capacity = 32,
    };
}

sdo_client_from_eds! {
    pub struct NamesClient = "tests/test_sdo_client_names.eds";
}

// Mock async CAN that connects client to server
struct MockCan {
    server: SdoServer,
    od: ServerOd,
    pending: Option<CanFrame>,
}

impl MockCan {
    fn new() -> Self {
        let mut od = ServerOd::new();
        od.statusword = 0x1234;
        od.device_name.push_str("TestDev").unwrap();
        Self {
            server: SdoServer::new(),
            od,
            pending: None,
        }
    }
}

#[derive(Debug)]
struct MockErr;

impl AsyncCan for MockCan {
    type Error = MockErr;

    async fn transmit(&mut self, frame: &CanFrame) -> Result<(), MockErr> {
        let mut req = [0u8; 8];
        req.copy_from_slice(frame.data());
        let mut resp = [0u8; 8];
        let mut events: heapless::Deque<OdEvent, 16> = heapless::Deque::new();
        if self
            .server
            .process(
                &req,
                &mut self.od,
                &mut resp,
                &mut events,
                NmtState::Operational,
                0,
            )
            .is_ok()
        {
            self.pending = CanFrame::new(0x580 + 1, &resp);
        }
        Ok(())
    }

    async fn receive(&mut self) -> Result<CanFrame, MockErr> {
        self.pending.take().ok_or(MockErr)
    }
}

fn block_on<F: core::future::Future>(f: F) -> F::Output {
    use core::pin::pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn raw_waker() -> RawWaker {
        fn no_op(_: *const ()) {}
        fn clone(p: *const ()) -> RawWaker {
            RawWaker::new(p, &VTABLE)
        }
        const VTABLE: RawWakerVTable = RawWakerVTable::new(clone, no_op, no_op, no_op);
        RawWaker::new(core::ptr::null(), &VTABLE)
    }

    let waker = unsafe { Waker::from_raw(raw_waker()) };
    let mut cx = Context::from_waker(&waker);
    let mut f = pin!(f);
    loop {
        match f.as_mut().poll(&mut cx) {
            Poll::Ready(val) => return val,
            Poll::Pending => {}
        }
    }
}

#[test]
fn typed_client_read_u32() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        let val = client.read_i1000_s00_device_type(&mut can).await.unwrap();
        assert_eq!(val, 0x191);
    });
}

#[test]
fn typed_client_read_u16() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        let val = client.read_i6041_s00_statusword(&mut can).await.unwrap();
        assert_eq!(val, 0x1234);
    });
}

#[test]
fn typed_client_write_u16() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        client
            .write_i6040_s00_controlword(0xBEEF, &mut can)
            .await
            .unwrap();
    });
    assert_eq!(can.od.controlword, 0xBEEF);
}

#[test]
fn typed_client_read_string() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        let mut buf = [0u8; 64];
        let len = client
            .read_i1008_s00_device_name(&mut buf, &mut can)
            .await
            .unwrap();
        assert_eq!(&buf[..len], b"TestDev");
    });
}

#[test]
fn typed_client_read_write_roundtrip() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        // Write heartbeat time
        client
            .write_i1017_s00_producer_heartbeat_time(1000, &mut can)
            .await
            .unwrap();

        // Read it back
        let val = client
            .read_i1017_s00_producer_heartbeat_time(&mut can)
            .await
            .unwrap();
        assert_eq!(val, 1000);
    });
}

#[allow(dead_code)]
async fn generated_address_prefixed_methods_exist(client: &TestClient, can: &mut MockCan) {
    let _ = client.read_i1003_pre_defined_error_field(1, can).await;
}

#[allow(dead_code)]
async fn generated_duplicate_name_methods_exist(client: &NamesClient, can: &mut MockCan) {
    let _ = client.read_i2032_s00_max_motor_speed(can).await;
    let _ = client.write_i2032_s00_max_motor_speed(0, can).await;
    let _ = client.read_i6080_s00_max_motor_speed(can).await;
    let _ = client.write_i6080_s00_max_motor_speed(0, can).await;
}

#[test]
fn typed_client_raw_access() {
    let client = TestClient::new(NodeId::new(1).unwrap());
    let mut can = MockCan::new();

    block_on(async {
        // Use raw upload method
        let mut buf = [0u8; 4];
        let len = client.upload(0x1000, 0, &mut buf, &mut can).await.unwrap();
        assert_eq!(len, 4);
        assert_eq!(u32::from_le_bytes(buf), 0x191);
    });
}
