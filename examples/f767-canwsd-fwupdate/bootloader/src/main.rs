//! Immutable STM32F767 CANopen updater reached through a single canwsd client.

#![no_std]
#![no_main]

use canopen_core::cobid::NodeId;
use canopen_core::node::{Node, NodeConfig, SharedNode};
use canopen_derive::object_dictionary;
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use defmt::{error, info, warn};
use embassy_executor::{Executor, Spawner};
use embassy_futures::select::{select, Either};
use embassy_net::StackResources;
use embassy_stm32::eth::{Ethernet, GenericPhy, PacketQueue, Sma};
use embassy_stm32::flash::WRITE_SIZE;
use embassy_stm32::{bind_interrupts, eth, peripherals};
use embassy_time::{Duration, Ticker, Timer};
use static_cell::StaticCell;

use f767_canwsd_fwupdate_common as fw;
use fw::hal::{ChannelTransport, EmbassyClock, Nvm, StatusLed};
use fw::{flash_status, program_control, FlashOps, ImageTarget, MetaRecord};

use {defmt_rtt as _, panic_probe as _};

const _: () = assert!(WRITE_SIZE == 16);

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
});

type Device = Ethernet<'static, peripherals::ETH, GenericPhy<Sma<'static, peripherals::ETH_SMA>>>;

object_dictionary! {
    #[validate_write(check_write)]
    pub struct BootOd {
        [0x1000] device_type: u32 = 0, ro;
        [0x1001] error_register: u8 = 0, ro;
        [0x1008] device_name: visible_string<32>, ro;
        [0x1018] identity: record {
            [1] vendor_id: u32 = 0, ro;
            [2] product_code: u32 = 0, ro;
            [3] revision: u32 = 1, ro;
            [4] serial_number: u32 = 0, ro;
        };
        [0x1F50] program_data: record {
            [1] image_chunk: domain<889>, rw;
        };
        [0x1F51] program_control: record {
            [1] command: u8 = 0, rw;
        };
        [0x1F56] program_sw_id: record {
            [1] expected_crc: u32 = 0, rw;
        };
        [0x1F57] flash_status_id: record {
            [1] flash_status: u32 = 0, ro, pdo;
        };
        [0x2000] update_info: record {
            [1] image_length: u32 = 0, rw;
            [2] bytes_written: u32 = 0, ro, pdo;
            [3] image_base: u32 = 0, ro;
            [4] image_capacity: u32 = 0, ro;
            [5] target: u8 = 1, rw;
        };
        tpdo[1](transmission_type = event_driven, inhibit_time = 50ms, event_timer = 1s) {
            flash_status,
            bytes_written,
        };
    }
}

impl BootOd {
    fn selected_target(&self) -> Option<ImageTarget> {
        ImageTarget::from_raw(self.target)
    }

    fn check_write(
        &self,
        index: u16,
        subindex: u8,
        data: &[u8],
    ) -> Result<(), canopen_core::od::OdError> {
        use canopen_core::od::OdError;
        match (index, subindex) {
            (0x1F50, 1) => {
                let target = self.selected_target().ok_or(OdError::ValueRange)?;
                if self.flash_status != flash_status::PROGRAMMING
                    || self.bytes_written as usize + data.len() > self.image_length as usize
                    || self.bytes_written as usize + data.len() > target.capacity() as usize
                {
                    Err(OdError::HardwareError)
                } else {
                    Ok(())
                }
            }
            (0x2000, 1) => {
                let len = u32::from_le_bytes(data.try_into().map_err(|_| OdError::ValueRange)?);
                let capacity = self
                    .selected_target()
                    .ok_or(OdError::ValueRange)?
                    .capacity();
                if len == 0 || len > capacity {
                    Err(OdError::ValueRange)
                } else {
                    Ok(())
                }
            }
            (0x2000, 5) => {
                let target = *data.first().ok_or(OdError::ValueRange)?;
                if data.len() != 1
                    || ImageTarget::from_raw(target).is_none()
                    || self.flash_status == flash_status::PROGRAMMING
                {
                    Err(OdError::ValueRange)
                } else {
                    Ok(())
                }
            }
            (0x1F51, 1) => match *data.first().ok_or(OdError::ValueRange)? {
                program_control::CLEAR if self.image_length != 0 => Ok(()),
                program_control::VALIDATE if self.flash_status == flash_status::PROGRAMMING => {
                    Ok(())
                }
                program_control::RESET => Ok(()),
                _ => Err(OdError::ValueRange),
            },
            _ => Ok(()),
        }
    }
}

struct Programmer {
    target: ImageTarget,
    accepted: u32,
    flushed: u32,
    pending: [u8; WRITE_SIZE],
    pending_len: usize,
}

impl Programmer {
    const fn new() -> Self {
        Self {
            target: ImageTarget::Firmware,
            accepted: 0,
            flushed: 0,
            pending: [0xFF; WRITE_SIZE],
            pending_len: 0,
        }
    }

    fn reset(&mut self, target: ImageTarget) {
        *self = Self {
            target,
            ..Self::new()
        };
    }

    fn received(&self) -> u32 {
        self.accepted
    }

    fn push(&mut self, nvm: &mut Nvm<'_>, mut data: &[u8]) -> Result<(), ()> {
        let accepted = data.len() as u32;
        if self.accepted as usize + data.len() > self.target.capacity() as usize {
            return Err(());
        }
        if self.pending_len != 0 {
            let take = core::cmp::min(WRITE_SIZE - self.pending_len, data.len());
            self.pending[self.pending_len..self.pending_len + take].copy_from_slice(&data[..take]);
            self.pending_len += take;
            data = &data[take..];
            if self.pending_len == WRITE_SIZE {
                let word = self.pending;
                self.commit(nvm, &word)?;
                self.pending_len = 0;
            }
        }
        let aligned = data.len() - data.len() % WRITE_SIZE;
        if aligned != 0 {
            self.commit(nvm, &data[..aligned])?;
        }
        let tail = &data[aligned..];
        self.pending[..tail.len()].copy_from_slice(tail);
        self.pending_len = tail.len();
        self.accepted += accepted;
        Ok(())
    }

    fn finish(&mut self, nvm: &mut Nvm<'_>) -> Result<(), ()> {
        if self.pending_len != 0 {
            let mut word = self.pending;
            word[self.pending_len..].fill(0xFF);
            self.pending_len = 0;
            self.commit(nvm, &word)?;
        }
        Ok(())
    }

    fn commit(&mut self, nvm: &mut Nvm<'_>, data: &[u8]) -> Result<(), ()> {
        nvm.write(self.target.offset() + self.flushed, data)
            .map_err(|_| ())?;
        self.flushed += data.len() as u32;
        Ok(())
    }
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let (meta, used) = fw::load_meta();
    if !meta.update_requested && fw::firmware_verifies(meta) {
        unsafe {
            (*SCB::PTR).vtor.write(fw::FIRMWARE_BASE);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
            cortex_m::asm::bootload(fw::FIRMWARE_BASE as *const u32);
        }
    }
    EXECUTOR
        .init(Executor::new())
        .run(|spawner| spawner.spawn(updater(spawner, meta, used).expect("spawn updater")))
}

#[embassy_executor::task]
async fn updater(spawner: Spawner, mut meta: MetaRecord, used: usize) -> ! {
    let p = embassy_stm32::init(fw::hal::clock_config());
    info!("F767 updater, metadata slot {}/{}", used, fw::META_SLOTS);
    let mut led = StatusLed::new(p.PB0);
    let mut nvm = Nvm::new(p.FLASH);
    if meta.update_requested {
        let cleared = meta.next(|r| r.update_requested = false);
        match fw::append_meta(&mut nvm, cleared) {
            Ok(()) => meta = cleared,
            Err(_) => warn!("could not clear update request"),
        }
    }

    let mac = fw::hal::mac_from_uid();
    static PACKETS: StaticCell<PacketQueue<4, 4>> = StaticCell::new();
    let device = Ethernet::new(
        PACKETS.init(PacketQueue::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        mac,
        p.ETH_SMA,
        p.PA2,
        p.PC1,
    );
    let mut dhcp = embassy_net::DhcpConfig::default();
    dhcp.hostname = Some(heapless::String::try_from("f767-updater").unwrap());
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        device,
        embassy_net::Config::dhcpv4(dhcp),
        RESOURCES.init(StackResources::new()),
        0,
    );
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(fw::ws::canwsd_task(stack).unwrap());

    let mut od = BootOd::new();
    od.vendor_id = fw::VENDOR_ID;
    od.product_code = fw::PRODUCT_CODE_BOOTLOADER;
    od.serial_number = fw::hal::serial_from_uid();
    od.target = ImageTarget::Firmware as u8;
    od.image_base = fw::FIRMWARE_BASE;
    od.image_capacity = fw::FIRMWARE_SIZE;
    od.device_name.push_str("F767 CANopen updater").unwrap();
    let node_id = NodeId::new(fw::NODE_ID).unwrap();
    let node: BootOdNode = Node::new(
        NodeConfig {
            heartbeat_interval_ms: 500,
            auto_start: true,
            ..NodeConfig::from_od(&od, node_id)
        },
        od,
    );
    NODE.init(node);
    spawner.spawn(protocol_task().unwrap());
    info!(
        "DHCP/canwsd ready on TCP {} network '{}'",
        fw::HTTP_PORT,
        fw::NETWORK_NAME
    );

    let mut programmer = Programmer::new();
    let mut ticker = Ticker::every(Duration::from_millis(250));
    loop {
        match select(NODE.wait_for_change(), ticker.next()).await {
            Either::First(()) => {
                while let Some(change) = NODE.with(|n| n.next_change()) {
                    handle_change(change, &mut nvm, &mut programmer, &mut meta).await;
                }
            }
            Either::Second(()) => led.toggle(),
        }
    }
}

async fn handle_change(
    change: BootOdChange,
    nvm: &mut Nvm<'_>,
    programmer: &mut Programmer,
    meta: &mut MetaRecord,
) {
    match change {
        BootOdChange::ImageChunk => {
            let mut chunk = [0u8; fw::MAX_CHUNK];
            let len = NODE.with(|n| {
                let data = n.od().image_chunk.as_slice();
                chunk[..data.len()].copy_from_slice(data);
                data.len()
            });
            if programmer.push(nvm, &chunk[..len]).is_ok() {
                NODE.with(|n| n.od_mut().bytes_written = programmer.received());
            } else {
                fail(nvm, meta, programmer.target);
            }
        }
        BootOdChange::Command(program_control::CLEAR) => {
            // Let the already-generated SDO response reach the socket before a
            // large blocking sector erase stalls execution.
            Timer::after_millis(20).await;
            clear(nvm, programmer, meta);
        }
        BootOdChange::Command(program_control::VALIDATE) => validate(nvm, programmer, meta),
        BootOdChange::Command(program_control::RESET) => SCB::sys_reset(),
        BootOdChange::Command(_) => {}
        BootOdChange::Target(raw) => {
            if let Some(target) = ImageTarget::from_raw(raw) {
                NODE.with(|n| {
                    n.od_mut().image_base = target.base();
                    n.od_mut().image_capacity = target.capacity();
                    n.od_mut().image_length = 0;
                });
            }
        }
        BootOdChange::ImageLength(len) => info!("image length {}", len),
        BootOdChange::ExpectedCrc(crc) => info!("expected crc {=u32:#010x}", crc),
    }
}

fn clear(nvm: &mut Nvm<'_>, programmer: &mut Programmer, meta: &mut MetaRecord) {
    let (target, declared) = NODE.with(|n| (n.od().selected_target(), n.od().image_length));
    let Some(target) = target else { return };
    let invalid = meta.next(|r| r.set_image(target, 0, 0, false));
    if fw::append_meta(nvm, invalid).is_err() {
        return fail(nvm, meta, target);
    }
    *meta = invalid;
    info!("erasing target {:?}, declared {} bytes", target, declared);
    if nvm
        .erase(target.offset(), target.offset() + target.capacity())
        .is_err()
    {
        return fail(nvm, meta, target);
    }
    programmer.reset(target);
    NODE.with(|n| {
        n.od_mut().bytes_written = 0;
        n.od_mut().flash_status = flash_status::PROGRAMMING;
    });
}

fn validate(nvm: &mut Nvm<'_>, programmer: &mut Programmer, meta: &mut MetaRecord) {
    if programmer.finish(nvm).is_err() {
        return fail(nvm, meta, programmer.target);
    }
    let (declared, expected) = NODE.with(|n| (n.od().image_length, n.od().expected_crc));
    let target = programmer.target;
    let actual = fw::image(target, declared).map(|bytes| fw::CRC32.checksum(bytes));
    if programmer.received() != declared
        || actual != Some(expected)
        || (target == ImageTarget::Firmware && !fw::firmware_vector_table_plausible(declared))
    {
        error!("validation failed for {:?}", target);
        return fail(nvm, meta, target);
    }
    let valid = meta.next(|r| r.set_image(target, declared, expected, true));
    if fw::append_meta(nvm, valid).is_err() {
        return fail(nvm, meta, target);
    }
    *meta = valid;
    NODE.with(|n| n.od_mut().flash_status = flash_status::VALID);
    info!(
        "accepted {:?}: {} bytes, crc {=u32:#010x}",
        target, declared, expected
    );
}

fn fail(nvm: &mut Nvm<'_>, meta: &mut MetaRecord, target: ImageTarget) {
    let invalid = meta.next(|r| r.set_image(target, 0, 0, false));
    if fw::append_meta(nvm, invalid).is_ok() {
        *meta = invalid;
    }
    NODE.with(|n| {
        n.od_mut().flash_status = flash_status::ERROR;
        n.set_error(0x5000, canopen_core::error_register::GENERIC, &[]);
    });
}

static NODE: SharedNode<BootOd, { BootOd::TPDO_COUNT }, { BootOd::RPDO_COUNT }> = SharedNode::new();

#[embassy_executor::task]
async fn protocol_task() -> ! {
    let mut transport = ChannelTransport;
    let clock = EmbassyClock;
    loop {
        NODE.with(|node| node.process(&mut transport, &clock));
        Timer::after_millis(1).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device>) -> ! {
    runner.run().await
}
