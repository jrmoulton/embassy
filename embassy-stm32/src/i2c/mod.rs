//! Inter-Integrated-Circuit (I2C)
#![macro_use]
#![allow(missing_docs)]
#![allow(dead_code)]
#![allow(unused)]

#[cfg_attr(i2c_v1, path = "v1.rs")]
#[cfg_attr(any(i2c_v2, i2c_v3), path = "v2.rs")]
mod _version;

use core::future::Future;
use core::iter;
use core::marker::PhantomData;

use embassy_hal_internal::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;
#[cfg(feature = "time")]
use embassy_time::{Duration, Instant};
use mode::{I2CMode, Master, MasterMode, SlaveMode};
use stm32_metapac::i2c::vals::Oamsk;

use crate::dma::ChannelAndRequest;
use crate::gpio::{AFType, Pull};
use crate::interrupt::typelevel::Interrupt;
use crate::mode::{Async, Blocking, Mode};
use crate::time::Hertz;
use crate::{interrupt, peripherals};

/// I2C error.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration lost
    Arbitration,
    /// ACK not received (either to the address or to a data byte)
    Nack,
    /// Timeout
    Timeout,
    /// CRC error
    Crc,
    /// Overrun error
    Overrun,
    // Underun error
    Underun,
    // PEC error
    Pec,
    /// Zero-length transfers are not allowed.
    ZeroLengthTransfer,
}

pub mod mode {
    trait SealedMode {}

    #[allow(private_bounds)]
    pub trait I2CMode: SealedMode {}

    #[allow(private_bounds)]
    pub trait MasterMode: I2CMode {}
    #[allow(private_bounds)]
    pub trait SlaveMode: I2CMode {}

    pub struct Master;
    pub struct Slave;
    pub struct MultiMaster;

    impl SealedMode for Master {}
    impl I2CMode for Master {}
    impl MasterMode for Master {}

    impl SealedMode for Slave {}
    impl I2CMode for Slave {}
    impl SlaveMode for Slave {}

    impl SealedMode for MultiMaster {}
    impl I2CMode for MultiMaster {}
    impl MasterMode for MultiMaster {}
    impl SlaveMode for MultiMaster {}
}

/// I2C config
#[non_exhaustive]
#[derive(Copy, Clone, Default)]
pub struct Config {
    /// Enable internal pullup on SDA.
    ///
    /// Using external pullup resistors is recommended for I2C. If you do
    /// have external pullups you should not enable this.
    pub sda_pullup: bool,
    /// Enable internal pullup on SCL.
    ///
    /// Using external pullup resistors is recommended for I2C. If you do
    /// have external pullups you should not enable this.
    pub scl_pullup: bool,

    /// Timeout.
    #[cfg(feature = "time")]
    pub timeout: embassy_time::Duration,
}

#[repr(u8)]
#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AddrMask {
    NOMASK,
    MASK1,
    MASK2,
    MASK3,
    MASK4,
    MASK5,
    MASK6,
    MASK7,
}
impl From<AddrMask> for Oamsk {
    fn from(value: AddrMask) -> Self {
        match value {
            AddrMask::NOMASK => Oamsk::NOMASK,
            AddrMask::MASK1 => Oamsk::MASK1,
            AddrMask::MASK2 => Oamsk::MASK2,
            AddrMask::MASK3 => Oamsk::MASK3,
            AddrMask::MASK4 => Oamsk::MASK4,
            AddrMask::MASK5 => Oamsk::MASK5,
            AddrMask::MASK6 => Oamsk::MASK6,
            AddrMask::MASK7 => Oamsk::MASK7,
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Address {
    SevenBit(u8),
    TenBit(u16),
}
impl From<u8> for Address {
    fn from(value: u8) -> Self {
        Address::SevenBit(value)
    }
}
impl From<u16> for Address {
    fn from(value: u16) -> Self {
        assert!(value < 0x400, "Ten bit address must be less than 0x400");
        Address::TenBit(value)
    }
}
impl Address {
    fn add_mode(&self) -> stm32_metapac::i2c::vals::Addmode {
        match self {
            Address::SevenBit(_) => stm32_metapac::i2c::vals::Addmode::BIT7,
            Address::TenBit(_) => stm32_metapac::i2c::vals::Addmode::BIT10,
        }
    }
    fn addr(&self) -> u16 {
        match self {
            Address::SevenBit(addr) => *addr as u16,
            Address::TenBit(addr) => *addr,
        }
    }
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OA2 {
    pub addr: u8,
    pub mask: AddrMask,
}

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OwnAddresses {
    OA1(Address),
    OA2(OA2),
    Both { oa1: Address, oa2: OA2 },
}

/// Slave Configuration
#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SlaveAddrConfig {
    /// Target Address(es)
    pub addr: OwnAddresses,
    /// Control if the peripheral should respond to the general call address
    pub general_call: bool,
}
impl SlaveAddrConfig {
    pub fn new_oa1(addr: Address, general_call: bool) -> Self {
        Self {
            addr: OwnAddresses::OA1(addr),
            general_call,
        }
    }

    pub fn basic(addr: Address) -> Self {
        Self {
            addr: OwnAddresses::OA1(addr),
            general_call: false,
        }
    }
}

/// I2C driver.
pub struct I2c<'d, T: Instance, M: Mode, IM: I2CMode> {
    _peri: PeripheralRef<'d, T>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    #[cfg(feature = "time")]
    timeout: Duration,
    _phantom: PhantomData<M>,
    _phantom2: PhantomData<IM>,
}

impl<'d, T: Instance, M: Mode, IM: I2CMode> I2c<'d, T, M, IM> {
    fn timeout(&self) -> Timeout {
        Timeout {
            #[cfg(feature = "time")]
            deadline: Instant::now() + self.timeout,
        }
    }
}

impl<'d, T: Instance, IM: MasterMode> I2c<'d, T, Async, IM> {
    /// Create a new I2C driver.
    #[allow(clippy::too_many_arguments)]
    pub fn new_master(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner_master(peri, scl, sda, new_dma!(tx_dma), new_dma!(rx_dma), freq, config)
    }
}

impl<'d, T: Instance, IM: MasterMode> I2c<'d, T, Blocking, IM> {
    /// Create a new blocking I2C driver.
    pub fn new_blocking_master(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        freq: Hertz,
        config: Config,
    ) -> Self {
        Self::new_inner_master(peri, scl, sda, None, None, freq, config)
    }
}
impl<'d, T: Instance, M: Mode, IM: MasterMode> I2c<'d, T, M, IM> {
    /// Create a new I2C driver.
    fn new_inner_master(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        freq: Hertz,
        config: Config,
    ) -> Self {
        into_ref!(peri, scl, sda);

        T::enable_and_reset();

        scl.set_as_af_pull(
            scl.af_num(),
            AFType::OutputOpenDrain,
            match config.scl_pullup {
                true => Pull::Up,
                false => Pull::None,
            },
        );
        sda.set_as_af_pull(
            sda.af_num(),
            AFType::OutputOpenDrain,
            match config.sda_pullup {
                true => Pull::Up,
                false => Pull::None,
            },
        );

        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        let mut this = Self {
            _peri: peri,
            tx_dma,
            rx_dma,
            #[cfg(feature = "time")]
            timeout: config.timeout,
            _phantom: PhantomData,
            _phantom2: PhantomData,
        };

        this.init(freq, config);

        this
    }
}

impl<'d, T: Instance, IM: SlaveMode> I2c<'d, T, Async, IM> {
    /// Create a new asynchronous I2C Slave driver.
    #[allow(clippy::too_many_arguments)]
    pub fn new_slave(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::EventInterrupt, EventInterruptHandler<T>>
            + interrupt::typelevel::Binding<T::ErrorInterrupt, ErrorInterruptHandler<T>>
            + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        freq: Hertz,
        config: Config,
        slave_addr_config: SlaveAddrConfig,
    ) -> Self {
        Self::new_inner_slave(
            peri,
            scl,
            sda,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            freq,
            config,
            slave_addr_config,
        )
    }
}

impl<'d, T: Instance, IM: SlaveMode> I2c<'d, T, Blocking, IM> {
    /// Create a new blocking I2C slave driver.
    pub fn new_blocking_slave(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        freq: Hertz,
        config: Config,
        slave_addr_config: SlaveAddrConfig,
    ) -> Self {
        Self::new_inner_slave(peri, scl, sda, None, None, freq, config, slave_addr_config)
    }
}

impl<'d, T: Instance, M: Mode, IM: SlaveMode> I2c<'d, T, M, IM> {
    fn new_inner_slave(
        peri: impl Peripheral<P = T> + 'd,
        scl: impl Peripheral<P = impl SclPin<T>> + 'd,
        sda: impl Peripheral<P = impl SdaPin<T>> + 'd,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        freq: Hertz,
        config: Config,
        slave_addr_config: SlaveAddrConfig,
    ) -> Self {
        into_ref!(peri, scl, sda);

        T::enable_and_reset();

        scl.set_as_af_pull(
            scl.af_num(),
            AFType::OutputOpenDrain,
            match config.scl_pullup {
                true => Pull::Up,
                false => Pull::None,
            },
        );
        sda.set_as_af_pull(
            sda.af_num(),
            AFType::OutputOpenDrain,
            match config.sda_pullup {
                true => Pull::Up,
                false => Pull::None,
            },
        );

        unsafe { T::EventInterrupt::enable() };
        unsafe { T::ErrorInterrupt::enable() };

        let mut this = Self {
            _peri: peri,
            tx_dma,
            rx_dma,
            #[cfg(feature = "time")]
            timeout: config.timeout,
            _phantom: PhantomData,
            _phantom2: PhantomData,
        };

        this.init(freq, config);
        this.init_slave(freq, slave_addr_config);

        this
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SendStatus {
    Done,
    LeftoverBytes(usize),
    MoreBytesRequested,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReceiveStatus {
    // contains the number of bytes received
    Done(usize),
    // contains the number of bytes received
    SendRequested(usize),
}
impl ReceiveStatus {
    pub fn len(&self) -> usize {
        match self {
            ReceiveStatus::Done(n) => *n,
            ReceiveStatus::SendRequested(n) => *n,
        }
    }

    pub fn is_done(&self) -> bool {
        matches!(self, ReceiveStatus::Done(_))
    }

    pub fn has_request(&self) -> bool {
        matches!(self, ReceiveStatus::SendRequested(_))
    }
}

/// Received command
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandKind {
    /// Read
    SlaveSend,
    /// Slave is receiving data
    SlaveReceive,
}

pub struct Command {
    pub kind: CommandKind,
    pub address: Address,
}

#[derive(Copy, Clone)]
struct Timeout {
    #[cfg(feature = "time")]
    deadline: Instant,
}

#[allow(dead_code)]
impl Timeout {
    #[inline]
    fn check(self) -> Result<(), Error> {
        #[cfg(feature = "time")]
        if Instant::now() > self.deadline {
            return Err(Error::Timeout);
        }

        Ok(())
    }

    #[inline]
    fn with<R>(self, fut: impl Future<Output = Result<R, Error>>) -> impl Future<Output = Result<R, Error>> {
        #[cfg(feature = "time")]
        {
            use futures_util::FutureExt;

            embassy_futures::select::select(embassy_time::Timer::at(self.deadline), fut).map(|r| match r {
                embassy_futures::select::Either::First(_) => Err(Error::Timeout),
                embassy_futures::select::Either::Second(r) => r,
            })
        }

        #[cfg(not(feature = "time"))]
        fut
    }
}

struct State {
    #[allow(unused)]
    waker: AtomicWaker,
}

impl State {
    const fn new() -> Self {
        Self {
            waker: AtomicWaker::new(),
        }
    }
}

trait SealedInstance: crate::rcc::RccPeripheral {
    fn regs() -> crate::pac::i2c::I2c;
    fn state() -> &'static State;
}

/// I2C peripheral instance
#[allow(private_bounds)]
pub trait Instance: SealedInstance + 'static {
    /// Event interrupt for this instance
    type EventInterrupt: interrupt::typelevel::Interrupt;
    /// Error interrupt for this instance
    type ErrorInterrupt: interrupt::typelevel::Interrupt;
}

pin_trait!(SclPin, Instance);
pin_trait!(SdaPin, Instance);
dma_trait!(RxDma, Instance);
dma_trait!(TxDma, Instance);

/// Event interrupt handler.
pub struct EventInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::EventInterrupt> for EventInterruptHandler<T> {
    unsafe fn on_interrupt() {
        _version::on_interrupt::<T>()
    }
}

/// Error interrupt handler.
pub struct ErrorInterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

impl<T: Instance> interrupt::typelevel::Handler<T::ErrorInterrupt> for ErrorInterruptHandler<T> {
    unsafe fn on_interrupt() {
        _version::on_interrupt::<T>()
    }
}

foreach_peripheral!(
    (i2c, $inst:ident) => {
        impl SealedInstance for peripherals::$inst {
            fn regs() -> crate::pac::i2c::I2c {
                crate::pac::$inst
            }

            fn state() -> &'static State {
                static STATE: State = State::new();
                &STATE
            }
        }

        impl Instance for peripherals::$inst {
            type EventInterrupt = crate::_generated::peripheral_interrupts::$inst::EV;
            type ErrorInterrupt = crate::_generated::peripheral_interrupts::$inst::ER;
        }
    };
);

impl<'d, T: Instance, M: Mode, IM: MasterMode, A: embedded_hal_02::blocking::i2c::AddressMode>
    embedded_hal_02::blocking::i2c::Read<A> for I2c<'d, T, M, IM>
where
    A: Into<Address>,
{
    type Error = Error;

    fn read(&mut self, address: A, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address.into(), buffer)
    }
}

impl<'d, T: Instance, M: Mode, IM: MasterMode, A: embedded_hal_02::blocking::i2c::AddressMode>
    embedded_hal_02::blocking::i2c::Write<A> for I2c<'d, T, M, IM>
where
    A: Into<Address>,
{
    type Error = Error;

    fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address.into(), write)
    }
}

impl<'d, T: Instance, M: Mode, IM: MasterMode, A: embedded_hal_02::blocking::i2c::AddressMode>
    embedded_hal_02::blocking::i2c::WriteRead<A> for I2c<'d, T, M, IM>
where
    A: Into<Address>,
{
    type Error = Error;

    fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address.into(), write, read)
    }
}

impl embedded_hal_1::i2c::Error for Error {
    fn kind(&self) -> embedded_hal_1::i2c::ErrorKind {
        match *self {
            Self::Bus => embedded_hal_1::i2c::ErrorKind::Bus,
            Self::Arbitration => embedded_hal_1::i2c::ErrorKind::ArbitrationLoss,
            Self::Nack => {
                embedded_hal_1::i2c::ErrorKind::NoAcknowledge(embedded_hal_1::i2c::NoAcknowledgeSource::Unknown)
            }
            Self::Timeout => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Crc => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Overrun => embedded_hal_1::i2c::ErrorKind::Overrun,
            Self::Underun => embedded_hal_1::i2c::ErrorKind::Other,
            Self::Pec => embedded_hal_1::i2c::ErrorKind::Other,
            Self::ZeroLengthTransfer => embedded_hal_1::i2c::ErrorKind::Other,
        }
    }
}

impl<'d, T: Instance, M: Mode, IM: I2CMode> embedded_hal_1::i2c::ErrorType for I2c<'d, T, M, IM> {
    type Error = Error;
}

impl<'d, T: Instance, M: Mode, IM: MasterMode, A: embedded_hal_1::i2c::AddressMode> embedded_hal_1::i2c::I2c<A>
    for I2c<'d, T, M, IM>
where
    Address: From<A>,
{
    fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_read(address.into(), read)
    }

    fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error> {
        self.blocking_write(address.into(), write)
    }

    fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.blocking_write_read(address.into(), write, read)
    }

    fn transaction(
        &mut self,
        address: A,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.blocking_transaction(address.into(), operations)
    }
}

impl<'d, T: Instance, IM: MasterMode, A: embedded_hal_async::i2c::AddressMode> embedded_hal_async::i2c::I2c<A>
    for I2c<'d, T, Async, IM>
where
    Address: From<A>,
{
    async fn read(&mut self, address: A, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read(address.into(), read).await
    }

    async fn write(&mut self, address: A, write: &[u8]) -> Result<(), Self::Error> {
        self.write(address.into(), write).await
    }

    async fn write_read(&mut self, address: A, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        self.write_read(address.into(), write, read).await
    }

    async fn transaction(
        &mut self,
        address: A,
        operations: &mut [embedded_hal_1::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction(address.into(), operations).await
    }
}

/// Frame type in I2C transaction.
///
/// This tells each method what kind of framing to use, to generate a (repeated) start condition (ST
/// or SR), and/or a stop condition (SP). For read operations, this also controls whether to send an
/// ACK or NACK after the last byte received.
///
/// For write operations, the following options are identical because they differ only in the (N)ACK
/// treatment relevant for read operations:
///
/// - `FirstFrame` and `FirstAndNextFrame`
/// - `NextFrame` and `LastFrameNoStop`
///
/// Abbreviations used below:
///
/// - `ST` = start condition
/// - `SR` = repeated start condition
/// - `SP` = stop condition
/// - `ACK`/`NACK` = last byte in read operation
#[derive(Copy, Clone)]
#[allow(dead_code)]
enum FrameOptions {
    /// `[ST/SR]+[NACK]+[SP]` First frame (of this type) in transaction and also last frame overall.
    FirstAndLastFrame,
    /// `[ST/SR]+[NACK]` First frame of this type in transaction, last frame in a read operation but
    /// not the last frame overall.
    FirstFrame,
    /// `[ST/SR]+[ACK]` First frame of this type in transaction, neither last frame overall nor last
    /// frame in a read operation.
    FirstAndNextFrame,
    /// `[ACK]` Middle frame in a read operation (neither first nor last).
    NextFrame,
    /// `[NACK]+[SP]` Last frame overall in this transaction but not the first frame.
    LastFrame,
    /// `[NACK]` Last frame in a read operation but not last frame overall in this transaction.
    LastFrameNoStop,
}

#[allow(dead_code)]
impl FrameOptions {
    /// Sends start or repeated start condition before transfer.
    fn send_start(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::FirstAndNextFrame => true,
            Self::NextFrame | Self::LastFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends stop condition after transfer.
    fn send_stop(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::LastFrame => true,
            Self::FirstFrame | Self::FirstAndNextFrame | Self::NextFrame | Self::LastFrameNoStop => false,
        }
    }

    /// Sends NACK after last byte received, indicating end of read operation.
    fn send_nack(self) -> bool {
        match self {
            Self::FirstAndLastFrame | Self::FirstFrame | Self::LastFrame | Self::LastFrameNoStop => true,
            Self::FirstAndNextFrame | Self::NextFrame => false,
        }
    }
}

/// Iterates over operations in transaction.
///
/// Returns necessary frame options for each operation to uphold the [transaction contract] and have
/// the right start/stop/(N)ACK conditions on the wire.
///
/// [transaction contract]: embedded_hal_1::i2c::I2c::transaction
#[allow(dead_code)]
fn operation_frames<'a, 'b: 'a>(
    operations: &'a mut [embedded_hal_1::i2c::Operation<'b>],
) -> Result<impl IntoIterator<Item = (&'a mut embedded_hal_1::i2c::Operation<'b>, FrameOptions)>, Error> {
    use embedded_hal_1::i2c::Operation::{Read, Write};

    // Check empty read buffer before starting transaction. Otherwise, we would risk halting with an
    // error in the middle of the transaction.
    //
    // In principle, we could allow empty read frames within consecutive read operations, as long as
    // at least one byte remains in the final (merged) read operation, but that makes the logic more
    // complicated and error-prone.
    if operations.iter().any(|op| match op {
        Read(read) => read.is_empty(),
        Write(_) => false,
    }) {
        return Err(Error::Overrun);
    }

    let mut operations = operations.iter_mut().peekable();

    let mut next_first_frame = true;

    Ok(iter::from_fn(move || {
        let op = operations.next()?;

        // Is `op` first frame of its type?
        let first_frame = next_first_frame;
        let next_op = operations.peek();

        // Get appropriate frame options as combination of the following properties:
        //
        // - For each first operation of its type, generate a (repeated) start condition.
        // - For the last operation overall in the entire transaction, generate a stop condition.
        // - For read operations, check the next operation: if it is also a read operation, we merge
        //   these and send ACK for all bytes in the current operation; send NACK only for the final
        //   read operation's last byte (before write or end of entire transaction) to indicate last
        //   byte read and release the bus for transmission of the bus master's next byte (or stop).
        //
        // We check the third property unconditionally, i.e. even for write opeartions. This is okay
        // because the resulting frame options are identical for write operations.
        let frame = match (first_frame, next_op) {
            (true, None) => FrameOptions::FirstAndLastFrame,
            (true, Some(Read(_))) => FrameOptions::FirstAndNextFrame,
            (true, Some(Write(_))) => FrameOptions::FirstFrame,
            //
            (false, None) => FrameOptions::LastFrame,
            (false, Some(Read(_))) => FrameOptions::NextFrame,
            (false, Some(Write(_))) => FrameOptions::LastFrameNoStop,
        };

        // Pre-calculate if `next_op` is the first operation of its type. We do this here and not at
        // the beginning of the loop because we hand out `op` as iterator value and cannot access it
        // anymore in the next iteration.
        next_first_frame = match (&op, next_op) {
            (_, None) => false,
            (Read(_), Some(Write(_))) | (Write(_), Some(Read(_))) => true,
            (Read(_), Some(Read(_))) | (Write(_), Some(Write(_))) => false,
        };

        Some((op, frame))
    }))
}
