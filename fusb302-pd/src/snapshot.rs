//! Controller configuration observations; reads only, no recovery writes.
use embedded_hal_async::i2c::I2c;

pub const ADDRESS: u8 = 0x22;
// Datasheet: ID, switches, control0..3, power, masks, status0/1. Never interrupt/FIFO.
pub const REGISTERS: [u8; 13] = [
    0x01, 0x02, 0x03, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0a, 0x0e, 0x0f, 0x40, 0x41,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    pub values: [u8; 13],
}

#[derive(Debug, PartialEq, Eq)]
pub struct Failure<E> {
    pub register: u8,
    pub source: E,
}

/// Sequential observations, not an atomic PHY-state snapshot. Fail closed on
/// first bus error, without retries, writes of register values or RX draining.
pub async fn snapshot<B: I2c>(bus: &mut B) -> Result<Snapshot, Failure<B::Error>> {
    let mut values = [0; 13];
    for (i, &register) in REGISTERS.iter().enumerate() {
        bus.write_read(ADDRESS, &[register], &mut values[i..i + 1])
            .await
            .map_err(|source| Failure { register, source })?;
    }
    Ok(Snapshot { values })
}
