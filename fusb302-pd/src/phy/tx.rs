//! Identity and acknowledgement evidence owned by one in-flight transmission.
use pd_protocol::pd_rx::Sop;

// USB PD message header: MessageID occupies bits 11:9.
const MESSAGE_ID_SHIFT: u32 = 9;
// USB PD MessageID is a three-bit field, wrapping through values zero to seven.
const MESSAGE_ID_MASK: u16 = 7;
// USB PD header: extended bit, data-object count and message-type fields.
// Masking these distinguishes control messages without considering roles/ID.
const CONTROL_HEADER_MASK: u16 = 0xf01f;
// USB PD control-message type encoding for GoodCRC.
const GOODCRC_TYPE: u16 = 1;
// USB PD control-message type encoding for Soft_Reset.
const SOFT_RESET_TYPE: u16 = 13;

pub(super) struct PendingTx {
    message_id: u16,
    sop: Sop,
    soft_reset: bool,
    // Hardware I_RETRYFAIL for this transmission, not permission to retry.
    pub(super) retry_failed: bool,
    goodcrc_at: Option<u64>,
}

impl PendingTx {
    pub(super) fn new(sop: Sop, header: u16) -> Self {
        Self {
            message_id: Self::header_id(header),
            sop,
            soft_reset: header & CONTROL_HEADER_MASK == SOFT_RESET_TYPE,
            retry_failed: false,
            goodcrc_at: None,
        }
    }

    pub(super) fn header_id(header: u16) -> u16 {
        (header >> MESSAGE_ID_SHIFT) & MESSAGE_ID_MASK
    }

    /// I_SOFTRST can describe our own transmission only for Soft_Reset with ID zero.
    pub(super) fn reset_with_zero_id(&self) -> bool {
        self.soft_reset && self.message_id == 0
    }

    /// Record only matching GoodCRC; duplicates cannot move the timer origin.
    pub(super) fn acknowledge(
        &mut self,
        sop: Sop,
        header: u16,
        now_ms: impl FnOnce() -> u64,
    ) -> bool {
        if header & CONTROL_HEADER_MASK != GOODCRC_TYPE
            || sop != self.sop
            || Self::header_id(header) != self.message_id
        {
            return false;
        }
        self.goodcrc_at.get_or_insert(now_ms());
        true
    }

    pub(super) fn acknowledged_at(&self) -> Option<u64> {
        self.goodcrc_at
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_matching_goodcrc_can_acknowledge_and_duplicates_keep_the_origin() {
        // USB PD header: power-role bit 8, revision bits 7:6 and data-role bit 5.
        const ROLES_AND_REVISION: u16 = (1 << 8) | (2 << 6) | (1 << 5);
        // Cover all per-channel MessageIDs; roles/revision do not change identity.
        for sop in [Sop::Partner, Sop::Cable, Sop::CableDoublePrime] {
            for id in 0..=MESSAGE_ID_MASK {
                let mut tx = PendingTx::new(sop, id << MESSAGE_ID_SHIFT);
                let goodcrc = (id << MESSAGE_ID_SHIFT) | GOODCRC_TYPE;
                let wrong_id = (((id + 1) & MESSAGE_ID_MASK) << MESSAGE_ID_SHIFT) | GOODCRC_TYPE;
                assert!(!tx.acknowledge(sop, wrong_id, || panic!(
                    "unmatched ACK must not sample time"
                )));
                for other in [Sop::Partner, Sop::Cable, Sop::CableDoublePrime] {
                    if other != sop {
                        assert!(!tx.acknowledge(other, goodcrc, || panic!(
                            "unmatched SOP must not sample time"
                        )));
                    }
                }
                assert!(!tx.acknowledge(
                    sop,
                    (id << MESSAGE_ID_SHIFT) | SOFT_RESET_TYPE,
                    || panic!("non-GoodCRC must not sample time")
                ));
                assert_eq!(tx.acknowledged_at(), None);
                assert!(tx.acknowledge(sop, goodcrc, || 20));
                assert!(tx.acknowledge(sop, goodcrc | ROLES_AND_REVISION, || 30));
                assert_eq!(tx.acknowledged_at(), Some(20));
            }
        }
    }

    #[test]
    fn own_soft_reset_requires_control_encoding_and_message_id_zero() {
        for sop in [Sop::Partner, Sop::Cable, Sop::CableDoublePrime] {
            assert!(PendingTx::new(sop, SOFT_RESET_TYPE).reset_with_zero_id());
            for id in 1..=MESSAGE_ID_MASK {
                assert!(
                    !PendingTx::new(sop, (id << MESSAGE_ID_SHIFT) | SOFT_RESET_TYPE)
                        .reset_with_zero_id()
                );
            }
            // Data-object count one, then extended bit set: neither is a control reset.
            for non_control in [1 << 12, 1 << 15] {
                assert!(!PendingTx::new(sop, non_control | SOFT_RESET_TYPE).reset_with_zero_id());
            }
        }
    }
}
