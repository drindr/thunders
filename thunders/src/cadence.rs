//! Deterministic mixed short/long-slot cadence planning.
//!
//! This module is deliberately pure: it does not touch a timer, radio, packet
//! queue, or allocation. The caller starts from a profile that is already
//! known to be stable, probes progressively shorter forward slots, records
//! each completed probe, and receives a final profile containing the lowest
//! passing short slot plus the configured safety margin.

use serde::{Deserialize, Serialize};

/// Fixed application payload lengths used by a negotiated cadence profile.
///
/// Once committed, every Data packet in each direction has exactly the
/// corresponding length. This lets the negotiated compact codec omit both the
/// payload and NACK vector lengths from every slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TrafficContract {
    /// Exact forward (central-to-peripheral) application payload bytes.
    pub forward_payload_len: u16,
    /// Exact reverse (peripheral-to-central) application payload bytes.
    pub reverse_payload_len: u16,
}

impl TrafficContract {
    /// Create a fixed-length directional traffic contract.
    pub const fn new(forward_payload_len: u16, reverse_payload_len: u16) -> Self {
        Self {
            forward_payload_len,
            reverse_payload_len,
        }
    }
}

/// Policy controlling a bounded short-slot descent.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CadenceProbePolicy {
    /// Lowest short-slot period that may be probed, in microseconds.
    pub min_slot_us: u16,
    /// Decrement between adjacent short-slot candidates, in microseconds.
    pub step_us: u16,
    /// Superframes that must complete without an error before a candidate
    /// is recorded as passed.
    pub probe_superframes: u16,
    /// Number of `step_us` increments added above the lowest passing
    /// candidate when the search finalizes.
    pub safety_steps: u16,
}

impl CadenceProbePolicy {
    /// Create a probe policy.
    pub const fn new(
        min_slot_us: u16,
        step_us: u16,
        probe_superframes: u16,
        safety_steps: u16,
    ) -> Self {
        Self {
            min_slot_us,
            step_us,
            probe_superframes,
            safety_steps,
        }
    }
}

/// Safety policy for leaving an active short-payload cadence contract.
///
/// A zero threshold disables that trigger. Packet length is intentionally not
/// part of this policy: oversized payloads continue to return an explicit
/// error and never cause an automatic cadence change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CadenceExitPolicy {
    /// Exit after this many new retry-exhausted deliveries under the active
    /// contract. Zero disables the delivery-failure trigger.
    pub delivery_failures: u16,
    /// Exit after this many consecutive slots without a peer packet. Zero
    /// disables the consecutive-loss trigger.
    pub consecutive_misses: u8,
}

impl CadenceExitPolicy {
    /// Create an automatic-exit policy.
    pub const fn new(delivery_failures: u16, consecutive_misses: u8) -> Self {
        Self {
            delivery_failures,
            consecutive_misses,
        }
    }

    /// True when at least one automatic-exit trigger is enabled.
    pub const fn is_enabled(self) -> bool {
        self.delivery_failures != 0 || self.consecutive_misses != 0
    }
}

impl Default for CadenceProbePolicy {
    /// Generic 2 Mbit search defaults. The Link API additionally clamps this
    /// policy to each negotiated backend's hardware-verified production
    /// floor before any probe is armed.
    fn default() -> Self {
        Self::new(450, 25, 8, 2)
    }
}

/// A phase-indexed short/long slot profile.
///
/// The first `forward_slots` phases of a superframe use `short_slot_us`; the
/// reverse and shared-idle phases use `long_slot_us`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CadenceProfile {
    /// Period of forward/central-TX phases, in microseconds.
    pub short_slot_us: u16,
    /// Period of reverse/peripheral-TX and idle phases, in microseconds.
    pub long_slot_us: u16,
    /// Number of forward phases at the start of each superframe.
    pub forward_slots: u16,
    /// Number of reverse phases after the forward phases.
    pub reverse_slots: u16,
    /// Number of shared idle phases after the active phases.
    pub idle_slots: u16,
}

impl CadenceProfile {
    /// Create a mixed short/long-slot profile.
    pub const fn new(
        short_slot_us: u16,
        long_slot_us: u16,
        forward_slots: u16,
        reverse_slots: u16,
        idle_slots: u16,
    ) -> Self {
        Self {
            short_slot_us,
            long_slot_us,
            forward_slots,
            reverse_slots,
            idle_slots,
        }
    }

    /// Create a profile in which every phase uses the same slot period.
    pub const fn uniform(
        slot_us: u16,
        forward_slots: u16,
        reverse_slots: u16,
        idle_slots: u16,
    ) -> Self {
        Self::new(slot_us, slot_us, forward_slots, reverse_slots, idle_slots)
    }

    /// Return a copy with a different short-slot period.
    pub const fn with_short_slot(self, short_slot_us: u16) -> Self {
        Self {
            short_slot_us,
            ..self
        }
    }

    /// Number of slots in one superframe.
    pub const fn period_slots(&self) -> u32 {
        self.forward_slots as u32 + self.reverse_slots as u32 + self.idle_slots as u32
    }

    /// Wall-clock duration of one superframe in microseconds.
    pub const fn superframe_us(&self) -> u32 {
        self.forward_slots as u32 * self.short_slot_us as u32
            + (self.reverse_slots as u32 + self.idle_slots as u32) * self.long_slot_us as u32
    }

    /// Validate the phase counts and slot ordering.
    pub fn validate(&self) -> Result<(), CadenceError> {
        if self.short_slot_us == 0
            || self.long_slot_us == 0
            || self.short_slot_us > self.long_slot_us
            || self.forward_slots == 0
            || self.reverse_slots == 0
        {
            Err(CadenceError::InvalidProfile)
        } else {
            Ok(())
        }
    }
}

/// Measurements collected while probing one candidate.
///
/// A candidate passes only after `completed_superframes` reaches the policy's
/// `probe_superframes` and both directional failure counters are zero. Any
/// reported failure makes the completed probe fail deterministically.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProbeMetrics {
    /// Complete superframes observed at the candidate cadence.
    pub completed_superframes: u16,
    /// Forward-link failures observed during those superframes.
    pub forward_failures: u16,
    /// Reverse-link failures observed during those superframes.
    pub reverse_failures: u16,
}

impl ProbeMetrics {
    /// Create a metrics sample.
    pub const fn new(
        completed_superframes: u16,
        forward_failures: u16,
        reverse_failures: u16,
    ) -> Self {
        Self {
            completed_superframes,
            forward_failures,
            reverse_failures,
        }
    }

    /// True when enough superframes have been observed to decide the probe.
    pub fn is_complete(&self, policy: &CadenceProbePolicy) -> bool {
        self.completed_superframes >= policy.probe_superframes
    }

    /// True when the metrics represent a completed, error-free probe.
    pub fn is_pass(&self, policy: &CadenceProbePolicy) -> bool {
        self.is_complete(policy) && self.forward_failures == 0 && self.reverse_failures == 0
    }
}

/// The planner's decision after a metrics sample is recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ProbeDecision {
    /// The sample was incomplete; continue measuring the contained profile.
    Incomplete(CadenceProfile),
    /// The current candidate passed; use the contained final profile if the
    /// complete directional search has finished.
    Passed(CadenceProfile),
    /// Continue empirical testing with the contained candidate. This is also
    /// returned when a failed forward candidate advances the search to the
    /// reverse axis using the last known passing forward period.
    Continue(CadenceProfile),
    /// The current reverse candidate failed; use the contained final profile.
    Failed(CadenceProfile),
}

/// Observable state of an API-triggered cadence negotiation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CadenceNegotiationStatus {
    /// No API negotiation has been requested.
    Idle,
    /// Request/offer/accept or probe-arm messages are being exchanged.
    Negotiating,
    /// The active traffic contract is being synchronously released and the
    /// acquisition-safe profile restored.
    Releasing,
    /// A bounded candidate profile is currently being exercised.
    Probing {
        /// Candidate profile under test.
        candidate: CadenceProfile,
    },
    /// The final stable profile is committed for a future boundary.
    Applying {
        /// Final profile awaiting its apply epoch.
        profile: CadenceProfile,
    },
    /// Negotiation completed and this profile is active.
    Stable(CadenceProfile),
    /// Negotiation stopped with an error; the previous stable profile remains.
    Failed(CadenceError),
}

/// Errors produced by cadence-plan validation and state transitions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CadenceError {
    /// The PHY does not support negotiated hardware-slot profiles.
    Unsupported,
    /// Another cadence negotiation is already active.
    Busy,
    /// A requested application payload exceeds the protocol maximum.
    PayloadTooLarge,
    /// The peer rejected the offered traffic contract/profile.
    PeerRejected,
    /// The known-stable profile has an impossible duration or phase shape.
    InvalidProfile,
    /// The probe policy has a zero value or its floor is above the stable
    /// profile's short slot.
    InvalidPolicy,
    /// Adding the protocol overhead to a payload would overflow `u16`.
    WireLengthOverflow,
    /// The caller recorded metrics for a profile other than the current
    /// deterministic candidate.
    WrongCandidate,
    /// The search is already finalized.
    SearchFinished,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SearchAxis {
    Forward,
    Reverse,
}

/// Deterministic two-axis empirical cadence search state.
///
/// Forward candidates descend first while reverse remains stable. The lowest
/// passing forward period is then held while reverse candidates descend. Each
/// axis stops at its feasibility floor or its first failed candidate; safety
/// steps are added independently before the final profile is returned. If no
/// lower candidate passes on an axis, its known-stable period is retained.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CadenceSearch {
    stable_profile: CadenceProfile,
    traffic: TrafficContract,
    policy: CadenceProbePolicy,
    forward_wire_len: u16,
    reverse_wire_len: u16,
    forward_floor_us: u16,
    reverse_floor_us: u16,
    axis: SearchAxis,
    current_probe_us: Option<u16>,
    selected_forward_us: u16,
    lowest_passed_us: Option<u16>,
    lowest_passed_reverse_us: Option<u16>,
    passed_probes: u16,
    failed_probes: u16,
    last_metrics: Option<ProbeMetrics>,
    final_profile: Option<CadenceProfile>,
}

impl CadenceSearch {
    /// Create a planner from a known-stable profile.
    ///
    /// `protocol_overhead` is the number of wire bytes added to each fixed
    /// application payload (for example framing and ARQ headers).
    pub fn new(
        stable_profile: CadenceProfile,
        traffic: TrafficContract,
        policy: CadenceProbePolicy,
        protocol_overhead: u16,
    ) -> Result<Self, CadenceError> {
        let forward_wire_len = traffic
            .forward_payload_len
            .checked_add(protocol_overhead)
            .ok_or(CadenceError::WireLengthOverflow)?;
        let reverse_wire_len = traffic
            .reverse_payload_len
            .checked_add(protocol_overhead)
            .ok_or(CadenceError::WireLengthOverflow)?;
        Self::new_with_wire_lengths_and_floors(
            stable_profile,
            traffic,
            policy,
            forward_wire_len,
            reverse_wire_len,
            policy.min_slot_us,
            stable_profile.long_slot_us,
        )
    }

    /// Create a two-axis empirical search with independent directional floors.
    ///
    /// The forward period is searched first. The selected passing forward
    /// period is then held while reverse/idle periods are shortened. Floors
    /// are feasibility bounds only; every selected production profile still
    /// has to pass an actual bounded probe.
    pub fn new_with_floors(
        stable_profile: CadenceProfile,
        traffic: TrafficContract,
        policy: CadenceProbePolicy,
        protocol_overhead: u16,
        forward_floor_us: u16,
        reverse_floor_us: u16,
    ) -> Result<Self, CadenceError> {
        let forward_wire_len = traffic
            .forward_payload_len
            .checked_add(protocol_overhead)
            .ok_or(CadenceError::WireLengthOverflow)?;
        let reverse_wire_len = traffic
            .reverse_payload_len
            .checked_add(protocol_overhead)
            .ok_or(CadenceError::WireLengthOverflow)?;
        Self::new_with_wire_lengths_and_floors(
            stable_profile,
            traffic,
            policy,
            forward_wire_len,
            reverse_wire_len,
            forward_floor_us,
            reverse_floor_us,
        )
    }

    /// Create a search using exact directional Data wire lengths.
    pub fn new_with_wire_lengths_and_floors(
        stable_profile: CadenceProfile,
        traffic: TrafficContract,
        policy: CadenceProbePolicy,
        forward_wire_len: u16,
        reverse_wire_len: u16,
        forward_floor_us: u16,
        reverse_floor_us: u16,
    ) -> Result<Self, CadenceError> {
        stable_profile.validate()?;
        if policy.min_slot_us == 0
            || policy.step_us == 0
            || policy.probe_superframes == 0
            || forward_floor_us == 0
            || reverse_floor_us == 0
            || forward_floor_us > stable_profile.short_slot_us
            || reverse_floor_us > stable_profile.long_slot_us
        {
            return Err(CadenceError::InvalidPolicy);
        }

        if forward_wire_len < traffic.forward_payload_len
            || reverse_wire_len < traffic.reverse_payload_len
        {
            return Err(CadenceError::WireLengthOverflow);
        }

        let first_forward = first_lower_candidate_from(
            stable_profile.short_slot_us,
            forward_floor_us,
            policy.step_us,
        );
        let first_reverse = first_lower_candidate_from(
            stable_profile.long_slot_us,
            reverse_floor_us.max(stable_profile.short_slot_us),
            policy.step_us,
        );
        let (axis, current_probe_us, final_profile) = if let Some(first) = first_forward {
            (SearchAxis::Forward, Some(first), None)
        } else if let Some(first) = first_reverse {
            (SearchAxis::Reverse, Some(first), None)
        } else {
            (SearchAxis::Forward, None, Some(stable_profile))
        };

        Ok(Self {
            stable_profile,
            traffic,
            policy,
            forward_wire_len,
            reverse_wire_len,
            forward_floor_us,
            reverse_floor_us,
            axis,
            current_probe_us,
            selected_forward_us: stable_profile.short_slot_us,
            lowest_passed_us: None,
            lowest_passed_reverse_us: None,
            passed_probes: 0,
            failed_probes: 0,
            last_metrics: None,
            final_profile,
        })
    }

    /// The known-stable profile from which the search started.
    pub const fn stable_profile(&self) -> CadenceProfile {
        self.stable_profile
    }

    /// The application traffic contract.
    pub const fn traffic_contract(&self) -> TrafficContract {
        self.traffic
    }

    /// The active probe policy.
    pub const fn policy(&self) -> CadenceProbePolicy {
        self.policy
    }

    /// Common directional overhead when both fixed wire formats have it.
    pub const fn protocol_overhead(&self) -> u16 {
        let forward = self
            .forward_wire_len
            .saturating_sub(self.traffic.forward_payload_len);
        let reverse = self
            .reverse_wire_len
            .saturating_sub(self.traffic.reverse_payload_len);
        if forward == reverse { forward } else { 0 }
    }

    /// Exact forward wire length exercised by the probes, in bytes.
    pub const fn forward_wire_len(&self) -> u16 {
        self.forward_wire_len
    }

    /// Exact reverse wire length exercised by the probes, in bytes.
    pub const fn reverse_wire_len(&self) -> u16 {
        self.reverse_wire_len
    }

    /// Maximum forward and reverse wire lengths exercised by the probes.
    pub const fn wire_lengths(&self) -> (u16, u16) {
        (self.forward_wire_len(), self.reverse_wire_len())
    }

    /// Number of distinct lower candidates generated by this policy.
    pub fn candidate_count(&self) -> u16 {
        let count = |from: u16, to: u16| {
            let span = from.saturating_sub(to);
            if span == 0 {
                0
            } else {
                span / self.policy.step_us + u16::from(span % self.policy.step_us != 0)
            }
        };
        count(self.stable_profile.short_slot_us, self.forward_floor_us).saturating_add(count(
            self.stable_profile.long_slot_us,
            self.reverse_floor_us.max(self.forward_floor_us),
        ))
    }

    /// The candidate currently awaiting measurements, if any.
    pub fn next_probe(&self) -> Option<CadenceProfile> {
        self.current_probe_us.map(|period| match self.axis {
            SearchAxis::Forward => self.stable_profile.with_short_slot(period),
            SearchAxis::Reverse => CadenceProfile {
                short_slot_us: self.selected_forward_us,
                long_slot_us: period,
                ..self.stable_profile
            },
        })
    }

    /// Record a metrics sample for the current candidate.
    ///
    /// Incomplete metrics return [`ProbeDecision::Incomplete`] without
    /// changing the pass/fail counts. A complete error-free sample records a
    /// pass and advances by one step; a complete sample with any directional
    /// failure records a failure and finalizes the search.
    pub fn record_probe(
        &mut self,
        candidate: CadenceProfile,
        metrics: ProbeMetrics,
    ) -> Result<ProbeDecision, CadenceError> {
        let Some(current_us) = self.current_probe_us else {
            return Err(CadenceError::SearchFinished);
        };
        if self.next_probe() != Some(candidate) {
            return Err(CadenceError::WrongCandidate);
        }

        self.last_metrics = Some(metrics);
        if !metrics.is_complete(&self.policy) {
            return Ok(ProbeDecision::Incomplete(candidate));
        }

        let passed = metrics.is_pass(&self.policy);
        if passed {
            self.passed_probes = self.passed_probes.saturating_add(1);
        } else {
            self.failed_probes = self.failed_probes.saturating_add(1);
        }

        match self.axis {
            SearchAxis::Forward => {
                if passed {
                    self.lowest_passed_us = Some(current_us);
                }
                let at_floor = current_us == self.forward_floor_us;
                if passed && !at_floor {
                    self.current_probe_us = Some(next_lower_candidate_from(
                        current_us,
                        self.forward_floor_us,
                        self.policy.step_us,
                    ));
                    return Ok(ProbeDecision::Passed(
                        self.next_probe().unwrap_or(candidate),
                    ));
                }

                self.selected_forward_us = self.final_forward_us();
                let reverse_floor = self.reverse_floor_us.max(self.selected_forward_us);
                self.axis = SearchAxis::Reverse;
                self.current_probe_us = first_lower_candidate_from(
                    self.stable_profile.long_slot_us,
                    reverse_floor,
                    self.policy.step_us,
                );
                if let Some(next) = self.next_probe() {
                    Ok(ProbeDecision::Continue(next))
                } else {
                    let final_profile = self.make_final_profile();
                    self.final_profile = Some(final_profile);
                    Ok(if passed {
                        ProbeDecision::Passed(final_profile)
                    } else {
                        ProbeDecision::Failed(final_profile)
                    })
                }
            }
            SearchAxis::Reverse => {
                if passed {
                    self.lowest_passed_reverse_us = Some(current_us);
                }
                let reverse_floor = self.reverse_floor_us.max(self.selected_forward_us);
                if passed && current_us != reverse_floor {
                    self.current_probe_us = Some(next_lower_candidate_from(
                        current_us,
                        reverse_floor,
                        self.policy.step_us,
                    ));
                    Ok(ProbeDecision::Continue(
                        self.next_probe().unwrap_or(candidate),
                    ))
                } else {
                    let final_profile = self.make_final_profile();
                    self.current_probe_us = None;
                    self.final_profile = Some(final_profile);
                    Ok(if passed {
                        ProbeDecision::Passed(final_profile)
                    } else {
                        ProbeDecision::Failed(final_profile)
                    })
                }
            }
        }
    }

    /// Lowest short-slot candidate that completed a passing probe.
    pub const fn lowest_passed_us(&self) -> Option<u16> {
        self.lowest_passed_us
    }

    /// Number of completed passing probes.
    pub const fn passed_probes(&self) -> u16 {
        self.passed_probes
    }

    /// Number of completed failed probes.
    pub const fn failed_probes(&self) -> u16 {
        self.failed_probes
    }

    /// Most recent metrics sample supplied for the current candidate.
    pub const fn last_metrics(&self) -> Option<ProbeMetrics> {
        self.last_metrics
    }

    /// The finalized profile, available after the floor, a failure, or an
    /// immediate no-lower-candidate construction.
    pub const fn final_profile(&self) -> Option<CadenceProfile> {
        self.final_profile
    }

    /// Lowest reverse/idle candidate that completed a passing probe.
    pub const fn lowest_passed_reverse_us(&self) -> Option<u16> {
        self.lowest_passed_reverse_us
    }

    fn final_forward_us(&self) -> u16 {
        let base = self
            .lowest_passed_us
            .unwrap_or(self.stable_profile.short_slot_us);
        let margin = self.policy.safety_steps.saturating_mul(self.policy.step_us);
        base.saturating_add(margin)
            .min(self.stable_profile.short_slot_us)
    }

    fn make_final_profile(&self) -> CadenceProfile {
        let short = self.final_forward_us();
        let base_long = self
            .lowest_passed_reverse_us
            .unwrap_or(self.stable_profile.long_slot_us);
        let margin = self.policy.safety_steps.saturating_mul(self.policy.step_us);
        let long = base_long
            .saturating_add(margin)
            .min(self.stable_profile.long_slot_us)
            .max(short);
        CadenceProfile {
            short_slot_us: short,
            long_slot_us: long,
            ..self.stable_profile
        }
    }
}

fn first_lower_candidate_from(current_us: u16, floor_us: u16, step_us: u16) -> Option<u16> {
    if current_us <= floor_us {
        None
    } else {
        Some(next_lower_candidate_from(current_us, floor_us, step_us))
    }
}

fn next_lower_candidate_from(current_us: u16, floor_us: u16, step_us: u16) -> u16 {
    current_us.saturating_sub(step_us).max(floor_us)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stable() -> CadenceProfile {
        CadenceProfile::uniform(600, 8, 2, 0)
    }

    fn traffic() -> TrafficContract {
        TrafficContract::new(32, 24)
    }

    fn pass(policy: &CadenceProbePolicy) -> ProbeMetrics {
        ProbeMetrics::new(policy.probe_superframes, 0, 0)
    }

    #[test]
    fn policy_defaults_are_conservative() {
        let policy = CadenceProbePolicy::default();
        assert_eq!(policy.min_slot_us, 450);
        assert_eq!(policy.step_us, 25);
        assert_eq!(policy.probe_superframes, 8);
        assert_eq!(policy.safety_steps, 2);
    }

    #[test]
    fn exit_policy_requires_an_enabled_trigger() {
        assert!(!CadenceExitPolicy::default().is_enabled());
        assert!(CadenceExitPolicy::new(2, 0).is_enabled());
        assert!(CadenceExitPolicy::new(0, 3).is_enabled());
    }

    #[test]
    fn profile_reports_period_and_superframe_time() {
        let profile = CadenceProfile::new(450, 600, 8, 2, 4);
        assert_eq!(profile.period_slots(), 14);
        assert_eq!(profile.superframe_us(), 8 * 450 + 6 * 600);
        assert!(profile.validate().is_ok());
    }

    #[test]
    fn invalid_profiles_are_rejected() {
        assert_eq!(
            CadenceProfile::new(0, 600, 8, 2, 0).validate(),
            Err(CadenceError::InvalidProfile)
        );
        assert_eq!(
            CadenceProfile::new(650, 600, 8, 2, 0).validate(),
            Err(CadenceError::InvalidProfile)
        );
        assert_eq!(
            CadenceProfile::new(600, 600, 0, 2, 0).validate(),
            Err(CadenceError::InvalidProfile)
        );
        assert_eq!(
            CadenceProfile::new(600, 600, 8, 0, 0).validate(),
            Err(CadenceError::InvalidProfile)
        );
    }

    #[test]
    fn planner_uses_configured_wire_overhead() {
        let planner = CadenceSearch::new(stable(), traffic(), Default::default(), 13).unwrap();
        assert_eq!(planner.protocol_overhead(), 13);
        assert_eq!(planner.forward_wire_len(), 45);
        assert_eq!(planner.reverse_wire_len(), 37);
        assert_eq!(planner.wire_lengths(), (45, 37));
    }

    #[test]
    fn planner_accepts_exact_directional_wire_lengths() {
        let planner = CadenceSearch::new_with_wire_lengths_and_floors(
            stable(),
            traffic(),
            CadenceProbePolicy::new(400, 25, 8, 1),
            38,
            31,
            400,
            450,
        )
        .unwrap();
        assert_eq!(planner.wire_lengths(), (38, 31));
        assert_eq!(planner.protocol_overhead(), 0);
    }

    #[test]
    fn wire_length_overflow_is_rejected() {
        let traffic = TrafficContract::new(u16::MAX, 1);
        let err = CadenceSearch::new(stable(), traffic, Default::default(), 1).unwrap_err();
        assert_eq!(err, CadenceError::WireLengthOverflow);
    }

    #[test]
    fn invalid_policies_are_rejected() {
        let mut policy = CadenceProbePolicy::default();
        policy.step_us = 0;
        assert_eq!(
            CadenceSearch::new(stable(), traffic(), policy, 0).unwrap_err(),
            CadenceError::InvalidPolicy
        );

        let mut policy = CadenceProbePolicy::default();
        policy.probe_superframes = 0;
        assert_eq!(
            CadenceSearch::new(stable(), traffic(), policy, 0).unwrap_err(),
            CadenceError::InvalidPolicy
        );

        let mut policy = CadenceProbePolicy::default();
        policy.min_slot_us = 601;
        assert_eq!(
            CadenceSearch::new(stable(), traffic(), policy, 0).unwrap_err(),
            CadenceError::InvalidPolicy
        );
    }

    #[test]
    fn incomplete_metrics_repeat_current_candidate_without_recording() {
        let policy = CadenceProbePolicy::default();
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let candidate = planner.next_probe().unwrap();
        let metrics = ProbeMetrics::new(policy.probe_superframes - 1, 0, 0);
        assert_eq!(
            planner.record_probe(candidate, metrics),
            Ok(ProbeDecision::Incomplete(candidate))
        );
        assert_eq!(planner.next_probe(), Some(candidate));
        assert_eq!(planner.passed_probes(), 0);
        assert_eq!(planner.failed_probes(), 0);
        assert_eq!(planner.last_metrics(), Some(metrics));
    }

    #[test]
    fn candidates_descend_deterministically_to_clamped_floor() {
        let policy = CadenceProbePolicy::new(455, 50, 2, 1);
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        assert_eq!(planner.candidate_count(), 3);

        let expected = [550, 500, 455];
        for short in expected {
            let candidate = planner.next_probe().unwrap();
            assert_eq!(candidate.short_slot_us, short);
            let decision = planner.record_probe(candidate, pass(&policy)).unwrap();
            if short == 455 {
                assert_eq!(
                    decision,
                    ProbeDecision::Passed(stable().with_short_slot(505))
                );
            } else {
                let ProbeDecision::Passed(next) = decision else {
                    panic!("pass should advance");
                };
                assert_eq!(
                    next.short_slot_us,
                    expected[expected.iter().position(|&v| v == short).unwrap() + 1]
                );
            }
        }
        assert_eq!(planner.final_profile(), Some(stable().with_short_slot(505)));
        assert_eq!(planner.lowest_passed_us(), Some(455));
    }

    #[test]
    fn bidirectional_search_measures_forward_then_reverse() {
        let policy = CadenceProbePolicy::new(300, 50, 2, 0);
        let mut planner =
            CadenceSearch::new_with_floors(stable(), traffic(), policy, 12, 400, 450).unwrap();

        for forward in [550, 500, 450] {
            let candidate = planner.next_probe().unwrap();
            assert_eq!(
                (candidate.short_slot_us, candidate.long_slot_us),
                (forward, 600)
            );
            assert!(matches!(
                planner.record_probe(candidate, pass(&policy)),
                Ok(ProbeDecision::Passed(_))
            ));
        }
        let candidate = planner.next_probe().unwrap();
        assert_eq!(
            (candidate.short_slot_us, candidate.long_slot_us),
            (400, 600)
        );
        assert!(matches!(
            planner.record_probe(candidate, pass(&policy)),
            Ok(ProbeDecision::Continue(_))
        ));

        for reverse in [550, 500, 450] {
            let candidate = planner.next_probe().unwrap();
            assert_eq!(
                (candidate.short_slot_us, candidate.long_slot_us),
                (400, reverse)
            );
            let decision = planner.record_probe(candidate, pass(&policy)).unwrap();
            if reverse == 450 {
                assert_eq!(
                    decision,
                    ProbeDecision::Passed(CadenceProfile::new(400, 450, 8, 2, 0))
                );
            }
        }
        assert_eq!(
            planner.final_profile(),
            Some(CadenceProfile::new(400, 450, 8, 2, 0))
        );
        assert_eq!(planner.lowest_passed_us(), Some(400));
        assert_eq!(planner.lowest_passed_reverse_us(), Some(450));
    }

    #[test]
    fn forward_failure_still_empirically_searches_reverse() {
        let policy = CadenceProbePolicy::new(300, 50, 2, 0);
        let mut planner =
            CadenceSearch::new_with_floors(stable(), traffic(), policy, 12, 400, 450).unwrap();
        let first = planner.next_probe().unwrap();
        planner.record_probe(first, pass(&policy)).unwrap();
        let failed = planner.next_probe().unwrap();
        let next = planner
            .record_probe(failed, ProbeMetrics::new(policy.probe_superframes, 1, 0))
            .unwrap();
        assert_eq!(
            next,
            ProbeDecision::Continue(CadenceProfile::new(550, 550, 8, 2, 0))
        );
    }

    #[test]
    fn full_pass_finalizes_floor_plus_safety_steps() {
        let policy = CadenceProbePolicy::default();
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        for expected in [575, 550, 525, 500, 475, 450] {
            let candidate = planner.next_probe().unwrap();
            assert_eq!(candidate.short_slot_us, expected);
            let decision = planner.record_probe(candidate, pass(&policy)).unwrap();
            assert!(matches!(decision, ProbeDecision::Passed(_)));
        }
        assert_eq!(planner.passed_probes(), 6);
        assert_eq!(planner.failed_probes(), 0);
        assert_eq!(planner.lowest_passed_us(), Some(450));
        assert_eq!(planner.next_probe(), None);
        assert_eq!(planner.final_profile(), Some(stable().with_short_slot(500)));
    }

    #[test]
    fn failure_stops_descent_and_adds_margin_to_lowest_pass() {
        let policy = CadenceProbePolicy::new(450, 25, 2, 1);
        let stable = CadenceProfile::uniform(700, 8, 2, 0);
        let mut planner = CadenceSearch::new(stable, traffic(), policy, 8).unwrap();

        for short in [675, 650, 625] {
            let candidate = planner.next_probe().unwrap();
            assert_eq!(candidate.short_slot_us, short);
            assert!(matches!(
                planner.record_probe(candidate, pass(&policy)),
                Ok(ProbeDecision::Passed(_))
            ));
        }

        let failed = planner.next_probe().unwrap();
        assert_eq!(failed.short_slot_us, 600);
        let metrics = ProbeMetrics::new(policy.probe_superframes, 0, 1);
        assert_eq!(
            planner.record_probe(failed, metrics),
            Ok(ProbeDecision::Failed(stable.with_short_slot(650)))
        );
        assert_eq!(planner.passed_probes(), 3);
        assert_eq!(planner.failed_probes(), 1);
        assert_eq!(planner.lowest_passed_us(), Some(625));
        assert_eq!(planner.final_profile(), Some(stable.with_short_slot(650)));
    }

    #[test]
    fn first_failure_retains_known_stable_profile() {
        let policy = CadenceProbePolicy::default();
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let candidate = planner.next_probe().unwrap();
        let metrics = ProbeMetrics::new(policy.probe_superframes, 1, 0);
        assert_eq!(
            planner.record_probe(candidate, metrics),
            Ok(ProbeDecision::Failed(stable()))
        );
        assert_eq!(planner.passed_probes(), 0);
        assert_eq!(planner.failed_probes(), 1);
        assert_eq!(planner.lowest_passed_us(), None);
        assert_eq!(planner.final_profile(), Some(stable()));
    }

    #[test]
    fn stable_at_floor_finishes_immediately() {
        let policy = CadenceProbePolicy::default();
        let stable = CadenceProfile::uniform(450, 8, 2, 0);
        let planner = CadenceSearch::new(stable, traffic(), policy, 8).unwrap();
        assert_eq!(planner.candidate_count(), 0);
        assert_eq!(planner.next_probe(), None);
        assert_eq!(planner.final_profile(), Some(stable));
    }

    #[test]
    fn wrong_candidate_and_finished_search_are_errors() {
        let policy = CadenceProbePolicy::default();
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let wrong = stable().with_short_slot(574);
        assert_eq!(
            planner.record_probe(wrong, pass(&policy)),
            Err(CadenceError::WrongCandidate)
        );

        let candidate = planner.next_probe().unwrap();
        let metrics = ProbeMetrics::new(policy.probe_superframes, 1, 0);
        assert!(matches!(
            planner.record_probe(candidate, metrics),
            Ok(ProbeDecision::Failed(_))
        ));
        assert_eq!(
            planner.record_probe(candidate, metrics),
            Err(CadenceError::SearchFinished)
        );
    }

    #[test]
    fn metrics_require_all_directions_to_be_clean() {
        let policy = CadenceProbePolicy::new(450, 25, 4, 1);
        assert!(!ProbeMetrics::new(3, 0, 0).is_complete(&policy));
        assert!(!ProbeMetrics::new(4, 1, 0).is_pass(&policy));
        assert!(!ProbeMetrics::new(4, 0, 1).is_pass(&policy));
        assert!(ProbeMetrics::new(5, 0, 0).is_pass(&policy));
    }

    #[test]
    fn safety_margin_never_exceeds_stable_short_slot() {
        let policy = CadenceProbePolicy::new(450, 25, 1, 10);
        let mut planner = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let candidate = planner.next_probe().unwrap();
        let metrics = ProbeMetrics::new(1, 1, 0);
        assert_eq!(
            planner.record_probe(candidate, metrics),
            Ok(ProbeDecision::Failed(stable()))
        );
    }

    #[test]
    fn planner_is_deterministic_for_identical_measurements() {
        let policy = CadenceProbePolicy::default();
        let mut a = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let mut b = CadenceSearch::new(stable(), traffic(), policy, 8).unwrap();
        let samples = [
            ProbeMetrics::new(8, 0, 0),
            ProbeMetrics::new(8, 0, 0),
            ProbeMetrics::new(8, 0, 1),
        ];
        for metrics in samples {
            let pa = a.next_probe().unwrap();
            let pb = b.next_probe().unwrap();
            assert_eq!(pa, pb);
            assert_eq!(a.record_probe(pa, metrics), b.record_probe(pb, metrics));
        }
        assert_eq!(a, b);
    }
}
