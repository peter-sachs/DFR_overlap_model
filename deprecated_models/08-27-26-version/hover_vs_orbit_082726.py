"""
Fine-grained sub-model for Scenario 3b: a delivery drone descending to make
a pickup/dropoff, vs. an ORBITING DFR drone.

CORRECTED DESIGN (this version): Drone A's horizontal position is a fixed
point for the 45-second hover -- drawn uniformly within the footprint
disk (radius = footprint_radius_m, shared with r_max), plus a small
isotropic Gaussian jitter (sigma = W_A/1.96, GPS/hover-hold precision).
This has been the correct design since the hover model was first built;
an earlier draft placed A at the orbit's exact center, which was caught
and fixed before shipping (the 50m orbit-radius floor made that placement
structurally impossible to ever collide, not merely conservative).

ITEM 3 FIX (this version): Drone A's ALTITUDE, unlike its horizontal
position, is now modeled as time-varying rather than fixed. The coarse
exposure-rate model exempts Scenario 3b from any vertical gate on the
reasoning that the drone (or its payload) physically sweeps through the
full altitude range while descending to deliver/collect the package and
climbing back to cruise altitude -- it is very likely to pass through
whatever altitude the DFR happens to be at, at some point during the
45-second window. The original version of this file did not implement
that sweep: it checked a single fixed hover altitude against a normal
vertical gate, which is a materially different (and more restrictive)
check than "did the drone pass through this altitude at some point."

This version models Z_A(t) as a linear descent from cruise altitude to
ground level (0 ft AGL) over the first half of the hover window, then a
linear ascent back to cruise altitude over the second half -- the widest
plausible sweep, chosen deliberately (consistent with this project's
general preference for not underestimating exposure) rather than
assuming a smaller, unvalidated minimum descent altitude. Cruise altitude
is drawn from the same five discrete values the coarse model uses
(200/250/300/350/400 ft), uniformly -- no rejection filter is applied
here, unlike the cruise (Scenario 3a) fix, because the coarse model
applies NO vertical gate at all for Scenario 3b, so all five values are
equally likely a priori and none are disqualified.

WHY THIS REQUIRES MORE THAN "ADD A VERTICAL LOOP": once Z_A varies with
time, a collision requires horizontal AND vertical proximity AT THE SAME
INSTANT -- checking "horizontal gets close at some point" and "vertical
gets close at some point" as two independent booleans and ANDing them
(valid in the original fixed-altitude version, since the vertical
condition was then time-invariant) is no longer correct. This version
instead:
  1. Solves EXACTLY, in closed form, for the (up to two) time windows
     during which |Z_A(t) - bz| <= S_v holds -- possible because Z_A(t)
     is a known piecewise-linear function of time, not transcendental.
     One window can occur during descent, one during ascent, since a
     V-shaped altitude profile can cross a fixed target band at most
     once per leg.
  2. Runs the existing two-stage horizontal search (still needed, since
     B's circular motion is transcendental) RESTRICTED to those exact
     windows, rather than across the full 45-second duration -- this
     also improves horizontal resolution for free, since the windows are
     typically far shorter than the full duration.
  3. Registers a collision if the horizontal minimum distance within
     EITHER exact window is <= S_h (vertical eligibility is guaranteed
     throughout each window by construction, not re-checked pointwise).

Validated against a brute-force fine grid evaluating the JOINT condition
directly across the full window (see validate_hover_v3.py) -- this is a
stronger validation than checking horizontal and vertical separately,
since it is precisely the joint-timing correctness that changed.
"""

import numpy as np


def run_hover_vs_orbit(
    hover_duration_s,
    footprint_radius_m,  # coarse layer's footprint radius -- A's hover point is
                          # uniform within this disk (we don't know exactly where
                          # within it the hover occurred), SHARED with r_max
    W_A_m,            # small-scale hover positional uncertainty, 95% bound (m)
    r_min_m,
    r_max_m,
    v_B_mps,
    B_total_low_m,
    B_total_high_m,
    S_h_m,
    S_v_m,
    cruise_alt_options_ft=(200.0, 250.0, 300.0, 350.0, 400.0),  # matches exposure_model.py
    n_trials=500_000,
    batch_size=10_000,
    seed=None,
):
    rng = np.random.default_rng(seed)
    sigma_h = W_A_m / 1.96
    FT_TO_M = 0.3048

    band_low, band_high = B_total_low_m, B_total_high_m

    collisions = np.zeros(n_trials, dtype=bool)

    def horiz_min_in_window(t_lo, t_hi, theta0, omega, r, dx_A, dy_A, n):
        """Two-stage horizontal search restricted to [t_lo, t_hi] per trial.
        Returns +inf for trials where the window is degenerate (t_hi<=t_lo)."""
        valid = t_hi > t_lo
        span = np.where(valid, t_hi - t_lo, 1.0)

        M1 = 200
        frac1 = np.linspace(0.0, 1.0, M1)[None, :]
        t_grid1 = t_lo[:, None] + frac1 * span[:, None]
        phi1 = theta0[:, None] + omega[:, None] * t_grid1
        bx1 = r[:, None] * np.cos(phi1)
        by1 = r[:, None] * np.sin(phi1)
        dist1 = np.sqrt((dx_A[:, None] - bx1) ** 2 + (dy_A[:, None] - by1) ** 2)
        idx_min = np.argmin(dist1, axis=1)
        coarse_min = dist1[np.arange(n), idx_min]

        spacing = span / (M1 - 1)
        t_at_min = t_grid1[np.arange(n), idx_min]
        M2 = 100
        local_hw = 2 * spacing
        frac2 = np.linspace(-1.0, 1.0, M2)[None, :]
        t_grid2 = np.clip(t_at_min[:, None] + frac2 * local_hw[:, None],
                           t_lo[:, None], t_hi[:, None])
        phi2 = theta0[:, None] + omega[:, None] * t_grid2
        bx2 = r[:, None] * np.cos(phi2)
        by2 = r[:, None] * np.sin(phi2)
        dist2 = np.sqrt((dx_A[:, None] - bx2) ** 2 + (dy_A[:, None] - by2) ** 2)
        fine_min = np.min(dist2, axis=1)

        true_min = np.minimum(coarse_min, fine_min)
        return np.where(valid, true_min, np.inf)

    for start in range(0, n_trials, batch_size):
        end = min(start + batch_size, n_trials)
        n = end - start

        U = rng.uniform(0.0, 1.0, size=n)
        r = np.sqrt(r_min_m**2 + U * (r_max_m**2 - r_min_m**2))
        theta0 = rng.uniform(0.0, 2 * np.pi, size=n)
        omega = v_B_mps / r

        A_r = footprint_radius_m * np.sqrt(rng.uniform(0.0, 1.0, size=n))
        A_theta = rng.uniform(0.0, 2 * np.pi, size=n)
        dx_A = A_r * np.cos(A_theta) + rng.normal(0.0, sigma_h, size=n)
        dy_A = A_r * np.sin(A_theta) + rng.normal(0.0, sigma_h, size=n)
        bz = rng.uniform(band_low, band_high, size=n)

        # Cruise altitude: uniform over the five discrete values, no
        # rejection filter (no coarse vertical gate exists for Scenario 3b).
        cruise_alt_ft = rng.choice(cruise_alt_options_ft, size=n)
        cruise_alt_m = cruise_alt_ft * FT_TO_M

        half_T = hover_duration_s / 2.0
        rate = cruise_alt_m / half_T  # m/s, linear descent/ascent rate

        # --- Exact closed-form vertical-eligible windows ---
        # Descent: Z(t) = cruise_alt_m - rate*t, t in [0, half_T]
        t_desc_lo = np.clip((cruise_alt_m - bz - S_v_m) / rate, 0.0, half_T)
        t_desc_hi = np.clip((cruise_alt_m - bz + S_v_m) / rate, 0.0, half_T)

        # Ascent: Z(t) = rate*(t-half_T), t in [half_T, hover_duration_s]
        t_asc_lo = np.clip(half_T + (bz - S_v_m) / rate, half_T, hover_duration_s)
        t_asc_hi = np.clip(half_T + (bz + S_v_m) / rate, half_T, hover_duration_s)

        min_desc = horiz_min_in_window(t_desc_lo, t_desc_hi, theta0, omega, r, dx_A, dy_A, n)
        min_asc = horiz_min_in_window(t_asc_lo, t_asc_hi, theta0, omega, r, dx_A, dy_A, n)
        true_min_dist = np.minimum(min_desc, min_asc)

        collisions[start:end] = true_min_dist <= S_h_m

    p_hat = collisions.mean()
    N = n_trials
    z = 1.96
    denom = 1 + z**2 / N
    center = (p_hat + z**2 / (2 * N)) / denom
    half_width = (z * np.sqrt((p_hat * (1 - p_hat) / N) + (z**2 / (4 * N**2)))) / denom
    ci_low, ci_high = max(0.0, center - half_width), min(1.0, center + half_width)

    diagnostics = {
        "n_trials": N,
        "B_band_low_m": band_low,
        "B_band_high_m": band_high,
    }

    return p_hat, (ci_low, ci_high), diagnostics
