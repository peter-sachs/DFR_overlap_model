"""
UAS Midair Collision Probability -- Orbiting-B Model (CORRECTED v3)

Replaces the "dwell and reposition over a filled disk" model of Drone B's
motion (mc_prototype.run_monte_carlo) with a continuous orbital model:
Drone B flies a circle of radius r around a fixed center point at constant
tangential speed v_B. r varies orbit-to-orbit (drawn with a size-biased
distribution, since larger orbits take proportionally longer to complete
and are therefore more likely to be the "active" orbit when Drone A
happens to transit -- an inspection-paradox correction). Altitude is fixed
per orbit and may change orbit-to-orbit.

THREE CORRECTIONS FROM THE ORIGINAL VERSION (documented, not silent):

1. HORIZONTAL ALIGNMENT: the original version assumed Drone A's flight
   path always passes through the EXACT center of Drone B's orbit, every
   single trial -- a genuine worst-case with no exceptions. This is now
   understood to be too extreme: the coarse exposure-rate layer only ever
   confirms "the route came within the footprint radius," never "passed
   exactly through center." The corrected model draws the path's
   cross-track offset from the orbit center from the SAME uniform-disk
   convention used to fix the analogous problem in the hover model
   (hover_vs_orbit.py): a point is drawn uniformly within the footprint
   disk (radius = r_max_m) and its cross-track component is used as the
   offset. This gives the correct semicircle-shaped marginal distribution
   (denser toward the center, tapering to zero at the edges) rather than
   either the old fixed-at-zero assumption or a naive flat uniform. A
   small Gaussian (the original TSE containment bound) is added on top,
   representing genuine navigation/positioning noise around wherever the
   path's offset actually is.

2. VERTICAL PLACEMENT: the original version placed Drone B's active
   altitude sub-band worst-case-centered on Drone A's cruise altitude,
   narrowed to half the total declared band -- guaranteeing vertical
   overlap was always geometrically favorable. The coarse exposure-rate
   model never made this assumption (it draws DFR altitude uniformly
   across the FULL declared band, independent of delivery altitude). The
   fine-grained model is now corrected to match: B's altitude is drawn
   uniformly across the full declared band, with no narrowing or
   centering.

3. CRUISE-ALTITUDE SCOPE MATCH (this version): the original version
   computed p_3a at a single fixed cruise altitude (200ft in all reference
   runs), while the coarse exposure-rate model varies Drone A's cruise
   altitude across five discrete values (200/250/300/350/400 ft) and
   applies its own vertical gate (|DFR_alt - cruise_alt| <= 50ft) when
   deciding whether a coarse Scenario 3a event occurs at all. Computing
   p_3a at a single fixed altitude (and one that happens to be the most
   favorable of the five, at that) does not correctly represent "the
   average Scenario 3a event," so this model now draws cruise altitude
   from the same five-value set and applies the SAME coarse gate as a
   rejection filter: trials that would not have passed the coarse gate at
   all are excluded from both the numerator and denominator, rather than
   being silently counted as non-collisions. This reproduces the correct
   Bayesian-conditional distribution of cruise altitude given a coarse
   Scenario 3a event, without needing to hand-derive posterior weights
   (which are not uniform -- 200/250ft dominate, 350/400ft never pass the
   coarse gate at all). Approach D in the working spec's terminology.

Because B's true path is a circle (not a straight line), the horizontal
distance between A (straight-line motion) and B (circular motion) is a
transcendental function of time -- it has no closed-form root in general.
This is evaluated via a two-stage coarse-then-refine numerical search,
validated against brute-force grids elsewhere in this project.
"""

import numpy as np


def run_monte_carlo_orbit(
    v_A_mps,            # Drone A cruise speed (m/s)
    W_A_m,               # Drone A cross-track half-width, 95% bound (m)
    H_A_m,               # Drone A vertical half-height, 95% bound (m)
    r_min_m,             # Drone B orbit radius range, lower bound (m)
    r_max_m,             # Drone B orbit radius range, upper bound (m)
    v_B_mps,              # Drone B orbital (tangential) speed (m/s), fixed per scenario
    B_total_low_m,        # Drone B DECLARED volume, total vertical lower bound (m AGL)
    B_total_high_m,       # Drone B DECLARED volume, total vertical upper bound (m AGL)
    S_h_m,                 # Horizontal collision/sNMAC threshold (m)
    S_v_m,                 # Vertical collision/sNMAC threshold (m)
    cruise_alt_options_ft=(200.0, 250.0, 300.0, 350.0, 400.0),  # matches exposure_model.py
    vertical_margin_ft=50.0,   # coarse-level gate margin, matches exposure_model.py
    n_trials=200_000,
    time_samples=1000,
    batch_size=20_000,
    seed=None,
):
    rng = np.random.default_rng(seed)
    FT_TO_M = 0.3048

    sigma_h_A = W_A_m / 1.96
    sigma_v_A = H_A_m / 1.96
    # FIX #2: no worst-case narrowing/centering -- B's altitude is drawn
    # uniformly across the FULL declared band, matching the coarse
    # exposure-rate model's convention exactly.
    band_low, band_high = B_total_low_m, B_total_high_m
    vertical_margin_m = vertical_margin_ft * FT_TO_M

    n_valid_total = 0
    n_hits_total = 0

    for start in range(0, n_trials, batch_size):
        end = min(start + batch_size, n_trials)
        n = end - start

        # --- Size-biased orbit radius: pdf(r) ~ r over [r_min, r_max], since
        # orbit period = 2*pi*r/v_B is proportional to r at fixed speed, and
        # a randomly-timed transit is more likely to land during a longer
        # (larger-radius) orbit.
        U = rng.uniform(0.0, 1.0, size=n)
        r = np.sqrt(r_min_m**2 + U * (r_max_m**2 - r_min_m**2))

        theta0 = rng.uniform(0.0, 2 * np.pi, size=n)   # B's phase at A's crossing instant (t=0)

        # FIX #1: A's cross-track offset from the orbit center is no longer
        # fixed near zero (worst-case exact alignment). Draw a point
        # uniformly within the footprint disk (radius = r_max_m, same
        # convention as hover_vs_orbit.py) and use its cross-track
        # component -- this gives the correct semicircle-shaped marginal
        # (denser toward center, tapering at the edges), not a naive flat
        # uniform. A small Gaussian on top represents genuine navigation
        # noise around wherever the path's true offset actually is.
        disk_r = r_max_m * np.sqrt(rng.uniform(0.0, 1.0, size=n))
        disk_theta = rng.uniform(0.0, 2 * np.pi, size=n)
        path_offset = disk_r * np.sin(disk_theta)  # cross-track component only
        dx_A = path_offset + rng.normal(0.0, sigma_h_A, size=n)

        # FIX #3 (Approach D): draw cruise altitude from the same discrete
        # set the coarse model uses, then apply the SAME coarse-level
        # vertical gate as a rejection filter. Trials failing this gate
        # would never have registered as a coarse Scenario 3a event in the
        # first place, so they are excluded from both numerator and
        # denominator below -- not counted as non-collisions.
        cruise_alt_ft = rng.choice(cruise_alt_options_ft, size=n)
        alt_A_m = cruise_alt_ft * FT_TO_M

        dz_A = rng.normal(0.0, sigma_v_A, size=n)
        Z_A = alt_A_m + dz_A

        bz = rng.uniform(band_low, band_high, size=n)   # fixed for this orbit

        coarse_gate_pass = np.abs(bz - alt_A_m) <= vertical_margin_m

        omega = v_B_mps / r                              # rad/s, shape (n,)
        half_range = (r + S_h_m) / v_A_mps                # s, candidate window half-length

        def horiz_min_dist(t_grid):
            u_A = v_A_mps * t_grid
            phi = theta0[:, None] + omega[:, None] * t_grid
            bx = r[:, None] * np.cos(phi)
            by = r[:, None] * np.sin(phi)
            return np.sqrt((u_A - bx) ** 2 + (dx_A[:, None] - by) ** 2)

        # --- Stage 1: coarse scan across the full candidate window ---
        M1 = 500
        frac1 = np.linspace(-1.0, 1.0, M1)[None, :]
        t_grid1 = frac1 * half_range[:, None]
        dist1 = horiz_min_dist(t_grid1)
        coarse_min_idx = np.argmin(dist1, axis=1)
        coarse_min_val = dist1[np.arange(n), coarse_min_idx]
        coarse_spacing = (2 * half_range) / (M1 - 1)
        t_at_coarse_min = t_grid1[np.arange(n), coarse_min_idx]

        # --- Stage 2: fine local refinement around the coarse minimum ---
        M2 = 200
        local_half_width = 2 * coarse_spacing  # a couple of coarse cells either side
        frac2 = np.linspace(-1.0, 1.0, M2)[None, :]
        t_grid2 = t_at_coarse_min[:, None] + frac2 * local_half_width[:, None]
        dist2 = horiz_min_dist(t_grid2)
        fine_min_val = np.min(dist2, axis=1)

        true_min_dist = np.minimum(coarse_min_val, fine_min_val)
        horiz_hit = true_min_dist <= S_h_m
        vert_hit = np.abs(Z_A - bz) <= S_v_m

        n_valid_total += int(coarse_gate_pass.sum())
        n_hits_total += int((coarse_gate_pass & horiz_hit & vert_hit).sum())

    # p_hat is conditional on having passed the coarse gate -- N below is
    # the count of gate-passing trials, NOT n_trials, since this is
    # rejection sampling, not a simple mean over all draws.
    N = n_valid_total
    p_hat = n_hits_total / N if N > 0 else 0.0
    z = 1.96
    denom = 1 + z**2 / N
    center = (p_hat + z**2 / (2 * N)) / denom
    half_width = (z * np.sqrt((p_hat * (1 - p_hat) / N) + (z**2 / (4 * N**2)))) / denom
    ci_low, ci_high = max(0.0, center - half_width), min(1.0, center + half_width)

    diagnostics = {
        "n_trials": n_trials,
        "n_valid_after_coarse_gate": N,
        "coarse_gate_pass_rate": N / n_trials,
        "time_samples": time_samples,
        "B_band_low_m": band_low,
        "B_band_high_m": band_high,
    }

    return p_hat, (ci_low, ci_high), diagnostics


if __name__ == "__main__":
    MPH_TO_MPS = 0.44704
    FT_TO_M = 0.3048

    from mc_prototype import snmac_thresholds
    s_h_contact, s_h_snmac, s_v_contact, s_v_snmac = snmac_thresholds(
        a_h_max=3.0, a_v_max=0.5,
        b_h_max=0.254, b_v_max=0.076,
    )

    base = dict(
        v_A_mps=45 * MPH_TO_MPS,
        W_A_m=5.0,
        H_A_m=5.0,
        r_min_m=50.0,
        r_max_m=500.0,
        v_B_mps=45 * MPH_TO_MPS,   # midpoint of 30-60 mph range, placeholder
        B_total_low_m=100 * FT_TO_M,
        B_total_high_m=300 * FT_TO_M,
        n_trials=200_000,
        time_samples=1000,
        seed=42,
    )

    print("Reference orbit scenario:")
    p_s, ci_s, d_s = run_monte_carlo_orbit(S_h_m=s_h_snmac, S_v_m=s_v_snmac, **base)
    p_c, ci_c, d_c = run_monte_carlo_orbit(S_h_m=s_h_contact, S_v_m=s_v_contact, **base)
    print(f"  sNMAC:   p = {p_s:.6e}  (95% CI [{ci_s[0]:.3e}, {ci_s[1]:.3e}])")
    print(f"  Contact: p = {p_c:.6e}  (95% CI [{ci_c[0]:.3e}, {ci_c[1]:.3e}])")
    print(f"  Worst-case B band: {d_s['B_band_low_m']/FT_TO_M:.1f}-{d_s['B_band_high_m']/FT_TO_M:.1f} ft")
    print(f"  Coarse-gate pass rate: {d_s['coarse_gate_pass_rate']*100:.1f}% "
          f"({d_s['n_valid_after_coarse_gate']}/{d_s['n_trials']} trials)")

    # --- Validation: does the two-stage method match a brute-force fine grid? ---
    print()
    print("Validation against brute-force fine grid (5000 samples), 2000 random trials:")
    rng_val = np.random.default_rng(999)
    n_val = 2000
    r_val = np.sqrt(base["r_min_m"]**2 + rng_val.uniform(0, 1, n_val) * (base["r_max_m"]**2 - base["r_min_m"]**2))
    theta0_val = rng_val.uniform(0, 2 * np.pi, n_val)
    sigma_h_A_val = base["W_A_m"] / 1.96
    dx_A_val = rng_val.normal(0, sigma_h_A_val, n_val)
    v_A_val = base["v_A_mps"]
    v_B_val = base["v_B_mps"]

    for S_h_test, label in [(s_h_snmac, "sNMAC"), (s_h_contact, "contact")]:
        omega_val = v_B_val / r_val
        half_range_val = (r_val + S_h_test) / v_A_val

        # Brute-force fine grid
        M_fine = 5000
        frac_fine = np.linspace(-1.0, 1.0, M_fine)[None, :]
        t_fine = frac_fine * half_range_val[:, None]
        u_A_fine = v_A_val * t_fine
        phi_fine = theta0_val[:, None] + omega_val[:, None] * t_fine
        bx_fine = r_val[:, None] * np.cos(phi_fine)
        by_fine = r_val[:, None] * np.sin(phi_fine)
        dist_fine = np.sqrt((u_A_fine - bx_fine)**2 + (dx_A_val[:, None] - by_fine)**2)
        min_fine = np.min(dist_fine, axis=1)

        # Two-stage method
        M1, M2 = 500, 200
        frac1 = np.linspace(-1.0, 1.0, M1)[None, :]
        t1 = frac1 * half_range_val[:, None]
        u_A1 = v_A_val * t1
        phi1 = theta0_val[:, None] + omega_val[:, None] * t1
        bx1 = r_val[:, None] * np.cos(phi1)
        by1 = r_val[:, None] * np.sin(phi1)
        dist1_val = np.sqrt((u_A1 - bx1)**2 + (dx_A_val[:, None] - by1)**2)
        idx_min = np.argmin(dist1_val, axis=1)
        coarse_min = dist1_val[np.arange(n_val), idx_min]
        spacing = (2 * half_range_val) / (M1 - 1)
        t_at_min = t1[np.arange(n_val), idx_min]

        local_hw = 2 * spacing
        frac2 = np.linspace(-1.0, 1.0, M2)[None, :]
        t2 = t_at_min[:, None] + frac2 * local_hw[:, None]
        u_A2 = v_A_val * t2
        phi2 = theta0_val[:, None] + omega_val[:, None] * t2
        bx2 = r_val[:, None] * np.cos(phi2)
        by2 = r_val[:, None] * np.sin(phi2)
        dist2_val = np.sqrt((u_A2 - bx2)**2 + (dx_A_val[:, None] - by2)**2)
        fine_min = np.min(dist2_val, axis=1)
        two_stage_min = np.minimum(coarse_min, fine_min)

        # Compare classification (below threshold or not) and magnitude
        agree = (two_stage_min <= S_h_test) == (min_fine <= S_h_test)
        max_abs_err = np.max(np.abs(two_stage_min - min_fine))
        n_disagree = np.sum(~agree)
        print(f"  [{label}] threshold classification agreement: {agree.mean()*100:.3f}%  "
              f"({n_disagree}/{n_val} disagree), max |min_dist error| = {max_abs_err:.4f} m")
