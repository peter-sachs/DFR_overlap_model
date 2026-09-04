"""
Layer 1+2: Airspace exposure-rate model.

Computes the RATE (events/day, then annualized) of "Rule 3" encounters:
a currently-Activated delivery drone's remaining route passing through a
currently-active DFR loiter footprint, split into:
  - 3a: overlap during a CRUISE segment (vertical gating applies)
  - 3b: overlap during a HOVER/descent segment (no effective vertical gate)

Explicitly OUT of scope (per user direction):
  - Accepted-state delivery routes preempted or non-intersecting DFR routes
    (Rules 1 and 2) -- assumed handled by strategic deconfliction.
  - The DFR's straight-line 45mph transit phase to the call site -- assumed
    already deconflicted; only used here to time when loiter starts.

Key modeling choices (flagged, not silently assumed):
  - DFR loiter footprint modeled as a STATIC circular disk at the call
    location for this layer (not orbiting motion -- that's the fine
    model's job; this layer only needs a coarse space-time overlap rate).
  - DFR operating altitude drawn uniformly within its declared vertical
    band per call (NOT worst-case-centered on delivery cruise altitude),
    so genuine vertical non-overlap is possible, per user requirement.
    Cruise-phase (3a) overlap additionally requires |dfr_alt - cruise_alt|
    <= vertical_margin_ft (default 50ft, an adjustable assumption).
  - Hover phase (3b) has no vertical gate (drone descends through the
    full altitude range to make the delivery/pickup).
  - 911 call locations are uniform over the whole region regardless of
    which dock responds (not restricted to a sub-region near that dock).
"""

import numpy as np

MPH_TO_MPS = 0.44704
FT_TO_M = 0.3048
MILE_TO_M = 1609.34


def dock_locations(n_docks, region_radius_m, dock_radius_frac=0.6):
    """Evenly angularly spaced docks at a fixed radial distance from hub."""
    r_dock = dock_radius_frac * region_radius_m
    angles = np.linspace(0, 2*np.pi, n_docks, endpoint=False)
    return np.column_stack([r_dock*np.cos(angles), r_dock*np.sin(angles)])


def sample_uniform_in_disk(n, radius, rng):
    r = radius * np.sqrt(rng.uniform(0, 1, n))
    theta = rng.uniform(0, 2*np.pi, n)
    return np.column_stack([r*np.cos(theta), r*np.sin(theta)])


def delivery_arrival_times(n, timing, operating_start_min, operating_end_min, rng):
    """timing: 'peaked' (60% in two 2h windows) or 'uniform'."""
    total_min = operating_end_min - operating_start_min
    if timing == 'uniform':
        return operating_start_min + rng.uniform(0, total_min, n)

    # Peaked: 60% of arrivals in two 2h peak windows (11-13h, 17-19h => 4h
    # total), 40% spread over the remaining 8h.
    peak_windows = [(11*60, 13*60), (17*60, 19*60)]
    peak_total_min = sum(b-a for a, b in peak_windows)
    off_peak_total_min = total_min - peak_total_min

    n_peak = rng.binomial(n, 0.6)
    n_off = n - n_peak

    # split n_peak between the two peak windows proportional to their length
    frac1 = (peak_windows[0][1]-peak_windows[0][0]) / peak_total_min
    n_peak1 = rng.binomial(n_peak, frac1)
    n_peak2 = n_peak - n_peak1

    t_peak1 = rng.uniform(peak_windows[0][0], peak_windows[0][1], n_peak1)
    t_peak2 = rng.uniform(peak_windows[1][0], peak_windows[1][1], n_peak2)

    # off-peak: uniform over operating window EXCLUDING peak windows
    off_peak_segments = []
    prev_end = operating_start_min
    for a, b in peak_windows:
        if a > prev_end:
            off_peak_segments.append((prev_end, a))
        prev_end = b
    if prev_end < operating_end_min:
        off_peak_segments.append((prev_end, operating_end_min))

    seg_lengths = np.array([b-a for a, b in off_peak_segments])
    seg_probs = seg_lengths / seg_lengths.sum()
    seg_choice = rng.choice(len(off_peak_segments), size=n_off, p=seg_probs)
    t_off = np.array([
        rng.uniform(off_peak_segments[s][0], off_peak_segments[s][1])
        for s in seg_choice
    ])

    return np.concatenate([t_peak1, t_peak2, t_off])


def build_delivery_routes(n, route_type, region_radius_m, v_mps, hover_s, rng,
                           cruise_alt_options_ft=(200, 250, 300, 350, 400)):
    """
    Returns a list of route dicts, each describing a piecewise path:
    segments = list of (t_start_s, t_end_s, kind, p_start, p_end)
    kind = 'cruise' or 'hover'. Times are RELATIVE to the route's own start
    (t=0 at departure from hub). Each route also carries its own cruise
    altitude, drawn uniformly from the discrete option set (representing
    altitude-stratified delivery routes -- some flights cruise higher or
    lower than others, giving a genuine chance of full vertical separation
    from a DFR loiter independent of the vertical margin check).
    """
    routes = []
    hub = np.array([0.0, 0.0])
    cruise_alts = rng.choice(cruise_alt_options_ft, size=n)

    if route_type == 'out_and_back':
        dest = sample_uniform_in_disk(n, region_radius_m, rng)
        for i in range(n):
            d = np.linalg.norm(dest[i] - hub)
            t_cruise1 = d / v_mps
            t_hover_end = t_cruise1 + hover_s
            t_cruise2 = t_hover_end + d / v_mps
            segs = [
                (0.0, t_cruise1, 'cruise', hub, dest[i]),
                (t_cruise1, t_hover_end, 'hover', dest[i], dest[i]),
                (t_hover_end, t_cruise2, 'cruise', dest[i], hub),
            ]
            routes.append({'segments': segs, 'total_duration_s': t_cruise2, 'cruise_alt_ft': cruise_alts[i]})

    elif route_type == 'triangular':
        pickup = sample_uniform_in_disk(n, region_radius_m, rng)
        dropoff = sample_uniform_in_disk(n, region_radius_m, rng)
        for i in range(n):
            d1 = np.linalg.norm(pickup[i] - hub)
            d2 = np.linalg.norm(dropoff[i] - pickup[i])
            d3 = np.linalg.norm(hub - dropoff[i])
            t1 = d1 / v_mps
            t1h = t1 + hover_s
            t2 = t1h + d2 / v_mps
            t2h = t2 + hover_s
            t3 = t2h + d3 / v_mps
            segs = [
                (0.0, t1, 'cruise', hub, pickup[i]),
                (t1, t1h, 'hover', pickup[i], pickup[i]),
                (t1h, t2, 'cruise', pickup[i], dropoff[i]),
                (t2, t2h, 'hover', dropoff[i], dropoff[i]),
                (t2h, t3, 'cruise', dropoff[i], hub),
            ]
            routes.append({'segments': segs, 'total_duration_s': t3, 'cruise_alt_ft': cruise_alts[i]})
    else:
        raise ValueError(route_type)

    return routes


def two_segment_intersection_window(pA_start, pA_end, tA0, tA1,
                                     pB_start, pB_end, tB0, tB1, threshold):
    """
    General closest-approach solver for two segments, each moving at its
    own constant velocity (including zero, i.e. stationary/hovering) over
    its own time interval. Returns the absolute time sub-interval during
    which their separation is <= threshold, or None. Degenerates correctly
    to point-vs-moving-point when either segment is stationary (pA_start
    == pA_end), so this single function covers cruise-vs-transit,
    hover-vs-transit, and (in principle) hover-vs-hover.
    """
    t_lo = max(tA0, tB0)
    t_hi = min(tA1, tB1)
    if t_lo >= t_hi:
        return None

    velA = (pA_end - pA_start) / (tA1 - tA0) if tA1 > tA0 else np.zeros(2)
    velB = (pB_end - pB_start) / (tB1 - tB0) if tB1 > tB0 else np.zeros(2)

    posA_tlo = pA_start + velA * (t_lo - tA0)
    posB_tlo = pB_start + velB * (t_lo - tB0)
    r0 = posA_tlo - posB_tlo
    relvel = velA - velB

    dur = t_hi - t_lo
    A = np.dot(relvel, relvel)
    B = 2 * np.dot(r0, relvel)
    C = np.dot(r0, r0) - threshold**2

    if A < 1e-12:
        return (t_lo, t_hi) if C <= 0 else None

    disc = B**2 - 4*A*C
    if disc < 0:
        return None
    sqrt_disc = np.sqrt(disc)
    s_lo = max(0.0, (-B - sqrt_disc) / (2*A))
    s_hi = min(dur, (-B + sqrt_disc) / (2*A))
    if s_lo > s_hi:
        return None
    return (t_lo + s_lo, t_lo + s_hi)


def segment_min_dist_and_window(p_start, p_end, seg_t_start, seg_t_end, target, radius):
    """
    For a cruise segment moving linearly from p_start to p_end over
    [seg_t_start, seg_t_end], find the time sub-interval (absolute, within
    the segment) during which distance to `target` <= radius. Returns
    (t_lo, t_hi) or None if never within radius.
    """
    d = p_end - p_start
    seg_dur = seg_t_end - seg_t_start
    if seg_dur <= 0:
        dist = np.linalg.norm(p_start - target)
        return (seg_t_start, seg_t_end) if dist <= radius else None

    # parametrize position(u) = p_start + u*d, u in [0,1], u = (t-seg_t_start)/seg_dur
    # |p_start + u*d - target|^2 <= radius^2
    f = p_start - target
    A = np.dot(d, d)
    B = 2*np.dot(f, d)
    C = np.dot(f, f) - radius**2

    if A < 1e-12:
        return (seg_t_start, seg_t_end) if C <= 0 else None

    disc = B**2 - 4*A*C
    if disc < 0:
        return None
    sqrt_disc = np.sqrt(disc)
    u_lo = (-B - sqrt_disc) / (2*A)
    u_hi = (-B + sqrt_disc) / (2*A)
    u_lo = max(0.0, u_lo)
    u_hi = min(1.0, u_hi)
    if u_lo > u_hi:
        return None
    return (seg_t_start + u_lo*seg_dur, seg_t_start + u_hi*seg_dur)


def delivery_already_airborne_before_dfr_dispatch(route_abs_start, dfr_transit_start):
    """
    The core Rule 1 / Scenario 1 vs. Scenario 3 gate: a delivery is only a
    valid Scenario 3 candidate if it took off (route_abs_start) BEFORE the
    DFR's dispatch time (dfr_transit_start) -- the moment the DFR's
    intended volume enters the deconfliction system. A delivery planned
    to launch at or after that moment would have been checked against the
    already-known DFR volume during its own pre-flight planning and
    rerouted if there was a conflict (Rule 1), so it can never be a
    genuine Scenario 3 case. Extracted as a standalone function so it can
    be exercised directly by acceptance tests, not just indirectly via a
    full simulated day.
    """
    return route_abs_start < dfr_transit_start


def simulate_one_day(params, rng):
    """
    Returns dict with counts: n_3a, n_3b (for this single simulated day),
    for EACH footprint radius in params['footprint_radii_m'].
    """
    R_m = params['region_radius_miles'] * MILE_TO_M
    v_mps = params['v_delivery_mph'] * MPH_TO_MPS
    v_dfr_transit_mps = params['v_dfr_transit_mph'] * MPH_TO_MPS
    op_start = params['operating_start_min']
    op_end = params['operating_end_min']

    # --- Delivery flights ---
    n_del = params['n_deliveries_per_day']
    t0 = delivery_arrival_times(n_del, params['delivery_timing'], op_start, op_end, rng)
    routes = build_delivery_routes(n_del, params['route_type'], R_m, v_mps,
                                    params['hover_s'], rng,
                                    cruise_alt_options_ft=params.get('cruise_alt_options_ft', (200, 250, 300, 350, 400)))

    # --- DFR events ---
    n_docks = params['n_docks']
    calls_per_dock = params['calls_per_dock_per_day']
    docks = dock_locations(n_docks, R_m)

    dfr_events = []
    for dock_idx in range(n_docks):
        n_calls = calls_per_dock
        call_times = delivery_arrival_times(n_calls, params['dfr_call_timing'], op_start, op_end, rng)
        call_locs = sample_uniform_in_disk(n_calls, R_m, rng)
        for j in range(n_calls):
            transit_dist = np.linalg.norm(call_locs[j] - docks[dock_idx])
            transit_s = transit_dist / v_dfr_transit_mps
            loiter_start_s = call_times[j]*60 + transit_s
            loiter_end_s = loiter_start_s + params['loiter_minutes']*60
            dfr_alt_ft = rng.uniform(params['dfr_band_low_ft'], params['dfr_band_high_ft'])
            dfr_events.append({
                'loc': call_locs[j],
                'dock_loc': docks[dock_idx],
                'transit_start': call_times[j]*60,
                'transit_end': call_times[j]*60 + transit_s,
                't_start': loiter_start_s,
                't_end': loiter_end_s,
                'alt_ft': dfr_alt_ft,
            })

    radii = params['footprint_radii_m']
    counts_3a = {r: 0 for r in radii}
    counts_3b = {r: 0 for r in radii}
    counts_3c = 0
    vmargin = params['vertical_margin_ft']
    cruise_alt_options = params.get('cruise_alt_options_ft', (200, 250, 300, 350, 400))

    # 3c: fixed combined TSE-style buffer (5m delivery + 5m DFR transit),
    # reusing the fine-grained model's convention, NOT part of the loiter
    # footprint radius sweep.
    combined_horiz_buffer_m = params.get('transit_combined_horiz_buffer_m', 10.0)
    combined_vert_buffer_ft = params.get('transit_combined_vert_buffer_m', 10.0) / FT_TO_M

    for i, route in enumerate(routes):
        t0_s = t0[i]*60  # absolute seconds since midnight
        cruise_alt_ft = route['cruise_alt_ft']
        for ev in dfr_events:
            route_abs_start = t0_s
            route_abs_end = t0_s + route['total_duration_s']

            # --- STRATEGIC-DECONFLICTION GATE (fix) ---
            # A delivery that takes off AT OR AFTER the DFR's dispatch time
            # (ev['transit_start'], the moment the DFR's intended volume
            # enters the deconfliction system) would have been checked
            # against that already-known DFR volume during its OWN
            # pre-flight planning and rerouted if there was a conflict --
            # i.e., it is a Rule 1/Scenario 1 case by definition, never a
            # Scenario 3 candidate. Only deliveries already airborne
            # (route_abs_start < ev['transit_start']) can ever be eligible.
            # This condition was previously MISSING entirely -- the code
            # only checked for temporal overlap with the loiter/transit
            # window, regardless of takeoff order, which incorrectly let
            # deliveries launched well into an already-active DFR session
            # count as Scenario 3. This is the root cause of the
            # (incorrect) unbounded linear growth in loiter duration.
            already_airborne = delivery_already_airborne_before_dfr_dispatch(
                route_abs_start, ev['transit_start'])

            # --- 3c: DFR TRANSIT phase vs this delivery route ---
            if already_airborne and not (route_abs_end < ev['transit_start'] or route_abs_start > ev['transit_end']):
                for (seg_t_start, seg_t_end, kind, p_start, p_end) in route['segments']:
                    seg_abs_start = t0_s + seg_t_start
                    seg_abs_end = t0_s + seg_t_end
                    window = two_segment_intersection_window(
                        p_start, p_end, seg_abs_start, seg_abs_end,
                        ev['dock_loc'], ev['loc'], ev['transit_start'], ev['transit_end'],
                        combined_horiz_buffer_m)
                    if window is not None:
                        if kind == 'cruise':
                            vertical_ok_3c = abs(ev['alt_ft'] - cruise_alt_ft) <= combined_vert_buffer_ft
                        else:  # hover -- no vertical gate, same rationale as 3b
                            vertical_ok_3c = True
                        if vertical_ok_3c:
                            counts_3c += 1

            if not already_airborne or route_abs_end < ev['t_start'] or route_abs_start > ev['t_end']:
                continue

            vertical_ok_3a = abs(ev['alt_ft'] - cruise_alt_ft) <= vmargin

            for (seg_t_start, seg_t_end, kind, p_start, p_end) in route['segments']:
                seg_abs_start = t0_s + seg_t_start
                seg_abs_end = t0_s + seg_t_end
                if seg_abs_end < ev['t_start'] or seg_abs_start > ev['t_end']:
                    continue

                for radius in radii:
                    if kind == 'cruise':
                        if not vertical_ok_3a:
                            continue
                        window = segment_min_dist_and_window(
                            p_start, p_end, seg_abs_start, seg_abs_end, ev['loc'], radius)
                        if window is not None:
                            lo, hi = window
                            if max(lo, ev['t_start']) <= min(hi, ev['t_end']):
                                counts_3a[radius] += 1
                    else:  # hover
                        dist = np.linalg.norm(p_start - ev['loc'])
                        if dist <= radius:
                            lo, hi = seg_abs_start, seg_abs_end
                            if max(lo, ev['t_start']) <= min(hi, ev['t_end']):
                                counts_3b[radius] += 1

    return counts_3a, counts_3b, counts_3c


DEFAULT_PARAMS = dict(
    region_radius_miles=5.0,
    v_delivery_mph=45.0,
    v_dfr_transit_mph=45.0,
    hover_s=45.0,
    operating_start_min=8*60,
    operating_end_min=20*60,
    route_type='out_and_back',
    delivery_timing='peaked',
    n_deliveries_per_day=1000,
    n_docks=3,
    calls_per_dock_per_day=8,
    dfr_call_timing='uniform',
    loiter_minutes=15.0,
    footprint_radii_m=[50, 150, 300, 500],
    dfr_band_low_ft=100.0,
    dfr_band_high_ft=300.0,
    cruise_alt_options_ft=(200.0, 250.0, 300.0, 350.0, 400.0),
    vertical_margin_ft=50.0,
)
