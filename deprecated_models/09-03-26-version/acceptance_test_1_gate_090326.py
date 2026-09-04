"""
Acceptance Test 1: deliberately inject delivery flights that take off
AFTER a DFR's dispatch time, and confirm they are excluded from
Scenario 3a/3b/3c (i.e., correctly treated as Scenario 1 -- would have
been rerouted before launch, per Rule 1).

Also runs a positive control: an otherwise-IDENTICAL delivery that takes
off BEFORE DFR dispatch, guaranteed to geometrically and temporally
intersect, to confirm the test setup itself is valid (i.e., that this
harness would have caught the bug if the fix were reverted).
"""
import numpy as np
from exposure_model import (
    delivery_already_airborne_before_dfr_dispatch,
    segment_min_dist_and_window,
)

FT_TO_M = 0.3048

# --- Construct a DFR event directly (bypassing random generation) ---
dfr_loc = np.array([1000.0, 0.0])          # DFR loiter/call location
dfr_dock_loc = np.array([0.0, 0.0])        # dock at origin
transit_start = 1000.0                      # DFR dispatched at t=1000s
transit_end = 1100.0                        # 100s transit to scene
t_start = transit_end                        # loiter begins
t_end = t_start + 15 * 60                    # 15-minute loiter
alt_ft = 200.0
radius = 300.0
vmargin = 50.0

# --- Construct a delivery route directly: a straight cruise segment that
# passes EXACTLY through dfr_loc, guaranteed to intersect if temporally
# eligible. ---
p_start = np.array([1000.0, -5000.0])
p_end = np.array([1000.0, 5000.0])
cruise_alt_ft = 200.0  # matches DFR altitude exactly -- vertical gate trivially passes

def check_eligibility_and_geometry(route_abs_start, seg_duration_s, label):
    seg_t_start = 0.0
    seg_t_end = seg_duration_s
    seg_abs_start = route_abs_start + seg_t_start
    seg_abs_end = route_abs_start + seg_t_end

    eligible = delivery_already_airborne_before_dfr_dispatch(route_abs_start, transit_start)

    # Replicate exactly the production gate used in simulate_one_day:
    # if not eligible, or no temporal overlap with loiter window, skip.
    route_abs_end = seg_abs_end
    skip = (not eligible) or (route_abs_end < t_start) or (route_abs_start > t_end)

    counted = False
    if not skip:
        vertical_ok = abs(alt_ft - cruise_alt_ft) <= vmargin
        if vertical_ok:
            window = segment_min_dist_and_window(p_start, p_end, seg_abs_start, seg_abs_end, dfr_loc, radius)
            if window is not None:
                lo, hi = window
                if max(lo, t_start) <= min(hi, t_end):
                    counted = True

    print(f"{label}: route_abs_start={route_abs_start:.0f}s  eligible(pre-dispatch)={eligible}  "
          f"skip_gate={skip}  counted_as_scenario3={counted}")
    return eligible, counted

print("=== Test 1: Post-dispatch injection (should be Scenario 1 -- excluded) ===")
# Delivery takes off AFTER DFR dispatch (t=1000s), but its cruise segment
# geometrically passes right through the DFR's loiter point during the
# loiter window -- if the bug were still present, this WOULD be counted.
eligible_late, counted_late = check_eligibility_and_geometry(
    route_abs_start=1050.0,   # takes off 50s AFTER DFR dispatch (t=1000s)
    seg_duration_s=400.0,     # cruises through t_start..t_end window
    label="Late-departing delivery (t0=1050s, DFR dispatched t=1000s)")

assert eligible_late == False, "FAIL: late delivery should NOT be eligible"
assert counted_late == False, "FAIL: late delivery should NOT be counted as Scenario 3"
print("PASS: late-departing delivery correctly excluded (Scenario 1)\n")

print("=== Positive control: pre-dispatch departure, otherwise identical ===")
# Same geometry, but takes off BEFORE DFR dispatch -- already airborne,
# genuine Scenario 3 candidate. Must extend long enough to still be
# flying through the loiter window.
eligible_early, counted_early = check_eligibility_and_geometry(
    route_abs_start=500.0,    # takes off well BEFORE DFR dispatch (t=1000s)
    seg_duration_s=1500.0,    # long enough to still be in flight during loiter
    label="Pre-dispatch delivery (t0=500s, DFR dispatched t=1000s)")

assert eligible_early == True, "FAIL: pre-dispatch delivery should be eligible"
assert counted_early == True, "FAIL: pre-dispatch delivery should be counted (positive control)"
print("PASS: pre-dispatch delivery correctly counted (proves harness would catch the bug if reverted)\n")

print("=== Edge case: takeoff EXACTLY at dispatch time (boundary) ===")
eligible_exact, counted_exact = check_eligibility_and_geometry(
    route_abs_start=1000.0,   # exactly equal to transit_start
    seg_duration_s=400.0,
    label="Exact-boundary delivery (t0=1000s == DFR dispatch t=1000s)")
assert eligible_exact == False, "FAIL: exact-boundary delivery should NOT be eligible (strict <, not <=)"
print("PASS: exact-boundary case correctly excluded (strict inequality enforced)\n")

print("ALL ACCEPTANCE TEST 1 CHECKS PASSED")
