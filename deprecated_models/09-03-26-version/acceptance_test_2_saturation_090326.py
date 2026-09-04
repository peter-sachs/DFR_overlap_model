"""
Acceptance Test 2: sweep loiter duration and confirm Scenario 3a/3b counts
(a) grow with loiter duration up to the maximum possible delivery flight
duration, then (b) go flat.

Uses many simulated days per loiter value to get a stable rate estimate,
across a fine grid of loiter durations straddling the predicted
saturation points:
  out-and-back: 14.083 min
  triangular (operationally-representative 15mi max): 21.5 min
  triangular (true mathematical 20mi corner-case max): 28.167 min (for
     reference, to see if a residual tail extends that far)
"""
import numpy as np
from exposure_model import simulate_one_day, DEFAULT_PARAMS

def run_sweep(route_type, loiter_values_min, n_days=400, seed=0):
    rng = np.random.default_rng(seed)
    results = []
    for loiter_min in loiter_values_min:
        params = dict(DEFAULT_PARAMS)
        params['route_type'] = route_type
        params['loiter_minutes'] = loiter_min
        params['n_deliveries_per_day'] = 2000   # higher volume for stable counts
        params['calls_per_dock_per_day'] = 20
        total_3a = {r: 0 for r in params['footprint_radii_m']}
        total_3b = {r: 0 for r in params['footprint_radii_m']}
        for _ in range(n_days):
            c3a, c3b, c3c = simulate_one_day(params, rng)
            for r in params['footprint_radii_m']:
                total_3a[r] += c3a[r]
                total_3b[r] += c3b[r]
        # focus on radius=300 for the growth-curve check
        results.append((loiter_min, total_3a[300]/n_days, total_3b[300]/n_days))
    return results

print("=== OUT-AND-BACK: sweep loiter duration (predicted flat point: 14.083 min) ===")
oab_loiters = [2, 4, 6, 8, 10, 12, 13, 14, 14.083, 14.5, 15, 18, 20, 25, 30]
oab_results = run_sweep('out_and_back', oab_loiters, n_days=500, seed=1)
for loiter, r3a, r3b in oab_results:
    print(f"  loiter={loiter:>7.3f} min:  3a/day={r3a:.4f}  3b/day={r3b:.4f}")

print()
print("=== TRIANGULAR: sweep loiter duration (operational flat point: 21.5 min; true max: 28.167 min) ===")
tri_loiters = [2, 6, 10, 14, 18, 20, 21, 21.5, 22, 24, 26, 28, 28.167, 28.5, 30]
tri_results = run_sweep('triangular', tri_loiters, n_days=500, seed=2)
for loiter, r3a, r3b in tri_results:
    print(f"  loiter={loiter:>7.3f} min:  3a/day={r3a:.4f}  3b/day={r3b:.4f}")
