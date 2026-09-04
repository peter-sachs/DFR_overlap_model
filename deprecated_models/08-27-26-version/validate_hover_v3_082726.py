import numpy as np
from mc_prototype import snmac_thresholds

MPH_TO_MPS = 0.44704
FT_TO_M = 0.3048
s_h_c, s_h_s, s_v_c, s_v_s = snmac_thresholds(3.0, 0.5, 0.254, 0.076)

def run_brute_force_joint(n, footprint_radius_m, W_A_m, r_min_m, r_max_m, v_B_mps,
                            band_low, band_high, S_h_m, S_v_m, hover_duration_s,
                            cruise_alt_options_ft, rng, M_fine=20000):
    """Brute-force ground truth: dense grid over the FULL window, checking
    the joint horizontal-AND-vertical condition directly at every point,
    no closed-form shortcuts at all."""
    sigma_h = W_A_m / 1.96
    U = rng.uniform(0.0, 1.0, size=n)
    r = np.sqrt(r_min_m**2 + U * (r_max_m**2 - r_min_m**2))
    theta0 = rng.uniform(0.0, 2*np.pi, size=n)
    omega = v_B_mps / r
    A_r = footprint_radius_m * np.sqrt(rng.uniform(0.0, 1.0, size=n))
    A_theta = rng.uniform(0.0, 2*np.pi, size=n)
    dx_A = A_r*np.cos(A_theta) + rng.normal(0.0, sigma_h, size=n)
    dy_A = A_r*np.sin(A_theta) + rng.normal(0.0, sigma_h, size=n)
    bz = rng.uniform(band_low, band_high, size=n)
    cruise_alt_ft = rng.choice(cruise_alt_options_ft, size=n)
    cruise_alt_m = cruise_alt_ft * FT_TO_M
    half_T = hover_duration_s/2.0
    rate = cruise_alt_m/half_T

    t_grid = np.linspace(0, hover_duration_s, M_fine)[None,:]
    # Z(t): descent then ascent, vectorized
    is_descent = t_grid <= half_T
    Z = np.where(is_descent,
                 cruise_alt_m[:,None] - rate[:,None]*t_grid,
                 rate[:,None]*(t_grid - half_T))
    vert_ok = np.abs(Z - bz[:,None]) <= S_v_m

    phi = theta0[:,None] + omega[:,None]*t_grid
    bx = r[:,None]*np.cos(phi)
    by = r[:,None]*np.sin(phi)
    dist = np.sqrt((dx_A[:,None]-bx)**2 + (dy_A[:,None]-by)**2)
    horiz_ok = dist <= S_h_m

    joint_ok = np.any(vert_ok & horiz_ok, axis=1)
    # Also return the minimum horizontal distance AMONG vertically-eligible points
    # (for magnitude comparison), using a large sentinel where none eligible
    dist_masked = np.where(vert_ok, dist, np.inf)
    min_dist_among_eligible = np.min(dist_masked, axis=1)
    return joint_ok, min_dist_among_eligible, dict(r=r, theta0=theta0, omega=omega,
        dx_A=dx_A, dy_A=dy_A, bz=bz, cruise_alt_m=cruise_alt_m, rate=rate, half_T=half_T)


import sys
sys.path.insert(0, '.')
from hover_vs_orbit_v3 import run_hover_vs_orbit

base = dict(
    footprint_radius_m=300.0, W_A_m=5.0, r_min_m=50.0, r_max_m=300.0,
    v_B_mps=45*MPH_TO_MPS, B_total_low_m=100*FT_TO_M, B_total_high_m=300*FT_TO_M,
    hover_duration_s=45.0,
)

rng = np.random.default_rng(777)
n_val = 3000

for S_h_test, S_v_test, label in [(s_h_s, s_v_s, 'sNMAC'), (s_h_c, s_v_c, 'contact')]:
    joint_ok, min_dist_eligible, params = run_brute_force_joint(
        n_val, base['footprint_radius_m'], base['W_A_m'], base['r_min_m'], base['r_max_m'],
        base['v_B_mps'], base['B_total_low_m'], base['B_total_high_m'], S_h_test, S_v_test,
        base['hover_duration_s'], (200.0,250.0,300.0,350.0,400.0), rng, M_fine=20000)

    # Now replicate the closed-form + restricted-two-stage method using the
    # EXACT SAME random draws (params), to do an apples-to-apples comparison
    r_ = params['r']; theta0_ = params['theta0']; omega_ = params['omega']
    dx_A_ = params['dx_A']; dy_A_ = params['dy_A']; bz_ = params['bz']
    cruise_alt_m_ = params['cruise_alt_m']; rate_ = params['rate']; half_T = params['half_T']
    n = n_val

    t_desc_lo = np.clip((cruise_alt_m_ - bz_ - S_v_test)/rate_, 0.0, half_T)
    t_desc_hi = np.clip((cruise_alt_m_ - bz_ + S_v_test)/rate_, 0.0, half_T)
    t_asc_lo = np.clip(half_T + (bz_ - S_v_test)/rate_, half_T, base['hover_duration_s'])
    t_asc_hi = np.clip(half_T + (bz_ + S_v_test)/rate_, half_T, base['hover_duration_s'])

    def horiz_min_in_window(t_lo, t_hi):
        valid = t_hi > t_lo
        span = np.where(valid, t_hi-t_lo, 1.0)
        M1=200
        frac1 = np.linspace(0,1,M1)[None,:]
        t_grid1 = t_lo[:,None] + frac1*span[:,None]
        phi1 = theta0_[:,None] + omega_[:,None]*t_grid1
        bx1 = r_[:,None]*np.cos(phi1); by1 = r_[:,None]*np.sin(phi1)
        dist1 = np.sqrt((dx_A_[:,None]-bx1)**2+(dy_A_[:,None]-by1)**2)
        idx_min = np.argmin(dist1,axis=1)
        coarse_min = dist1[np.arange(n), idx_min]
        spacing = span/(M1-1)
        t_at_min = t_grid1[np.arange(n), idx_min]
        M2=100
        local_hw = 2*spacing
        frac2 = np.linspace(-1,1,M2)[None,:]
        t_grid2 = np.clip(t_at_min[:,None]+frac2*local_hw[:,None], t_lo[:,None], t_hi[:,None])
        phi2 = theta0_[:,None]+omega_[:,None]*t_grid2
        bx2 = r_[:,None]*np.cos(phi2); by2 = r_[:,None]*np.sin(phi2)
        dist2 = np.sqrt((dx_A_[:,None]-bx2)**2+(dy_A_[:,None]-by2)**2)
        fine_min = np.min(dist2,axis=1)
        true_min = np.minimum(coarse_min, fine_min)
        return np.where(valid, true_min, np.inf)

    min_desc = horiz_min_in_window(t_desc_lo, t_desc_hi)
    min_asc = horiz_min_in_window(t_asc_lo, t_asc_hi)
    method_min = np.minimum(min_desc, min_asc)
    method_ok = method_min <= S_h_test

    agree = joint_ok == method_ok
    n_disagree = np.sum(~agree)
    print(f'[{label}] agreement: {agree.mean()*100:.4f}%  ({n_disagree}/{n_val} disagree)')
    if n_disagree > 0:
        idx = np.where(~agree)[0][:5]
        for i in idx:
            print(f'   trial {i}: brute_min_eligible={min_dist_eligible[i]:.3f} method_min={method_min[i]:.3f} S_h={S_h_test:.3f}')
