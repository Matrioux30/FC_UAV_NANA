"""Single source of truth for the MPC formulation used by the C++ golden model and the FPGA accelerator."""
import numpy as np
from scipy.linalg import expm

NX, NU, NP, NC = 12, 4, 20, 5
NU_TOTAL = NU * NC
TS = 0.005

MASS, GRAVITY = 0.068, 9.81
IX, IY, IZ = 6.86e-5, 9.2e-5, 1.366e-4

# State: x, y, z, u, v, w, phi, theta, psi, p, q, r
Q_DIAG = np.array([2000.0, 2000.0, 1500.0, 50.0, 50.0, 50.0, 1.0, 1.0, 5.0, 0.1, 0.1, 0.1])
# Input: dF, tau_x, tau_y, tau_z
R_DIAG = np.array([5e-4, 1e-4, 1e-4, 1e-4])
U_MAX_CH = np.array([0.5, 0.02, 0.02, 0.02])

# Only these reference channels are non-zero (x, y, z, psi), so the accelerator only needs MR for them.
REF_CHANNELS = [0, 1, 2, 8]

# ADMM settings chosen by scripts/select_admm_params.py (RHO is for the unit-box Ruiz-scaled problem).
RHO = 0.2
ALPHA = 1.6
N_ITER = 80
RUIZ_ITERS = 20
# Fixed-point range scaling per number format; neither changes the solution, only a uniform factor on iterates.
# z/u/dual scale with 1/VAR_SCALE, |g| and |Mx|,|Mr| with VAR_SCALE/COST_SCALE, |W| with COST_SCALE/VAR_SCALE^2.
# Q16.16 (range +-32768): enlarge signals to use the integer bits; Q8.24 (range +-128): shrink them.
# Both choices make the online rho a power of two (exact in fixed point).
FORMAT_SCALING = {
    'q16_16': dict(var_scale=1.0 / 32.0, cost_scale=1.0 / 1280.0),
    'q8_24': dict(var_scale=4.0, cost_scale=16.0),
}
FORMAT_MAX_REAL = {'q16_16': 32768.0, 'q8_24': 128.0}

# Design envelope of the measured state / reference; inputs are multiplied by a per-channel power of two so this
# envelope fills 1/INPUT_HEADROOM of the format range (a shift in hardware), folded into MX/MR offline.
STATE_RANGE = np.array([16.0, 16.0, 16.0, 16.0, 16.0, 16.0, np.pi, np.pi, np.pi, 32.0, 32.0, 32.0])
REF_RANGE = np.array([16.0, 16.0, 16.0, np.pi])
INPUT_HEADROOM = 8.0


def input_scales(fmt):
    limit = FORMAT_MAX_REAL[fmt] / INPUT_HEADROOM
    x_scale = 2.0 ** np.floor(np.log2(limit / STATE_RANGE))
    r_scale = 2.0 ** np.floor(np.log2(limit / REF_RANGE))
    return x_scale, np.tile(r_scale, NP)
DEPLOYED_FORMAT = 'q16_16'
VAR_SCALE = FORMAT_SCALING[DEPLOYED_FORMAT]['var_scale']
COST_SCALE = FORMAT_SCALING[DEPLOYED_FORMAT]['cost_scale']


def discretize(Ts=TS):
    Ac = np.zeros((NX, NX))
    Ac[0, 3] = Ac[1, 4] = Ac[2, 5] = 1.0
    Ac[3, 7] = GRAVITY
    Ac[4, 6] = -GRAVITY
    Ac[6, 9] = Ac[7, 10] = Ac[8, 11] = 1.0
    Bc = np.zeros((NX, NU))
    Bc[5, 0] = 1.0 / MASS
    Bc[9, 1] = 1.0 / IX
    Bc[10, 2] = 1.0 / IY
    Bc[11, 3] = 1.0 / IZ
    M = np.zeros((NX + NU, NX + NU))
    M[:NX, :NX] = Ac
    M[:NX, NX:] = Bc
    eM = expm(M * Ts)
    return eM[:NX, :NX], eM[:NX, NX:]


def build_quadcopter_mpc_matrices(Np=NP, Nc=NC, Ts=TS, q_diag=Q_DIAG, r_diag=R_DIAG):
    """Unscaled condensed QP: min 0.5 U'HU + g'U, g = Mx x0 - Mr R."""
    Ad, Bd = discretize(Ts)
    Phi = np.zeros((NX * Np, NX))
    Gamma = np.zeros((NX * Np, NU * Nc))
    A_pow = np.eye(NX)
    for i in range(Np):
        A_pow = A_pow @ Ad
        Phi[i * NX:(i + 1) * NX, :] = A_pow
    for i in range(Np):
        for j in range(min(i + 1, Nc)):
            if j < Nc - 1:
                blk = np.linalg.matrix_power(Ad, i - j) @ Bd
            else:
                # Last move is held to the end of the horizon: x_{i+1} += sum_{p=0}^{i-(Nc-1)} Ad^p Bd
                blk = sum(np.linalg.matrix_power(Ad, p) for p in range(i - Nc + 2)) @ Bd
            Gamma[i * NX:(i + 1) * NX, j * NU:(j + 1) * NU] = blk
    Q_bar = np.kron(np.eye(Np), np.diag(q_diag))
    R_bar = np.kron(np.eye(Nc), np.diag(r_diag))
    H = 2.0 * (Gamma.T @ Q_bar @ Gamma + R_bar)
    H = 0.5 * (H + H.T)
    Mx = 2.0 * (Gamma.T @ Q_bar @ Phi)
    Mr = 2.0 * (Gamma.T @ Q_bar)
    return Ad, Bd, Phi, Gamma, H, Mx, Mr


def ruiz_diag(Hm, iters=RUIZ_ITERS):
    d = np.ones(Hm.shape[0])
    for _ in range(iters):
        Hc = (d[:, None] * Hm) * d[None, :]
        d = d / np.sqrt(np.sqrt(np.abs(Hc).max(axis=1)))
    return d


def build_scaled_problem(var_scale=None, cost_scale=None):
    """Scaled QP in z, with U = E z (E diagonal), bounds z_min <= z <= z_max.

    E = var_scale * D * S: D maps actuator limits to +-1, S is Ruiz equilibration. The cost is divided by cost_scale.
    """
    VAR_SCALE = globals()['VAR_SCALE'] if var_scale is None else var_scale
    COST_SCALE = globals()['COST_SCALE'] if cost_scale is None else cost_scale
    Ad, Bd, _, _, H, Mx, Mr = build_quadcopter_mpc_matrices()
    u_max = np.tile(U_MAX_CH, NC)
    Hd = (u_max[:, None] * H) * u_max[None, :]
    s = ruiz_diag(Hd)
    E = VAR_SCALE * u_max * s
    Hs = (E[:, None] * H) * E[None, :] / COST_SCALE
    Mxs = E[:, None] * Mx / COST_SCALE
    ref_cols = [p * NX + c for p in range(NP) for c in REF_CHANNELS]
    Mrs = E[:, None] * Mr[:, ref_cols] / COST_SCALE
    rho = RHO * VAR_SCALE**2 / COST_SCALE
    W = np.linalg.inv(Hs + rho * np.eye(NU_TOTAL))
    z_max = 1.0 / (VAR_SCALE * s)
    return dict(Ad=Ad, Bd=Bd, H=H, Mx=Mx, Mr=Mr, Hs=Hs, Mxs=Mxs, Mrs=Mrs, W=W, rho=rho,
                E=E, z_min=-z_max, z_max=z_max, u_max=u_max)


def ref_horizon_full(r_small):
    """Expand a (NP, 4) reference [x, y, z, psi] into the full NX*NP vector used by Mr."""
    R = np.zeros(NX * NP)
    for p in range(NP):
        for k, c in enumerate(REF_CHANNELS):
            R[p * NX + c] = r_small[p, k]
    return R


def admm_scaled(prob, g_s, n_iter=N_ITER, alpha=ALPHA, z0=None):
    W, rho, lo, hi = prob['W'], prob['rho'], prob['z_min'], prob['z_max']
    z = np.zeros(NU_TOTAL) if z0 is None else z0.copy()
    y = np.zeros(NU_TOTAL)
    for _ in range(n_iter):
        u = W @ (rho * (z - y) - g_s)
        u_hat = alpha * u + (1.0 - alpha) * z
        z_new = np.clip(u_hat + y, lo, hi)
        y = y + u_hat - z_new
        z = z_new
    return z


def custom_admm_qp(H, g, u_min, u_max, rho=1.0, max_iter=50, tol=1e-4, warm_start=None):
    """Plain (unscaled) ADMM, kept for comparison with the scaled solver."""
    n = len(g)
    W = np.linalg.inv(H + rho * np.eye(n))
    z = np.zeros(n) if warm_start is None else warm_start.copy()
    u_dual = np.zeros(n)
    iter_used = 0
    for it in range(max_iter):
        iter_used = it + 1
        u_val = W @ (rho * (z - u_dual) - g)
        z_new = np.clip(u_val + u_dual, u_min, u_max)
        u_dual += (u_val - z_new)
        pri_res = np.linalg.norm(u_val - z_new)
        dual_res = rho * np.linalg.norm(z_new - z)
        z = z_new
        if pri_res < tol and dual_res < tol:
            break
    return z, iter_used


def solve_osqp(H, g, u_min, u_max, eps=1e-10):
    import osqp
    from scipy import sparse
    prob = osqp.OSQP()
    prob.setup(P=sparse.csc_matrix(H), q=g, A=sparse.eye(len(g), format='csc'), l=u_min, u=u_max,
               verbose=False, eps_abs=eps, eps_rel=eps, polish=True, max_iter=200000)
    return prob.solve().x
