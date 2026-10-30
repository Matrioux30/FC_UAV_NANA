"""Sanity check: deployed scaled ADMM vs OSQP on the same QP, and the effect of the scaling itself."""
import numpy as np
import mpc_model as m


def main():
    prob = m.build_scaled_problem()
    H, Mx, Mr = prob['H'], prob['Mx'], prob['Mr']
    u_max = prob['u_max']
    print(f"cond(H) unscaled = {np.linalg.cond(H):.3e}, scaled = {np.linalg.cond(prob['Hs']):.1f}")

    x0 = np.array([0.1, -0.05, 0.5, 0, 0, 0, 0.05, -0.03, 0, 0, 0, 0])
    r = np.tile([0.0, 0.0, 1.0, 0.0], (m.NP, 1))
    g = Mx @ x0 - Mr @ m.ref_horizon_full(r)
    u_osqp = m.solve_osqp(H, g, -u_max, u_max)

    g_s = prob['Mxs'] @ x0 - prob['Mrs'] @ r.ravel()
    u_scaled = prob['E'] * m.admm_scaled(prob, g_s)
    u_plain, it_plain = m.custom_admm_qp(H, g, -u_max, u_max, rho=1.0, max_iter=m.N_ITER, tol=0.0)

    err = lambda u: np.abs((u - u_osqp)[:m.NU] / u_max[:m.NU]).max()
    print(f"OSQP u0          : {u_osqp[:m.NU]}")
    print(f"Scaled ADMM u0   : {u_scaled[:m.NU]}  (N={m.N_ITER}, max |du0|/u_max = {err(u_scaled):.2e})")
    print(f"Unscaled ADMM u0 : {u_plain[:m.NU]}  (N={m.N_ITER}, rho=1, max |du0|/u_max = {err(u_plain):.2e})")


if __name__ == '__main__':
    main()
