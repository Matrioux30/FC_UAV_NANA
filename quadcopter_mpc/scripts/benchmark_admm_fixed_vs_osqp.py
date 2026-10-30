"""Accuracy of fixed-iteration ADMM vs OSQP ground truth, scaled (deployed) vs unscaled formulation."""
import numpy as np
import mpc_model as m


def reference(fn):
    return np.array([fn(i * m.TS) for i in range(m.NP)])


def main():
    prob = m.build_scaled_problem()
    H, Mx, Mr, u_max = prob['H'], prob['Mx'], prob['Mr'], prob['u_max']
    cases = [
        ("Nominal (Hover)", np.array([0.05, -0.02, 0.95, 0.01, -0.01, 0, 0.02, -0.01, 0, 0, 0, 0]),
         reference(lambda t: [0, 0, 1.0, 0])),
        ("Constraint Aktif (Step 1.5m)", np.zeros(12), reference(lambda t: [0, 0, 1.5, 0])),
        ("Dinamis (Trajectory Corner)", np.array([0.5, 0.0, 1.2, 0.2, 0.2, -0.1, 0.1, -0.08, 0.05, 0.1, -0.1, 0.05]),
         reference(lambda t: [0.5 * np.cos(0.5 * t), 0.5 * np.sin(0.5 * t), 1.0 + 0.2 * np.sin(t), 0])),
    ]
    print("=" * 78)
    print(" ADMM FIXED ITERATION VS OSQP  (max |du0| / u_max)")
    print("=" * 78)
    for name, x0, r in cases:
        g = Mx @ x0 - Mr @ m.ref_horizon_full(r)
        u_true = m.solve_osqp(H, g, -u_max, u_max)
        g_s = prob['Mxs'] @ x0 - prob['Mrs'] @ r.ravel()
        print(f"\nKasus: {name}\n  OSQP u0* = {u_true[:4]}")
        for n in (15, 30, 40, 60, m.N_ITER, 120):
            u_s = prob['E'] * m.admm_scaled(prob, g_s, n)
            u_p, _ = m.custom_admm_qp(H, g, -u_max, u_max, rho=1.0, max_iter=n, tol=0.0)
            e = lambda u: np.abs((u - u_true)[:4] / u_max[:4]).max()
            tag = '  <- deployed' if n == m.N_ITER else ''
            print(f"  N={n:3d}  scaled: {e(u_s):.2e}   unscaled(rho=1): {e(u_p):.2e}{tag}")

    closed_loop_sweep(prob)


def closed_loop_sweep(prob):
    """The three cases above are easy; evaluate every QP from the golden-model closed loop."""
    import os
    import pandas as pd
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'test_vectors', 'test_vectors.csv')
    if not os.path.exists(path):
        return
    tv = pd.read_csv(path)
    H, Mx, Mr, u_max = prob['H'], prob['Mx'], prob['Mr'], prob['u_max']
    xs = tv[[f'x{i}' for i in range(m.NX)]].values
    rs = tv[[f'r{p}_{c}' for p in range(m.NP) for c in 'xyzw']].values
    u_ex = tv[[f'u_exact{c}' for c in range(m.NU)]].values.copy()
    u_ex[:, 0] -= m.MASS * m.GRAVITY
    print(f"\nSemua {len(tv)} QP closed-loop (acuan: solusi eksak di test_vectors.csv), N = {m.N_ITER}:")
    for label, solve in [
        ('scaled (deployed)', lambda x, r: prob['E'] * m.admm_scaled(prob, prob['Mxs'] @ x - prob['Mrs'] @ r)),
        ('unscaled rho=1   ', lambda x, r: m.custom_admm_qp(H, Mx @ x - Mr @ m.ref_horizon_full(r.reshape(m.NP, -1)),
                                                             -u_max, u_max, 1.0, m.N_ITER, 0.0)[0]),
    ]:
        err = np.array([np.abs((solve(x, r)[:m.NU] - ue) / u_max[:m.NU]).max() for x, r, ue in zip(xs, rs, u_ex)])
        print(f"  {label}: worst {err.max():.2e}, siklus > 1e-3: {(err > 1e-3).sum()}")


if __name__ == '__main__':
    main()
