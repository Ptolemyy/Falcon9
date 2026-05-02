# solver_lcvx_log.py
"""
LCvx solver faithful to Acikmese & Ploen (Soft Landing LCvx) with integrated
lexicographic (P4 -> P3) runner.

Core model (discrete, fixed dt):
    r_{k+1} = r_k + dt * v_k
    v_{k+1} = v_k + dt * (g + u_k)
    z_{k+1} = z_k - alpha * sigma_k * dt         (z = ln m, alpha = 1/(Isp*g0))

Lossless convexification constraints:
    ||u_k||_2 <= sigma_k
    n^T u_k >= cos(theta) * sigma_k              (pointing cone about +z)

Thrust magnitude bounds (paper's exponential form) approximated by Eq. (37):
    rho1 * exp(-z0_k) * [1 - (z_k - z0_k) + 0.5 (z_k - z0_k)^2] <= sigma_k
    sigma_k <= rho2 * exp(-z0_k) * [1 - (z_k - z0_k)]

Provides:
- lcvx_solve(...)   : Solve P3 (fuel-min), P4 (error-min), or Weighted
- run_lexicographic: Stage-1 P4 -> Stage-2 P3 with hard caps based on P4
- plot_lexi(...)   : Make plots for the lexicographic planned trajectories only
"""

import numpy as np
import cvxpy as cp
import copy
from config import *
import os
os.environ["CMAKE_GENERATOR"] = "Visual Studio 17 2022"
os.environ["CMAKE_GENERATOR_PLATFORM"] = "x64"

from cvxpygen import cpg
from EvilPlotting import plot_run3D

def generated_solver_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "cpg_solver"))

class LCvxSolver:
    def __init__(self, params=params, bnd=bnd, problem_type='p4', N_override=None):
        self.params = copy.copy(params)
        if N_override is not None:
            self.params.N  = N_override
        self.bnd = bnd
        self.lcvx_solve(problem_type=problem_type)
        
    @staticmethod
    def _calculate_parameter(expression, *args, **kwargs):
        """Helper function to create parameters from expressions with values"""
        return cp.Parameter(*args, value=expression.value, **kwargs)

    @staticmethod
    def e(i):
        return signal.unit_impulse(3, i)

    def lcvx_solve(self, problem_type):
        """params"""
        params = self.params
        bnd = self.bnd

        log_m0_ = np.log(bnd.m0)
        rho1_ = bnd.throt1 * bnd.T_max
        rho2_ = bnd.throt2 * bnd.T_max
        cos_theta_deg_ = np.cos(np.deg2rad(bnd.theta_deg))
        sin_y_gs_ = np.sin(np.deg2rad(bnd.y_gs))
        cot_y_gs_ = 1.0 / np.tan(np.deg2rad(bnd.y_gs))

        N = params.N
        dt = cp.Parameter(name='dt', value=params.dt, nonneg=True)
        g = cp.Parameter(3, name='g', value=params.g)
        g_dt = self._calculate_parameter(g * dt, 3, name='g_dt')
        a = cp.Parameter(name='a', value=params.a, nonneg=True)
        a_dt = self._calculate_parameter(a * dt, name='a_dt', nonneg=True)
        #g_dt_sq = self._calculate_parameter(g * dt * dt, 3, name="g_dt_sq")
        #dt_squared = self._calculate_parameter(dt**2, name="dt_squared")
        """boundary conditions"""
        r0 = cp.Parameter(3, name='r0', value=bnd.r0)
        v0 = cp.Parameter(3, name='v0', value=bnd.v0)
        log_m0 = cp.Parameter(name='log_m0', value=log_m0_, nonneg=True)
        sin_y_gs = cp.Parameter(name='sin_y_gs', value=sin_y_gs_, nonneg=True)
        cot_y_gs = cp.Parameter(name='cot_y_gs', value=cot_y_gs_, nonneg=True)
        cos_theta_deg = cp.Parameter(name='cos_theta_deg', value=cos_theta_deg_)
        rT = bnd.rT
        vT = bnd.vT
        V_max = bnd.V_max
        rp3 = cp.Parameter(3, name='rp3', value=bnd.rp3)

        z0 = cp.Parameter(N, name='z0')
        mu_2 = cp.Parameter(N, name='mu_2', nonneg=True)
        mu_1 = cp.Parameter(N, name='mu_1', nonneg=True)

        c_z0_ = []
        c_mu2_ = []
        c_mu1_ = []

        for k in range(0, N):
            z00_term = bnd.m0 - a.value * rho2_ * (k) * dt.value  # see ref [2], eq 34,35,36
            z00 = np.log(z00_term)
            mu_2_ = 1 / (rho2_ * np.exp(-z00))
            mu_1_ = 1 / (rho1_ * np.exp(-z00))
            c_z0_.append(z00)
            c_mu2_.append(mu_2_)
            c_mu1_.append(mu_1_)

        z0.value = c_z0_
        mu_2.value = c_mu2_
        mu_1.value = c_mu1_

        if problem_type == 'p3':
            program = 3
        elif problem_type == 'p4':
            program = 4

        # Decision variables
        r = cp.Variable((N, 3), "r")
        v = cp.Variable((N, 3), "v")
        z = cp.Variable(N, "z")            # ln m
        u = cp.Variable((N, 3), "u")         # specific thrust: Tc/m
        s = cp.Variable(N, "s")          # ||Tc||

        self.variables = {
            "r": r,
            "v": v,
            "u": u,
            "s": s,
            "z": z
        }
        # Constraints
        constraints = []
        # Initial conditions
        constraints += [r[0, :] == r0]
        constraints += [v[0, :] == v0]
        constraints += [v[-1, :] == vT]

        constraints += [z[0] == log_m0]

        if program == 3:
            constraints += [r[-1, 0] == 0]
        elif program == 4:
            if N != 100:
                # For N=10, relax terminal position to y/z-only tolerance.
                constraints += [cp.norm(r[-1, 1:3] - rp3[1:3]) <= 2.0]
                constraints += [r[-1, 0] == rp3[0]]
            else:
                constraints += [r[-1, :] == rp3]
            #con += [norm(E*(x[0:3,N-1]-rf))<=norm(rp3-rf)] # CONVEX <= CONVEX (?)

        for k in range(0, N):
            # Dynamics --> v = A(w)*x + B*(g + u)
            if k != N - 1:
                acc = (u[k, :] + u[k+1, :]) / 2
                constraints += [
                    r[k+1, :] == r[k, :] + (v[k, :] + v[k+1, :]) * dt / 2, #+ (acc * dt_squared + g_dt_sq) * (1 / 2),
                    v[k+1, :] == v[k, :] + acc * dt + g_dt,
                ]
                constraints += [z[k+1] == z[k] - (a_dt / 2) * (s[k] + s[k+1])]  # mass decreases
            # For N=10 in P4, anchor glide-slope cone at the optimized terminal point r[-1,:].
            # SOC form keeps convexity: ||(y,z)-(y_f,z_f)|| <= (x-x_f) * cot(gamma).
            if program == 4 and N == 10:
                constraints += [r[k, 0] >= r[-1, 0]]
                constraints += [cp.norm(r[k, 1:3] - r[-1, 1:3]) <= (r[k, 0] - r[-1, 0]) * cot_y_gs]
            else:
                #constraints += [cp.norm(E*(r[k, :]- rp3)) - c.T*(r[k, :]- rp3) <= 0 ] # glideslope, full generality # (5)
                #constraints += [cp.norm((r[k,:]-rT)[0:2]) - c.T[0]*(r[k,0]-rT[0]) <= 0] # glideslope, specific, but faster
                constraints += [r[k, 0] >= cp.norm(r[k, :]) * sin_y_gs]
            constraints += [cp.norm(v[k, :]) <= V_max]  # velocity

            constraints += [cp.norm(u[k, :]) <= s[k]]  # limit thrust magnitude & also therefore, mass
            constraints += [u[k, 0] >= cos_theta_deg * s[k]]

            constraints += [(1 - (z[k] - z0[k]) + 0.5 * (z[k] - z0[k])**2) <= s[k] * mu_1[k]]
            constraints += [s[k] * mu_2[k] <= (1 - (z[k] - z0[k]))]

        if program == 3:
            print('-----------------------------')
            objective = cp.Minimize(cp.norm(r[-1, :] - rT))
            self.problem = cp.Problem(objective, constraints=constraints)
            print('-----------------------------')
        elif program == 4:
            print('-----------------------------')
            objective = cp.Minimize(cp.sum(s))
            self.problem = cp.Problem(objective, constraints=constraints)
            print('-----------------------------')

    def solve_direct(self):
        self.problem.solve(solver=cp.ECOS, verbose=True)
        print("Problem status: ", self.problem.status)
        m_val = np.exp(self.variables["z"].value).tolist()
        r_val = self.variables["r"].value
        v_val = self.variables["v"].value
        u_val = self.variables["u"].value
        z_val = self.variables["z"].value
        s_val = self.variables["s"].value
        return m_val, r_val, v_val, u_val, z_val, s_val
    
    def generate_code(self,code_dir=None):
        if code_dir is None:
            code_dir = os.path.join(generated_solver_root(), "p3")
        print("Generating code for P3 and P4...")
        cpg.generate_code(self.problem, code_dir=code_dir, solver=cp.ECOS,wrapper=False)
        print("Code generation complete.")

class LCvxLanding2DFreeXSolver:
    """
    2D GFOLD landing-burn problem for the planner recovery chain.

    State:
        r = [h, s]      h: altitude, s: downrange
        v = [vh, vs]

    Terminal constraints:
        h_N = 0
        vh_N = 0
        vs_N = 0
        s_N is free

    This matches the "planning-version v1" contract in
    REENTRY_LANDING_GUIDANCE_PLAN.md: free touchdown downrange, but a true
    zero-altitude / zero-velocity landing.
    """

    def __init__(
        self,
        params=params,
        bnd=bnd,
        N_override=50,
        terminal_downrange_regularization=1.0e-8,
    ):
        self.params = copy.copy(params)
        if N_override is not None:
            self.params.N = N_override
        self.bnd = bnd
        self.terminal_downrange_regularization = terminal_downrange_regularization
        self.lcvx_solve()

    @staticmethod
    def _parameter(expression, *args, **kwargs):
        return cp.Parameter(*args, value=expression.value, **kwargs)

    def _mass_linearization_terms(self, N, rho1, rho2, dt, alpha):
        z0_values = []
        mu2_values = []
        mu1_values = []
        for k in range(N):
            z00_term = self.bnd.m0 - alpha * rho2 * k * dt
            if z00_term <= 1.0:
                raise ValueError("2D GFOLD mass linearization became non-positive; reduce dt/N or thrust.")
            z00 = np.log(z00_term)
            z0_values.append(z00)
            mu2_values.append(1.0 / (rho2 * np.exp(-z00)))
            mu1_values.append(1.0 / (rho1 * np.exp(-z00)))
        return z0_values, mu1_values, mu2_values

    def lcvx_solve(self):
        params = self.params
        bnd = self.bnd

        N = params.N
        dt = cp.Parameter(name="dt2d", value=params.dt, nonneg=True)
        g2 = cp.Parameter(2, name="g2d", value=np.array([params.g[0], params.g[1]], dtype=float))
        g_dt = self._parameter(g2 * dt, 2, name="g2d_dt")
        alpha = cp.Parameter(name="alpha2d", value=params.a, nonneg=True)
        alpha_dt = self._parameter(alpha * dt, name="alpha2d_dt", nonneg=True)

        r0 = cp.Parameter(2, name="r0_2d", value=np.array([bnd.r0[0], bnd.r0[1]], dtype=float))
        v0 = cp.Parameter(2, name="v0_2d", value=np.array([bnd.v0[0], bnd.v0[1]], dtype=float))
        log_m0 = cp.Parameter(name="log_m0_2d", value=np.log(bnd.m0), nonneg=True)
        cos_theta = cp.Parameter(name="cos_theta_2d", value=np.cos(np.deg2rad(bnd.theta_deg)))
        cot_glide = cp.Parameter(name="cot_glide_2d", value=1.0 / np.tan(np.deg2rad(bnd.y_gs)), nonneg=True)

        rho1 = bnd.throt1 * bnd.T_max
        rho2 = bnd.throt2 * bnd.T_max
        z0_values, mu1_values, mu2_values = self._mass_linearization_terms(N, rho1, rho2, params.dt, params.a)
        z0 = cp.Parameter(N, name="z0_2d", value=z0_values)
        mu1 = cp.Parameter(N, name="mu1_2d", value=mu1_values, nonneg=True)
        mu2 = cp.Parameter(N, name="mu2_2d", value=mu2_values, nonneg=True)

        r = cp.Variable((N, 2), "r2d")
        v = cp.Variable((N, 2), "v2d")
        u = cp.Variable((N, 2), "u2d")
        sigma = cp.Variable(N, "sigma2d", nonneg=True)
        z = cp.Variable(N, "z2d")

        constraints = [
            r[0, :] == r0,
            v[0, :] == v0,
            z[0] == log_m0,
            r[-1, 0] == 0.0,
            v[-1, :] == np.zeros(2),
        ]

        for k in range(N):
            if k != N - 1:
                acc = (u[k, :] + u[k + 1, :]) / 2.0
                constraints += [
                    r[k + 1, :] == r[k, :] + (v[k, :] + v[k + 1, :]) * dt / 2.0,
                    v[k + 1, :] == v[k, :] + acc * dt + g_dt,
                    z[k + 1] == z[k] - (alpha_dt / 2.0) * (sigma[k] + sigma[k + 1]),
                ]

            # Free touchdown downrange: glide slope is anchored to the optimized
            # terminal s_N instead of a prescribed landing point.
            constraints += [
                r[k, 0] >= 0.0,
                cp.abs(r[k, 1] - r[-1, 1]) <= (r[k, 0] - r[-1, 0]) * cot_glide,
                cp.norm(v[k, :]) <= bnd.V_max,
                cp.norm(u[k, :]) <= sigma[k],
                u[k, 0] >= cos_theta * sigma[k],
                (1.0 - (z[k] - z0[k]) + 0.5 * (z[k] - z0[k]) ** 2) <= sigma[k] * mu1[k],
                sigma[k] * mu2[k] <= (1.0 - (z[k] - z0[k])),
            ]

        objective = cp.Minimize(
            cp.sum(sigma) * dt +
            self.terminal_downrange_regularization * cp.square(r[-1, 1])
        )
        self.problem = cp.Problem(objective, constraints)
        self.variables = {
            "r": r,
            "v": v,
            "u": u,
            "sigma": sigma,
            "z": z,
        }

    def solve_direct(self, solver=cp.ECOS, verbose=False):
        self.problem.solve(solver=solver, verbose=verbose)
        status = self.problem.status
        feasible = status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
        out = {
            "feasible": feasible,
            "status": status,
            "objective": self.problem.value,
        }
        if not feasible:
            return out

        r_val = self.variables["r"].value
        v_val = self.variables["v"].value
        u_val = self.variables["u"].value
        sigma_val = self.variables["sigma"].value
        z_val = self.variables["z"].value
        out.update({
            "r": r_val,
            "v": v_val,
            "u": u_val,
            "sigma": sigma_val,
            "z": z_val,
            "mass": np.exp(z_val),
            "touchdown_s": float(r_val[-1, 1]),
            "terminal_mass": float(np.exp(z_val[-1])),
            "terminal_altitude": float(r_val[-1, 0]),
            "terminal_speed": float(np.linalg.norm(v_val[-1, :])),
            "max_glide_residual": self.max_glide_residual(r_val),
        })
        return out

    def max_glide_residual(self, r_val):
        cot_glide = 1.0 / np.tan(np.deg2rad(self.bnd.y_gs))
        terminal_s = r_val[-1, 1]
        residuals = np.abs(r_val[:, 1] - terminal_s) - r_val[:, 0] * cot_glide
        return float(np.max(residuals))

    def generate_code(self, code_dir=None):
        if code_dir is None:
            code_dir = os.path.join(generated_solver_root(), "p4_n50_2d_free_x")
        cpg.generate_code(self.problem, code_dir=code_dir, solver=cp.ECOS, wrapper=False)
        print(f"2D free-downrange GFOLD code generation complete: {code_dir}")

def verify_2d_free_x(verbose=False):
    direct_solver = cp.CLARABEL if "CLARABEL" in cp.installed_solvers() else cp.ECOS
    candidates = []
    print("2D free-x GFOLD verification time search")
    for tf in (40.0, 45.0, 50.0, 55.0, 60.0, 70.0, 80.0, 90.0):
        params_tf = copy.copy(params)
        params_tf.N = 50
        params_tf.dt = tf / (params_tf.N - 1)
        solver = LCvxLanding2DFreeXSolver(params=params_tf, N_override=None)
        result = solver.solve_direct(solver=direct_solver, verbose=verbose)
        print(f"  tf={tf:5.1f} s status={result['status']}")
        if result["feasible"]:
            result["tf"] = tf
            candidates.append(result)

    if not candidates:
        print("2D free-x GFOLD verification failed: no feasible terminal time in search window.")
        return 1

    result = min(candidates, key=lambda item: item["objective"])

    print("2D free-x GFOLD verification")
    print(f"  direct solver     : {direct_solver}")
    print(f"  selected tf       : {result['tf']:.3f} s")
    print(f"  status           : {result['status']}")
    print(f"  objective        : {result['objective']:.6f}")
    print(f"  touchdown_s      : {result['touchdown_s']:.3f} m")
    print(f"  terminal_altitude: {result['terminal_altitude']:.6e} m")
    print(f"  terminal_speed   : {result['terminal_speed']:.6e} m/s")
    print(f"  terminal_mass    : {result['terminal_mass']:.3f} kg")
    print(f"  max_glide_resid  : {result['max_glide_residual']:.6e} m")

    ok = (
        abs(result["terminal_altitude"]) <= 1.0e-4 and
        result["terminal_speed"] <= 1.0e-4 and
        result["max_glide_residual"] <= 1.0e-4 and
        np.isfinite(result["touchdown_s"]) and
        np.isfinite(result["terminal_mass"])
    )
    if not ok:
        print("2D free-x GFOLD verification failed tolerance checks.")
        return 2
    print("  result           : PASS")
    return 0

def _strip_m_link_guard(code_dir: str):
    """
    After cvxpygen emit, remove the auto-added
      if(NOT MSVC) target_link_libraries(ecos PRIVATE m) endif()
    block in c/solver_code/CMakeLists.txt to keep MSVC builds clean.
    """
    cmake_path = os.path.join(code_dir, "c", "solver_code", "CMakeLists.txt")
    if not os.path.exists(cmake_path):
        return
    with open(cmake_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    out = []
    skip = False
    changed = False
    for ln in lines:
        if not skip and ln.strip().startswith("if(NOT MSVC"):
            skip = True
            changed = True
            continue
        if skip:
            if ln.strip().startswith("endif()"):
                skip = False
                continue
            continue
        out.append(ln)

    if changed:
        with open(cmake_path, "w", encoding="utf-8") as f:
            f.writelines(out)
        print(f"Stripped MSVC guard in {cmake_path}")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--gen", action="store_true", default=False)
    parser.add_argument("--gen-2d-free-x", action="store_true", default=False)
    parser.add_argument("--verify-2d-free-x", action="store_true", default=False)
    parser.add_argument("--plot", action="store_true", default=False)
    parser.add_argument("--verbose", action="store_true", default=False)
    _args = parser.parse_args()

    if not (_args.gen or _args.gen_2d_free_x or _args.verify_2d_free_x or _args.plot):
        # Preserve the old script behavior: running solver.py with no flags
        # generates the original 3D P3/P4 variants.
        _args.gen = True

    if _args.verify_2d_free_x:
        raise SystemExit(verify_2d_free_x(verbose=_args.verbose))
    
    if _args.gen:
        solver_p3 = LCvxSolver(problem_type='p3')
        out_root = generated_solver_root()
        os.makedirs(out_root, exist_ok=True)

        # P3: keep default N from config, do not override.
        p3_dir = os.path.join(out_root, "p3")
        solver_p3.generate_code(code_dir=p3_dir)
        _strip_m_link_guard(p3_dir)

        # P4: generate multiple N variants by overriding N.
        for n in (100, 50, 25, 10):
            p4_dir = os.path.join(out_root, f"p4_n{n}")
            solver_p4_n = LCvxSolver(problem_type='p4', N_override=n)
            solver_p4_n.generate_code(code_dir=p4_dir)
            _strip_m_link_guard(p4_dir)
    if _args.gen_2d_free_x:
        out_root = generated_solver_root()
        os.makedirs(out_root, exist_ok=True)
        p4_2d_dir = os.path.join(out_root, "p4_n50_2d_free_x")
        solver_2d = LCvxLanding2DFreeXSolver(N_override=50)
        solver_2d.generate_code(code_dir=p4_2d_dir)
        _strip_m_link_guard(p4_2d_dir)
    if _args.plot:
        print("Plotting results...")
        solver_p3 = LCvxSolver(problem_type='p3')
        solver_p4 = LCvxSolver(problem_type='p4')
        m_p3, r_p3, v_p3, u_p3, z_p3, s_p3 = solver_p3.solve_direct()
        m_p4, r_p4, v_p4, u_p4, z_p4, s_p4 = solver_p4.solve_direct()
        plot_run3D(params.dt*params.N, r_p4, v_p4, z_p4, u_p4, m_p4, s_p4, params, bnd)
        
