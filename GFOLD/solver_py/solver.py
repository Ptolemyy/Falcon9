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
            if N == 10:
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
    
    def generate_code(self,code_dir="p3_cpg_solver"):
        print("Generating code for P3 and P4...")
        cpg.generate_code(self.problem, code_dir=code_dir, solver=cp.ECOS,wrapper=False)
        print("Code generation complete.")

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

    solver_p3 = LCvxSolver(problem_type='p3')

    parser = argparse.ArgumentParser()
    parser.add_argument("--gen", action="store_true", default=True)
    parser.add_argument("--plot", action="store_true", default=False)
    _args = parser.parse_args()
    
    if _args.gen:
        out_root = "cpg_solver"
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
    elif _args.plot:
        print("Plotting results...")
        solver_p4 = LCvxSolver(problem_type='p4')
        m_p3, r_p3, v_p3, u_p3, z_p3, s_p3 = solver_p3.solve_direct()
        m_p4, r_p4, v_p4, u_p4, z_p4, s_p4 = solver_p4.solve_direct()
        plot_run3D(params.dt*params.N, r_p4, v_p4, z_p4, u_p4, m_p4, s_p4, params, bnd)
        
