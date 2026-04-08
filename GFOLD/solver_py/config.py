from dataclasses import dataclass
import numpy as np
from scipy import signal
@dataclass
class Boundary:
    r0: np.ndarray
    v0: np.ndarray
    m0: float
    rT: np.ndarray
    vT: np.ndarray
    throt1: float       # thrust lower bound ()
    throt2: float       # thrust upper bound ()
    T_max: float
    V_max: float
    y_gs: float = 30.0 #glide_slope angle (deg)
    theta_deg: float = 120.0  # pointing cone half-angle (deg) relative to +z
    rp3: float = 0.0  # radius for p3 target

@dataclass
class ParamsLCVX:
    N: int
    dt: float
    g: np.ndarray
    a: float

params = ParamsLCVX(
        N=100,
        dt=0.5729, #tf/N
        g=np.array([-3.71, 0.0, 0.0]),
        a = 0.0005,
    )
bnd = Boundary(
    r0=np.array([2400.0, 450.0, -330.0]),
    v0=np.array([-10.0, -40.0, 10.0]),
    m0=2000.0,
    throt1= 0.2, #throt
    throt2= 0.8,
    T_max= 24000.0,
    rT=np.array([0.0, 0.0, 0.0]),
    vT=np.array([0.0, 0.0, 0.0]),
    V_max = 90.0,
    theta_deg=45.0,
    y_gs=30.0,
    rp3 = np.array([0.0,0.0,0.0])
)

    