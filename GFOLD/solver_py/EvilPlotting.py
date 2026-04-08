import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import numpy.linalg as npl
import math
#from solver_lcvx_log import ParamsLCVX, Boundary

def plot_run2D(t, r, v, u, m, p):
    '''
    print('r =',r)
    print('v =',v)
    print('u =',u)
    print('m =',m)
    print('s =',s)
    '''
    r = np.array(r.value)
    v = np.array(v.value)
    u = np.array(u.value)
    T_val = [np.linalg.norm(u[:,i])*m[i] for i in range(len(v.T))]
    vnorm = [np.linalg.norm(vel) for vel in v.T]

    #u_dirs = 90 - np.degrees(np.atan2(u[1,:], u[0,:]))
    #T_vals = np.multiply(u_norms , m)

    traj = plt.figure()

    #plt.subplot(4,1,1)
    plt.plot(r[0,:],r[1,:])
    M =str(np.tan(np.radians(p.slope)))
    nM=str(-float(M))
    bx=str(p.r_d[0])
    by=str(p.r_d[1])
    x=np.array(range(0,int(max(r[0,:]))))
    plt.plot(x,eval(M+'*(x-'+bx+')+'+by))
    x=np.array(range(int(min(r[0,:])),0))
    plt.plot(x,eval(nM+'*(x-'+bx+')+'+by))
    plt.title('Position (m)')

    f = plt.figure()
    ax = f.add_subplot(411)

    plt.plot(t,vnorm)
    by=str(p.V_max)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval(by))
    plt.xlabel(r"$t$", fontsize=16)
    plt.title('Velocity Magnitude (m/s)')

    plt.subplot(4,1,2)
    plt.plot(t,r[1,:])
    plt.xlabel(r"$t$", fontsize=16)
    plt.title('Altitude (m)')

    plt.subplot(4,1,3)
    plt.plot(t,m)
    plt.title('Mass (kg)')

    plt.subplot(4,1,4)
    plt.plot(t,T_val)
    by=str(p.T_max)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval(by))
    plt.title('Thrust (N)')

    plt.tight_layout()
    plt.subplots_adjust(hspace=0)
    plt.show()

def plot_run3D(tf, r, v, z, u, m, s, params, bnd):
    
    print('tf',tf)
    t = np.linspace(0,tf,num=len(m))

    print('t',t.shape)
    print('r',r.shape)
    print('v',v.shape)
    print('u',u.shape)
    print('m',len(m))
    print('z',z.shape)

    if t.shape==() or r.shape==() or v.shape==() or u.shape==():
        print('data actually empty')
        return

    Th= [np.linalg.norm(u[i])*m[i] for i in range(len(v))]
    vnorm = [np.linalg.norm(vel) for vel in v]
    s_mid = s[1:-1]
    T_mid = (s * m)[1:-1]

    print("s_mid min/max:", s_mid.min(), s_mid.max())
    print("T_mid min/max:", T_mid.min(), T_mid.max())
    print("Required fuel ", m[0] - m[-1])

    #u_dirs_1 = [90 - np.degrees(np.atan2(u[0,n], u[1,n])) for n in range(p.N)]
    #u_dirs_2 = [90 - np.degrees(np.atan2(u[0,n], u[2,n])) for n in range(p.N)]

    traj = plt.figure()
    ax = traj.add_subplot(111,projection='3d')
    ax.set_aspect('equal')

    r_= np.linspace(0, max(max(r[:,1]),max(r[:,2])), 7)
    a_= np.linspace(0, 2*np.pi, 20)
    R, P = np.meshgrid(r_, a_)
    X, Y, Z = R*np.cos(P), R*np.sin(P), R*(np.tan(np.deg2rad(bnd.y_gs)))
    #X,Y,Z=R*np.cos(P), R*np.sin(P),((R**2 - 1)**2)

    #ax.plot(x(t),y(t),z(t),label='Flight Path')
    ax.plot(r[:,1],r[:,2],r[:,0],label='Flight Path')
    ax.plot_surface(X, Y, Z, cmap=plt.cm.YlGnBu_r)

    # Tweak the limits and add latex math labels.

    ax.set_xlabel(r'$x{1}$')
    ax.set_ylabel(r'$x{2}$')
    ax.set_zlabel(r'$x{0}$')

    ax.legend()

    f = plt.figure()
    ax = f.add_subplot(511)

    plt.plot(t,vnorm)
    y=str(bnd.V_max)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,eval('0*x+'+y))
    plt.title('Velocity Magnitude (m/s)')

    plt.subplot(6,1,2)
    plt.plot(t,r[:,0])
    plt.title('Altitude (m)')

    plt.subplot(6,1,3)
    plt.plot(t,m)
    plt.title('Mass (kg)')

    plt.subplot(6,1,4)
    plt.plot(t,Th)
    y1=(bnd.T_max*bnd.throt2)
    y2=(bnd.T_max*bnd.throt1)
    x=np.array(range(0,int(max(t))))
    plt.plot(x,0*x+y1,linestyle='--',color='red')
    plt.plot(x,0*x+y2,linestyle='--',color='red')
    plt.title('Thrust (N)')


    plt.subplot(6,1,5)
    u_angle = [np.degrees(math.acos(min(1,ui[0] / npl.norm(ui)))) for ui in u]
    plt.plot(x,0*x+(bnd.theta_deg),linestyle='--',color='red')
    plt.plot(t,u_angle)
    plt.title('Thrust angle')

    plt.tight_layout()
    plt.subplots_adjust(hspace=1)
    plt.show()
