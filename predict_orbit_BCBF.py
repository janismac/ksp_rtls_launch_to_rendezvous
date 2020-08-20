import numpy as np
import scipy.integrate

def predict_orbit_BCBF(vessel, frame):
    r0 = vessel.position(frame)
    v0 = vessel.velocity(frame)

    omega = vessel.orbit.body.rotational_speed
    mu = vessel.orbit.body.gravitational_parameter

    y0 = list(r0)+list(v0)

    t_grid = np.arange(0.0,60.0*20,5.0)
    result = scipy.integrate.odeint(lambda y,t: vacuum_coast_BCBF_ODE(y,omega,mu), y0, t_grid, atol=1e-5, rtol=1e-5)

    # return vessel position for the next 10 to 20 minutes
    return result[t_grid >= 600.0]

def vacuum_coast_BCBF_ODE(y,omega,mu):
    r = y[0:3]
    v = y[3:6]
    w = np.array([0.0,-omega,0.0])
    a_gravity = -r * mu * np.dot(r,r)**(-1.5)
    a_coriolis = -2 * np.cross(w,v)
    a_centrifugal = -np.cross(w,np.cross(w,r))
    a = a_gravity + a_coriolis + a_centrifugal
    return np.hstack((v,a))
