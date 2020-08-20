import glob, os, os.path
import collections
import time
import json
import casadi
from casadi import pi, sin, cos, DM, cross, sumsqr, dot
from yamtof.mocp import MultiPhaseOptimalControlProblem

def optimizer():
    p = get_parameters()
    mocp = MultiPhaseOptimalControlProblem()
    start = lambda a: mocp.start(a)
    end = lambda a: mocp.end(a)
    consecutive_phases = list()

    launch_site = DM([p.launch_site_x, p.launch_site_y, p.launch_site_z])
    landing_site = DM([p.landing_site_x, p.landing_site_y, p.landing_site_z])
    launch_direction_guess = DM([p.launch_direction_guess_x, p.launch_direction_guess_y, p.launch_direction_guess_z])
    up_direction = normalize(launch_site)

    phase_target = create_rocket_stage_phase(
        mocp, 'target', p,
        engine_parameters = None,
        atmosphere_parameters = None,
        init_mass = None,
    )

    phase_ascent = create_rocket_stage_phase(
        mocp, 'ascent', p,
        engine_parameters = p.S1_engine_parameters,
        atmosphere_parameters = p.atmosphere_parameters,
        init_mass = p.total_mass_ascent,
        has_steering_unit_vector_constraint = False,
    )

    phase_S2_coast1 = create_rocket_stage_phase(
        mocp, 'S2_coast1', p,
        engine_parameters = None,
        atmosphere_parameters = None,
        init_mass = p.total_mass_S2,
    )
    consecutive_phases.append((phase_ascent, phase_S2_coast1))
    add_phase_continuity_constraint(mocp, phase_ascent, phase_S2_coast1, include_mass=False)

    phase_S2_burn1 = create_rocket_stage_phase(
        mocp, 'S2_burn1', p,
        engine_parameters = p.S2_engine_parameters,
        atmosphere_parameters = p.atmosphere_parameters,
        init_mass = p.total_mass_S2,
    )
    consecutive_phases.append((phase_S2_coast1, phase_S2_burn1))
    add_phase_continuity_constraint(mocp, phase_S2_coast1, phase_S2_burn1, include_mass=True)

    phase_S2_coast2 = create_rocket_stage_phase(
        mocp, 'S2_coast2', p,
        engine_parameters = None,
        atmosphere_parameters = None,
        init_mass = p.total_mass_S2,
    )
    consecutive_phases.append((phase_S2_burn1, phase_S2_coast2))
    add_phase_continuity_constraint(mocp, phase_S2_burn1, phase_S2_coast2, include_mass=True)

    phase_S2_rendezvous_burn = create_rocket_stage_phase(
        mocp, 'S2_rendezvous_burn', p,
        engine_parameters = p.S2_engine_parameters,
        atmosphere_parameters = None,
        init_mass = p.total_mass_S2,
    )
    consecutive_phases.append((phase_S2_coast2, phase_S2_rendezvous_burn))
    add_phase_continuity_constraint(mocp, phase_S2_coast2, phase_S2_rendezvous_burn, include_mass=True)


    phase_S1_coast1 = create_rocket_stage_phase(
        mocp, 'S1_coast1', p,
        engine_parameters = None,
        atmosphere_parameters = p.atmosphere_parameters,
        init_mass = p.total_mass_ascent - p.total_mass_S2,
    )
    consecutive_phases.append((phase_ascent, phase_S1_coast1))
    add_phase_continuity_constraint(mocp, phase_ascent, phase_S1_coast1, include_mass=False)

    phase_S1_boostback = create_rocket_stage_phase(
        mocp, 'S1_boostback', p,
        engine_parameters = p.S1_engine_parameters,
        atmosphere_parameters = p.atmosphere_parameters,
        init_mass = p.total_mass_ascent - p.total_mass_S2,
    )
    consecutive_phases.append((phase_S1_coast1, phase_S1_boostback))
    add_phase_continuity_constraint(mocp, phase_S1_coast1, phase_S1_boostback, include_mass=True)


    phase_S1_coast2 = create_rocket_stage_phase(
        mocp, 'S1_coast2', p,
        engine_parameters = None,
        atmosphere_parameters = None,
        init_mass = p.total_mass_ascent - p.total_mass_S2,
    )
    consecutive_phases.append((phase_S1_boostback, phase_S1_coast2))
    add_phase_continuity_constraint(mocp, phase_S1_boostback, phase_S1_coast2, include_mass=True)


    phase_S1_landing = create_rocket_stage_phase(
        mocp, 'S1_landing', p,
        engine_parameters = p.S1_engine_parameters,
        atmosphere_parameters = p.atmosphere_parameters,
        init_mass = p.total_mass_ascent - p.total_mass_S2,
    )
    consecutive_phases.append((phase_S1_coast2, phase_S1_landing))
    add_phase_continuity_constraint(mocp, phase_S1_coast2, phase_S1_landing, include_mass=True)


    # Limit landing acceleration
    slack_landing_acceleration = mocp.add_variable('slack_landing_acceleration', init=10.0)
    mocp.add_constraint(slack_landing_acceleration > 0)
    mocp.add_objective(5.0 * slack_landing_acceleration)
    mocp.add_path_constraint(phase_S1_landing.thrust_acceleration < 23.0 / p.scale.acceleration + slack_landing_acceleration)


    # Limit boostback acceleration
    slack_boostback_acceleration = mocp.add_variable('slack_boostback_acceleration', init=10.0)
    mocp.add_constraint(slack_boostback_acceleration > 0)
    mocp.add_objective(1.0 * slack_boostback_acceleration)
    mocp.add_path_constraint(phase_S1_boostback.thrust_acceleration < 25.0 / p.scale.acceleration + slack_boostback_acceleration)

    # Taper throttle at burn end
    mocp.add_constraint(end(phase_ascent.throttle) == 0.05)
    mocp.add_constraint(end(phase_S1_boostback.throttle) == 0.05)
    mocp.add_constraint(end(phase_S2_burn1.throttle) == 0.05)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.throttle) == 0.05)

    # Target initial state constraint
    mocp.add_constraint(start(phase_target.rx) == p.target_position_x)
    mocp.add_constraint(start(phase_target.ry) == p.target_position_y)
    mocp.add_constraint(start(phase_target.rz) == p.target_position_z)
    mocp.add_constraint(start(phase_target.vx) == p.target_velocity_x)
    mocp.add_constraint(start(phase_target.vy) == p.target_velocity_y)
    mocp.add_constraint(start(phase_target.vz) == p.target_velocity_z)

    # Ascent initial state constraint
    mocp.add_constraint(start(phase_ascent.mass) == p.total_mass_ascent)
    mocp.add_constraint(start(phase_ascent.rx) == p.launch_site_x)
    mocp.add_constraint(start(phase_ascent.ry) == p.launch_site_y)
    mocp.add_constraint(start(phase_ascent.rz) == p.launch_site_z)
    mocp.add_constraint(start(phase_ascent.vx) == 0.0)
    mocp.add_constraint(start(phase_ascent.vy) == 0.0)
    mocp.add_constraint(start(phase_ascent.vz) == 0.0)
    mocp.add_constraint(start(phase_ascent.ux) == up_direction[0])
    mocp.add_constraint(start(phase_ascent.uy) == up_direction[1])
    mocp.add_constraint(start(phase_ascent.uz) == up_direction[2])

    # Phase timing constraints
    vessel_switch_time = mocp.add_variable('vessel_switch_time', init=1.0 / p.scale.time)
    mocp.add_constraint(vessel_switch_time > 0.0)
    mocp.add_constraint(vessel_switch_time < 27.0 / p.scale.time)

    mocp.add_constraint(phase_S1_coast1.duration < vessel_switch_time)
    mocp.add_objective(-2.0 * phase_S1_coast1.duration)
    mocp.add_objective(-2.0 * vessel_switch_time)

    mocp.add_constraint(phase_S2_coast2.duration < 300.0 / p.scale.time)

    mocp.add_constraint(phase_S2_coast1.duration > vessel_switch_time + phase_S1_coast1.duration + phase_S1_boostback.duration)
    mocp.add_constraint(phase_S2_coast1.duration + phase_S2_burn1.duration + vessel_switch_time < phase_S1_coast1.duration + phase_S1_boostback.duration + phase_S1_coast2.duration)
    mocp.add_constraint(phase_S2_coast1.duration + phase_S2_burn1.duration + phase_S2_coast2.duration > vessel_switch_time + phase_S1_coast1.duration + phase_S1_boostback.duration + phase_S1_coast2.duration + phase_S1_landing.duration)

    # Staging mass constraints
    mocp.add_constraint(start(phase_S2_coast1.mass) == p.total_mass_S2)
    mocp.add_constraint(p.total_mass_S2 + start(phase_S1_coast1.mass) == end(phase_ascent.mass))


    mocp.add_constraint(end(phase_S1_landing.mass) > p.touchdown_mass)


    # Temporary shaping objectives
    separation_velocity_guess = 0.4 * (0.86 * up_direction + 0.5 * launch_direction_guess)
    orbit_velocity_guess = p.orbit_speed_guess * (0.05 * up_direction + 0.97 * launch_direction_guess)
    boostback_velocity_guess = 0.1 * (0.65 * up_direction - 0.75 * launch_direction_guess)
    entry_position_guess = (1.0+5.0*p.atmosphere_parameters.scale_height) * landing_site

    objective_separation_velocity_guess = mocp.add_objective(100.0 *
        sumsqr(end(phase_ascent.v_vec) - separation_velocity_guess))

    objective_orbit_velocity_guess = mocp.add_objective(100.0 *
        sumsqr(end(phase_S2_rendezvous_burn.v_vec) - orbit_velocity_guess))

    objective_entry_position = mocp.add_objective(10.0 *
        sumsqr(end(phase_S1_coast2.r_vec) - entry_position_guess))

    objective_boostback_velocity = mocp.add_objective(100.0 *
        sumsqr(end(phase_S1_boostback.v_vec) - boostback_velocity_guess))

    #objective_match_target_velocity = mocp.add_objective(100.0 *
    #    sumsqr(end(phase_S2_rendezvous_burn.v_vec) - end(phase_target.v_vec)))

    objective_target_position = mocp.add_objective(100.0 *
        sumsqr(launch_site - end(phase_target.r_vec)))

    objective_landing_velocity = mocp.add_objective(100.0 *
        sumsqr(end(phase_S1_landing.v_vec)))

    objective_landing_position = mocp.add_objective(100.0 *
        sumsqr(end(phase_S1_landing.r_vec) - landing_site))

    objective_ascent_duration              = mocp.add_objective(10.0 * (phase_ascent.duration              - (151.0  / p.scale.time))**2)
    objective_S2_coast1_duration           = mocp.add_objective(10.0 * (phase_S2_coast1.duration           - (122.0  / p.scale.time))**2)
    objective_S2_burn1_duration            = mocp.add_objective(10.0 * (phase_S2_burn1.duration            - (118.0  / p.scale.time))**2)
    objective_S2_coast2_duration           = mocp.add_objective(10.0 * (phase_S2_coast2.duration           - (153.0  / p.scale.time))**2)
    objective_S2_rendezvous_burn_duration  = mocp.add_objective(10.0 * (phase_S2_rendezvous_burn.duration  - (119.0  / p.scale.time))**2)
    objective_S1_coast1_duration           = mocp.add_objective(10.0 * (phase_S1_coast1.duration           - (29.0   / p.scale.time))**2)
    objective_S1_boostback_duration        = mocp.add_objective(10.0 * (phase_S1_boostback.duration        - (62.0   / p.scale.time))**2)
    objective_S1_coast2_duration           = mocp.add_objective(10.0 * (phase_S1_coast2.duration           - (178.0  / p.scale.time))**2)
    objective_S1_landing_duration          = mocp.add_objective(10.0 * (phase_S1_landing.duration          - (93.0   / p.scale.time))**2)

    objective_S2_rendezvous_burn_altitude = mocp.add_objective(10.0 * (end(phase_S2_rendezvous_burn.altitude) - 0.5)**2)


    # Maximize final masses
    objective_mass_S1 = mocp.add_objective(-0.1 * end(phase_S1_landing.mass))
    objective_mass_S2 = mocp.add_objective(-1.0 * end(phase_S2_rendezvous_burn.mass))

    # Solve step 1: trajectory shaping problem
    (solver_output, slack_values) = mocp.solve()


    # Solve step 2: proper final constraints, mesh refinement
    mocp.remove_objective(objective_entry_position)
    mocp.remove_objective(objective_boostback_velocity)
    mocp.remove_objective(objective_orbit_velocity_guess)
    #mocp.remove_objective(objective_match_target_velocity)
    mocp.remove_objective(objective_target_position)
    mocp.remove_objective(objective_landing_velocity)
    mocp.remove_objective(objective_landing_position)
    mocp.remove_objective(objective_S2_rendezvous_burn_altitude)

    mocp.remove_objective(objective_ascent_duration)
    mocp.remove_objective(objective_S2_coast1_duration)
    mocp.remove_objective(objective_S2_burn1_duration)
    mocp.remove_objective(objective_S2_coast2_duration)
    mocp.remove_objective(objective_S2_rendezvous_burn_duration)
    mocp.remove_objective(objective_S1_coast1_duration)
    mocp.remove_objective(objective_S1_boostback_duration)
    mocp.remove_objective(objective_S1_coast2_duration)
    mocp.remove_objective(objective_S1_landing_duration)

    mocp.remove_objective(objective_mass_S1)


    # Increase duration of rendezvous burn for better tracking
    slack_rendezvous_duration = mocp.add_variable('slack_rendezvous_duration', init=10.0)
    mocp.add_constraint(slack_rendezvous_duration > 0)
    mocp.add_objective(1.0 * slack_rendezvous_duration)
    mocp.add_constraint(phase_S2_rendezvous_burn.duration > 70.0/p.scale.time - slack_rendezvous_duration)

    # Landing position soft constraint
    slack_landing_position = mocp.add_variable('slack_landing_position', init=3.0)
    mocp.add_constraint(slack_landing_position > 0)
    mocp.add_objective(100.0 * slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.rx) - p.landing_site_x <  slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.rx) - p.landing_site_x > -slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.ry) - p.landing_site_y <  slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.ry) - p.landing_site_y > -slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.rz) - p.landing_site_z <  slack_landing_position)
    mocp.add_constraint(end(phase_S1_landing.rz) - p.landing_site_z > -slack_landing_position)

    # Landing speed soft constraint
    slack_landing_speed = mocp.add_variable('slack_landing_speed', init=3.0)
    mocp.add_constraint(slack_landing_speed > 0)
    mocp.add_objective(10.0 * slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vx) <  slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vx) > -slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vy) <  slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vy) > -slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vz) <  slack_landing_speed)
    mocp.add_constraint(end(phase_S1_landing.vz) > -slack_landing_speed)

    # rendezvous position soft constraint
    slack_rendezvous_position = mocp.add_variable('slack_rendezvous_position', init=3.0)
    mocp.add_constraint(slack_rendezvous_position > 0)
    mocp.add_objective(100.0 * slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.rx) - end(phase_target.rx) <  slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.rx) - end(phase_target.rx) > -slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.ry) - end(phase_target.ry) <  slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.ry) - end(phase_target.ry) > -slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.rz) - end(phase_target.rz) <  slack_rendezvous_position)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.rz) - end(phase_target.rz) > -slack_rendezvous_position)

    # rendezvous speed soft constraint
    slack_rendezvous_speed = mocp.add_variable('slack_rendezvous_speed', init=3.0)
    mocp.add_constraint(slack_rendezvous_speed > 0)
    mocp.add_objective(10.0 * slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vx) - end(phase_target.vx) <  slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vx) - end(phase_target.vx) > -slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vy) - end(phase_target.vy) <  slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vy) - end(phase_target.vy) > -slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vz) - end(phase_target.vz) <  slack_rendezvous_speed)
    mocp.add_constraint(end(phase_S2_rendezvous_burn.vz) - end(phase_target.vz) > -slack_rendezvous_speed)

    if True: # Toggle for testing
        for phase_name in ['S2_coast1', 'S2_burn1', 'S2_coast2', 'S2_rendezvous_burn', 'S1_boostback', 'S1_coast2', 'S1_landing']:
            mocp.phases[phase_name].change_time_resolution(12)

        mocp.phases['ascent'].change_time_resolution(18)
        mocp.phases['target'].change_time_resolution(12)
        mocp.phases['S1_coast1'].change_time_resolution(4)

        (solver_output, slack_values) = mocp.solve()

        # Check slack variables
        max_slack = max([e[1] for e in slack_values])
        assert max_slack < 1e-6, str([e[0] for e in slack_values if max_slack*0.99 < e[1]])
        print('max_slack', max_slack, str([e[0] for e in slack_values if max_slack*0.99 < e[1]]))

    # Interpolate resulting tajectory
    interpolated_results = dict()
    for phase_name in mocp.phases:
        interpolated_results[phase_name] = mocp.phases[phase_name].interpolate()
        interpolated_results[phase_name]['start_time'] = 0.0

    for pair in consecutive_phases: # Fix phase start times
        interpolated_results[pair[1].phase_name]['start_time'] = interpolated_results[pair[0].phase_name]['start_time'] + interpolated_results[pair[0].phase_name]['duration']
    interpolated_results['target']['start_time'] = interpolated_results['S2_rendezvous_burn']['start_time'] + interpolated_results['S2_rendezvous_burn']['duration'] - interpolated_results['target']['duration']

    for phase_name in mocp.phases:
        del interpolated_results[phase_name]['tau_grid']

    print('phase durations:')
    for n in interpolated_results:
        print((n+'               ')[:18], interpolated_results[n]['duration']*p.scale.time)


    import json
    with open('trajectory.json', 'w') as f:
        pairs = mocp.get_symbol_value_pairs()
        json.dump({
            'phases':interpolated_results,
            'scale':p.scale._asdict(),
            'values':{e[0].name():e[1] for e in pairs}
        }, f)

    return None

def normalize(v):
    return v/casadi.norm_2(v)

def f2s(f):
    return ("{:21.6f}".format(f))


def add_phase_continuity_constraint(mocp, phase_end, phase_start, **kwargs):
    if kwargs['include_mass']:
        mocp.add_constraint(mocp.start(phase_start.mass) == mocp.end(phase_end.mass))
    mocp.add_constraint(mocp.start(phase_start.rx) == mocp.end(phase_end.rx))
    mocp.add_constraint(mocp.start(phase_start.ry) == mocp.end(phase_end.ry))
    mocp.add_constraint(mocp.start(phase_start.rz) == mocp.end(phase_end.rz))
    mocp.add_constraint(mocp.start(phase_start.vx) == mocp.end(phase_end.vx))
    mocp.add_constraint(mocp.start(phase_start.vy) == mocp.end(phase_end.vy))
    mocp.add_constraint(mocp.start(phase_start.vz) == mocp.end(phase_end.vz))

def create_rocket_stage_phase(mocp, phase_name, p, **kwargs):
    engine_parameters = kwargs['engine_parameters']
    atmosphere_parameters = kwargs['atmosphere_parameters']
    init_mass = kwargs['init_mass']
    has_steering_unit_vector_constraint = kwargs.get('has_steering_unit_vector_constraint', True)

    if init_mass is None:
        assert atmosphere_parameters is None # Must have mass for drag-acceleration
        assert engine_parameters is None # Must have mass for thrust-acceleration

    if phase_name == 'target':
        init_position = [p.target_position_x, p.target_position_y, p.target_position_z]
        init_velocity = [p.target_velocity_x, p.target_velocity_y, p.target_velocity_z]
    else:
        init_position = [p.launch_site_x, p.launch_site_y, p.launch_site_z]
        init_velocity = [0.0, 0.0, 0.0]

    duration = mocp.create_phase(phase_name, init=0.001, n_intervals=3)

    if init_mass is not None:
        # Total vessel mass
        mass  = mocp.add_trajectory(phase_name, 'mass', init=init_mass)

    # ECEF position vector
    rx    = mocp.add_trajectory(phase_name, 'rx', init=init_position[0])
    ry    = mocp.add_trajectory(phase_name, 'ry', init=init_position[1])
    rz    = mocp.add_trajectory(phase_name, 'rz', init=init_position[2])
    r_vec = casadi.vertcat(rx,ry,rz)

    mocp.add_path_constraint(sumsqr(r_vec) > 0.99)

    # ECEF velocity vector
    vx    = mocp.add_trajectory(phase_name, 'vx', init=init_velocity[0])
    vy    = mocp.add_trajectory(phase_name, 'vy', init=init_velocity[1])
    vz    = mocp.add_trajectory(phase_name, 'vz', init=init_velocity[2])
    v_vec = casadi.vertcat(vx,vy,vz)


    if engine_parameters is not None:
        launch_site_param = DM([p.launch_site_x, p.launch_site_y, p.launch_site_z])
        init_up_direction = normalize(launch_site_param)

        # ECEF steering unit vector
        ux    = mocp.add_trajectory(phase_name, 'ux', init=init_up_direction[0])
        uy    = mocp.add_trajectory(phase_name, 'uy', init=init_up_direction[1])
        uz    = mocp.add_trajectory(phase_name, 'uz', init=init_up_direction[2])
        u_vec = casadi.vertcat(ux,uy,uz)

        if has_steering_unit_vector_constraint:
            # u is a unit vector
            mocp.add_constraint(sumsqr(mocp.start(u_vec)) == 1.0)

        omega_x = mocp.add_trajectory(phase_name, 'omega_x', init=0.0)
        omega_y = mocp.add_trajectory(phase_name, 'omega_y', init=0.0)
        omega_z = mocp.add_trajectory(phase_name, 'omega_z', init=0.0)
        omega_vec = casadi.vertcat(omega_x,omega_y,omega_z)

        alpha_x = mocp.add_trajectory(phase_name, 'alpha_x', init=0.0)
        alpha_y = mocp.add_trajectory(phase_name, 'alpha_y', init=0.0)
        alpha_z = mocp.add_trajectory(phase_name, 'alpha_z', init=0.0)
        alpha_vec = casadi.vertcat(alpha_x,alpha_y,alpha_z)

        du_dt = casadi.cross(omega_vec, u_vec) + u_vec * 10 * (1.0 - sumsqr(u_vec))
        mocp.set_derivative(ux, du_dt[0])
        mocp.set_derivative(uy, du_dt[1])
        mocp.set_derivative(uz, du_dt[2])


        mocp.set_derivative(omega_x, alpha_x)
        mocp.set_derivative(omega_y, alpha_y)
        mocp.set_derivative(omega_z, alpha_z)

        mocp.add_path_constraint(dot(u_vec, omega_vec) == 1e-20)

        mocp.add_mean_objective(1e-6 * sumsqr(alpha_vec))
        mocp.add_mean_objective(1e-6 * sumsqr(omega_vec))

        mocp.add_path_output('u_mag_err', sumsqr(u_vec)-1)

        throttle = mocp.add_trajectory(phase_name, 'throttle', init=1.0)
        throttle_rate = mocp.add_trajectory(phase_name, 'throttle_rate', init=0.0)



    r_squared = sumsqr(r_vec) + 1e-8
    v_squared = sumsqr(v_vec) + 1e-8
    airspeed = v_squared**0.5
    orbital_radius = r_squared**0.5
    up_direction = r_vec / orbital_radius
    altitude = orbital_radius - 1.0
    planet_angular_velocity = casadi.DM([0.0, -p.body_angular_rate, 0.0]) # KSP coordinates: planet angular velocity points in negative y direction
    a_gravity = -(r_squared**(-1.5)) * r_vec # Gravitational acceleration (mu=1 is omitted)
    a_coriolis = -2 * cross(planet_angular_velocity, v_vec) # Coriolis acceleration
    a_centrifugal = -cross(planet_angular_velocity, cross(planet_angular_velocity, r_vec)) # Centrifugal acceleration
    a_vec = a_gravity + a_coriolis + a_centrifugal # Total acceleration on vehicle

    # Atmosphere
    if atmosphere_parameters is not None:
        atmosphere_fraction = casadi.exp(-altitude/atmosphere_parameters.scale_height)
        air_density = atmosphere_parameters.air_density_MSL * atmosphere_fraction
        dynamic_pressure = 0.5 * air_density * v_squared
        mocp.add_path_output('dynamic_pressure', dynamic_pressure)

        # Drag force
        drag_force_vector = (-0.5 * atmosphere_parameters.drag_area * atmosphere_parameters.air_density_MSL) * atmosphere_fraction * airspeed * v_vec
        a_vec += (drag_force_vector / mass)

    # Engine thrust
    if engine_parameters is not None:
        if atmosphere_parameters is None:
            exhaust_velocity = engine_parameters.exhaust_velocity_vacuum
        else:
            exhaust_velocity = engine_parameters.exhaust_velocity_vacuum + \
                atmosphere_fraction * (engine_parameters.exhaust_velocity_sea_level - engine_parameters.exhaust_velocity_vacuum)
        engine_mass_flow = engine_parameters.max_mass_flow * throttle
        thrust = exhaust_velocity * engine_mass_flow
        mocp.add_path_output('thrust', thrust)
        thrust_acceleration = thrust / mass
        mocp.add_path_output('thrust_acceleration', thrust_acceleration)
        thrust_force_vector = thrust * u_vec
        a_vec += (thrust_force_vector / mass)


    ## Differential equations
    if engine_parameters is not None:
        mocp.set_derivative(mass, -engine_mass_flow)
        mocp.set_derivative(throttle, throttle_rate)
    elif init_mass is not None:
        mocp.set_derivative(mass, 0.0) # Atmospheric coast: mass is constant

    for i in range(3):
        mocp.set_derivative(r_vec[i], v_vec[i])
        mocp.set_derivative(v_vec[i], a_vec[i])


    ## Constraints

    # Duration is positive and bounded
    mocp.add_constraint(duration > 0.004)
    if phase_name == 'target':
        mocp.add_constraint(duration < 20.0)
    else:
        mocp.add_constraint(duration < 10.0)

    # Mass is positive
    if init_mass is not None:
        mocp.add_path_constraint(mass > 1e-6)


    if engine_parameters is not None:
        # Do not point down
        mocp.add_path_constraint(dot(up_direction, u_vec) > -0.2)

        # Throttle limits
        mocp.add_path_constraint(throttle < 0.90)
        mocp.add_path_constraint(throttle > 0.05)

        # Throttle rate reduction
        mocp.add_mean_objective(1e-5 * throttle_rate**2)

    # Q alpha constraint
    if (atmosphere_parameters is not None) and (engine_parameters is not None):
        lateral_airspeed = casadi.sqrt(casadi.sumsqr(v_vec-(dot(v_vec,u_vec)*u_vec))+1e-8)
        lateral_dynamic_pressure = 0.5 * air_density * lateral_airspeed * airspeed # Same as Q * sin(alpha), but without trigonometry
        mocp.add_path_output('lateral_dynamic_pressure', lateral_dynamic_pressure)
        mocp.add_mean_objective(  (lateral_dynamic_pressure/atmosphere_parameters.lateral_dynamic_pressure_limit )**8 )

    ## Outputs
    mocp.add_path_output('altitude', altitude)
    mocp.add_path_output('airspeed', airspeed)
    mocp.add_path_output('vertical_speed', dot(up_direction,v_vec))
    mocp.add_path_output('horizontal_speed_ECEF', casadi.norm_2( v_vec - (dot(up_direction,v_vec)*(up_direction) ) ))

    if engine_parameters is not None:
        mocp.add_path_output('AoA', casadi.acos(dot(v_vec,u_vec)/airspeed)*180.0/pi)
        mocp.add_path_output('pitch', 90.0-casadi.acos(dot(up_direction,u_vec))*180.0/pi)

    variables = locals().copy()
    RocketStagePhase = collections.namedtuple('RocketStagePhase',sorted(list(variables.keys())))
    return RocketStagePhase(**variables)

def get_parameters():
    config = load_config()
    parameter_selection = ['total_mass_ascent', 'vacuum_thrust_ascent', 'total_mass_S2', 'vacuum_thrust_S2', 'touchdown_mass', 'landing_site_x', 'landing_site_y', 'landing_site_z', 'dynamic_pressure_limit', 'lateral_dynamic_pressure_limit', 'drag_area', 'launch_site_x', 'launch_site_y', 'launch_site_z', 'target_position_x', 'target_position_y', 'target_position_z', 'target_velocity_x', 'target_velocity_y', 'target_velocity_z', 'body_angular_rate', 'air_density_MSL', 'scale_height', 'launch_direction_guess_x', 'launch_direction_guess_y', 'launch_direction_guess_z', 'orbit_speed_guess', 'exhaust_velocity_sea_level_ascent', 'exhaust_velocity_vacuum_ascent', 'exhaust_velocity_sea_level_S2', 'exhaust_velocity_vacuum_S2']
    scale = get_scale(config['body_radius'], config['gravitational_parameter'], config['total_mass_ascent'])
    config = {e:config[e]/get_scale_factor(e, scale) for e in config if e in parameter_selection}
    S1_engine_parameters = {
        'vacuum_thrust': config['vacuum_thrust_ascent'],
        'exhaust_velocity_sea_level': config['exhaust_velocity_sea_level_ascent'],
        'exhaust_velocity_vacuum': config['exhaust_velocity_vacuum_ascent'],
        'max_mass_flow': config['vacuum_thrust_ascent'] / config['exhaust_velocity_vacuum_ascent']
    }
    S2_engine_parameters = {
        'vacuum_thrust': config['vacuum_thrust_S2'],
        'exhaust_velocity_sea_level': config['exhaust_velocity_sea_level_S2'],
        'exhaust_velocity_vacuum': config['exhaust_velocity_vacuum_S2'],
        'max_mass_flow': config['vacuum_thrust_S2'] / config['exhaust_velocity_vacuum_S2']
    }
    atmosphere_parameters = {
        'dynamic_pressure_limit': config['dynamic_pressure_limit'],
        'lateral_dynamic_pressure_limit': config['lateral_dynamic_pressure_limit'],
        'drag_area': config['drag_area'],
        'air_density_MSL': config['air_density_MSL'],
        'scale_height': config['scale_height'],
    }
    config = {k:config[k] for k in config if k not in ['vacuum_thrust_ascent', 'exhaust_velocity_sea_level_ascent', 'exhaust_velocity_vacuum_ascent', 'vacuum_thrust_ascent', 'exhaust_velocity_vacuum_ascent', 'vacuum_thrust_S2', 'exhaust_velocity_sea_level_S2', 'exhaust_velocity_vacuum_S2', 'vacuum_thrust_S2', 'exhaust_velocity_vacuum_S2', 'dynamic_pressure_limit', 'lateral_dynamic_pressure_limit', 'drag_area', 'air_density_MSL', 'scale_height']}
    EngineParameters = collections.namedtuple('EngineParameters',sorted(list(S1_engine_parameters.keys())))
    AtmosphereParameters = collections.namedtuple('AtmosphereParameters',sorted(list(atmosphere_parameters.keys())))
    config['S1_engine_parameters'] = EngineParameters(**S1_engine_parameters)
    config['S2_engine_parameters'] = EngineParameters(**S2_engine_parameters)
    config['atmosphere_parameters'] = AtmosphereParameters(**atmosphere_parameters)
    config['scale'] = scale
    Parameters = collections.namedtuple('Parameters',sorted(list(config.keys())))
    return Parameters(**config)

def get_scale_factor(variable_name, scales):
    if not isinstance(scales, dict):
        scales = scales._asdict()
    scales['thrust'] = scales['force']
    scales['position'] = scales['length']
    scales['height'] = scales['length']
    scales['altitude'] = scales['length']
    scales['velocity'] = scales['speed']
    scales['angular_rate'] = 1.0/scales['time']
    scales['omega_'] = 1.0/scales['time']
    scales['alpha_'] = 1.0/(scales['time']**2)
    scales['direction'] = 1.0
    scales['throttle'] = 1.0
    scales['_site'] = scales['length']

    for s in scales:
        if s in variable_name:
            return scales[s]

    scales_full_match = dict()
    scales_full_match['rx'] = scales['length']
    scales_full_match['ry'] = scales['length']
    scales_full_match['rz'] = scales['length']

    scales_full_match['vx'] = scales['speed']
    scales_full_match['vy'] = scales['speed']
    scales_full_match['vz'] = scales['speed']

    scales_full_match['ux'] = 1.0
    scales_full_match['uy'] = 1.0
    scales_full_match['uz'] = 1.0
    scales_full_match['AoA'] = 1.0
    scales_full_match['pitch'] = 1.0
    scales_full_match['u_mag_err'] = 1.0

    for s in scales_full_match:
        if s == variable_name:
            return scales_full_match[s]

    raise NotImplementedError()

def get_scale(length, mu, mass):
    time = (mu)**(-1.0/2) * (length)**(3.0/2)
    speed = length / time
    area = length**2
    acceleration = speed / time
    force = mass * acceleration
    mass_flow = mass / time
    density = mass / (length**3)
    pressure = force / area
    del mu
    variables = locals().copy()
    Scale = collections.namedtuple('Scale',sorted(list(variables.keys())))
    return Scale(**variables)

def load_config():
    with open('mission.json', 'r') as f:
        return json.load(f)

def generate_figures():
    import matplotlib
    import matplotlib.pyplot as plt
    import numpy as np
    import json

    with open('trajectory.json', 'r') as f:
        data = json.load(f)

    phases = data['phases']
    scale = data['scale']

    # Profile plot
    e1 = normalize(DM([phases['ascent']['trajectories']['rx'][0], phases['ascent']['trajectories']['ry'][0], phases['ascent']['trajectories']['rz'][0]]))
    e2 = -normalize(DM([phases['target']['trajectories']['rx'][0], phases['target']['trajectories']['ry'][0], phases['target']['trajectories']['rz'][0]]))
    e3 = normalize(cross(e1,e2))
    e2 = normalize(cross(e3,e1))
    R = casadi.horzcat(e1,e2,e3).T

    fig, ax = plt.subplots()
    fig.set_size_inches(fig.get_size_inches() * 0.6)
    to_km = scale['length']/1000
    a = np.linspace(0,2*pi,200)
    ax.plot(np.cos(a)*to_km, np.sin(a)*to_km, 'k')
    for phase_name in phases:
        position_trajectory = R @ DM([phases[phase_name]['trajectories']['rx'],phases[phase_name]['trajectories']['ry'],phases[phase_name]['trajectories']['rz']])
        position_trajectory = np.array(position_trajectory)
        ax.plot(position_trajectory[1]*to_km, position_trajectory[0]*to_km)

    ax.set(xlabel='y (km)', ylabel='x (km)', title='Path Profile')
    ax.axis('equal')
    ax.grid()
    fig.tight_layout()
    fig.savefig('fig/trajectory_profile.png', dpi=288)
    ax.set_xlim(-300,300)
    ax.set_ylim(-300+600,300+600)
    fig.tight_layout()
    fig.savefig('fig/trajectory_profile_zoomed.png', dpi=288)
    plt.close('all')


    fig, ax = plt.subplots()
    fig.set_size_inches(fig.get_size_inches() * 0.6)
    to_km = scale['length']/1000
    a = np.linspace(0,2*pi,200)
    ax.plot(np.cos(a)*to_km, np.sin(a)*to_km, 'k')
    for phase_name in phases:
        position_trajectory = R @ DM([phases[phase_name]['trajectories']['rx'],phases[phase_name]['trajectories']['ry'],phases[phase_name]['trajectories']['rz']])
        position_trajectory = np.array(position_trajectory)
        ax.plot(position_trajectory[2]*to_km, position_trajectory[0]*to_km)

    ax.set(xlabel='z (km)', ylabel='x (km)', title='Path Parallel')
    ax.axis('equal')
    ax.grid()
    fig.tight_layout()
    fig.savefig('fig/trajectory_parallel.png', dpi=288)
    ax.set_xlim(-300,300)
    ax.set_ylim(-300+600,300+600)
    fig.tight_layout()
    fig.savefig('fig/trajectory_parallel_zoomed.png', dpi=288)
    plt.close('all')


    # Timeseries plots
    paths = \
        sorted(list(set([('trajectories',k) for e in phases.values() for k in e['trajectories'].keys()])))+\
        sorted(list(set([('outputs',k) for e in phases.values() for k in e['outputs'].keys()])))


    for path in paths:
        fig, ax = plt.subplots()
        fig.set_size_inches(fig.get_size_inches() * 0.6)
        for phase_name in phases:
            path_scale = get_scale_factor(path[1], scale)
            ax.set(xlabel='Time (s)', ylabel=path[1], title=path[1])

            if path[1] in phases[phase_name][path[0]]:
                if path[1] in ['mass', 'rx', 'ry', 'rz', 'altitude']:
                    path_scale /= 1000.0
                elif 'pressure' in path[1]:
                    path_scale /= 1000.0
                t_grid = phases[phase_name]['start_time'] + phases[phase_name]['duration'] * np.linspace(0.0,1.0,len(phases[phase_name][path[0]][path[1]]))
                ax.plot(scale['time'] * t_grid,
                    np.array(phases[phase_name][path[0]][path[1]])*path_scale)

        ax.grid()
        fig.tight_layout()
        fig.savefig('fig/trajectory_' + path[1] + '.png', dpi=288)
        plt.close('all')

def panelize_figures():
    from PIL import Image
    import math
    filelist = glob.glob("fig/*.png")
    images = [Image.open(x) for x in filelist]
    max_height = max([i.height for i in images])
    max_width = max([i.width for i in images])
    n_columns = math.ceil(math.sqrt(len(images)) * 1.3)
    n_rows = math.ceil(len(images)/n_columns)
    out_img = Image.new('RGB', (n_columns * max_width, n_rows * max_height), color=(255,255,255))

    for j in range(n_columns):
        for i in range(n_rows):
            k = i * n_columns + j
            if k < len(images):
                out_img.paste(images[k], (j * max_width, i * max_height))
    out_img.save('fig/0_panel.png')


if __name__ == '__main__':

    from time import time
    t_start = time()
    optimizer()
    t_end = time()
    print('solve time:', f2s(t_end-t_start), 'sec')

    print('generating figures ...')
    if not os.path.exists('fig'): os.mkdir('fig')
    filelist = glob.glob("fig/*")
    for f in filelist:
        os.remove(f)
    generate_figures()
    panelize_figures()
    import sys
    sys.exit(42)
