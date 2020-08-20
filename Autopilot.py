from krpc_utils import TelemetryStreams
import time
import math
import numpy as np
import json
from krpc_utils import quaternion_rotation

class BeforePhaseTimespanError(Exception):
    pass

class AfterPhaseTimespanError(Exception):
    pass


class Autopilot:
    def __init__(self, connection, ut_stream):
        self.connection = connection
        self.telemetry_streams = None

        self.position_error_body = None
        self.velocity_error_body = None
        self.should_reboot = False
        self.inactive = False
        self.should_disable = False
        self.estimated_RCS_angular_acceleration = None
        self.touchdown_guidance_t_init = -10.0

        with open('mission.json', 'r') as f:
            self.mission = json.load(f)

        with open('trajectory.json', 'r') as f:
            self.trajectory = json.load(f)

        # identify target vessel
        all_vessels = connection.space_center.vessels
        vessel_spawn_times = [ut_stream() - e.met for e in all_vessels]
        is_target_vehicle = [math.fabs(t-self.mission['target_spawn_time'])<2.0 for t in vessel_spawn_times]
        assert sum(is_target_vehicle) == 1, 'Target vessel could not be identified'
        target_index = [e[0] for e in enumerate(is_target_vehicle) if e[1]][0]
        assert all_vessels[target_index].name == self.mission['target_name']
        connection.space_center.target_vessel = all_vessels[target_index]
        self.target_vessel = all_vessels[target_index]
        self.target_position_stream = self.connection.add_stream(self.target_vessel.position, self.target_vessel.orbit.body.reference_frame)

        # convenience values
        self.scale_length = self.trajectory['scale']['length']
        self.scale_speed = self.trajectory['scale']['speed']
        self.scale_time = self.trajectory['scale']['time']
        self.scale_mass = self.trajectory['scale']['mass']
        self.scale_acceleration = self.trajectory['scale']['acceleration']

        self.phases = self.trajectory['phases']
        self.mission_time_offset = self.mission['t0'] - self.phases['target']['start_time'] * self.scale_time

        self.active_vessel = connection.space_center.active_vessel
        frame = self.active_vessel.orbit.body.reference_frame

        # Determine expected state / vessel from flight plan
        mission_time = ut_stream() - self.mission_time_offset
        (self.active_phase, self.next_burn_phase) = self.find_planned_active_phase(max(mission_time, 1e-3))

        if self.active_phase in self.phases and  self.next_burn_phase in self.phases:
            # Find vessel matching the expected phase/stage and switch to it
            expected_state = self.interpolate_phase(self.active_phase, max(mission_time, 1e-3))
            expected_position = get_position(expected_state) * self.scale_length
            expected_velocity = get_velocity(expected_state) * self.scale_speed
            expected_mass = expected_state['mass'] * self.scale_mass

            vessel_deviation_measures = [{
                'vessel':v,
                'position':np.linalg.norm(np.array(v.position(frame)) - expected_position),
                'velocity':np.linalg.norm(np.array(v.velocity(frame)) - expected_velocity) ,
            } for v in all_vessels]

            candidate_vessels = [v for v in vessel_deviation_measures if v['position'] < 10000.0 and v['velocity'] < 300.0]

            if len(candidate_vessels) > 1:
                candidate_vessels = [candidate_vessels[np.argmin([abs(v['vessel'].mass/expected_mass-1.0) + v['position']/100 + v['velocity']/10 for v in candidate_vessels])]]

            assert len(candidate_vessels) == 1 # TODO fix if its a problem
            connection.space_center.active_vessel = candidate_vessels[0]['vessel']
            self.active_vessel = candidate_vessels[0]['vessel']
            self.control = self.active_vessel.control
            self.telemetry_streams = TelemetryStreams(connection, self.active_vessel)
        else:
            self.inactive = True
            self.should_disable = True


    def interpolate_phase(self, phase_name, mission_time):

        phase_t_start = self.phases[phase_name]['start_time'] * self.scale_time
        phase_duration = self.phases[phase_name]['duration'] * self.scale_time

        n_samples = len(next(iter(self.phases[phase_name]['trajectories'].values())))

        tau = (mission_time - phase_t_start) / phase_duration
        index = tau * (n_samples - 1)
        index_low = math.floor(index)
        index_high = math.ceil(index)

        if index_high >= n_samples:
            raise AfterPhaseTimespanError()

        if index_low < 0:
            raise BeforePhaseTimespanError()

        weight_high = index - index_low
        result = dict()
        for s in ['trajectories', 'outputs']:
            for n in self.phases[phase_name][s]:
                assert len(self.phases[phase_name][s][n]) == n_samples
                value_low = self.phases[phase_name][s][n][index_low]
                value_high = self.phases[phase_name][s][n][index_high]
                assert not n in result
                result[n] = value_low + weight_high * (value_high - value_low)

        return result

    def control_update(self, ut, delta_t):
        if self.inactive:
            if hasattr(self,'control'):
                self.control.throttle = 0.0
                self.control.pitch = 0.0
                self.control.roll  = 0.0
                self.control.yaw   = 0.0
            return

        mission_time = ut - self.mission_time_offset
        t_end = (self.phases[self.next_burn_phase]['start_time']+self.phases[self.next_burn_phase]['duration']) * self.scale_time
        t_start = (self.phases[self.next_burn_phase]['start_time']) * self.scale_time
        t_go = t_end - mission_time
        t_ign = t_start - mission_time

        telemetry = self.telemetry_streams.read_telemetry()

        # Stop time warp at the right time
        if 1 < t_ign < 40 and telemetry.rails_warp_factor > 1:
            self.connection.space_center.rails_warp_factor = 1

        if 1 < t_ign < 10 and telemetry.rails_warp_factor > 0:
            self.connection.space_center.rails_warp_factor = 0

        # Activate stage before liftoff
        if telemetry.available_thrust == 0.0 and mission_time > -0.5:
            self.control.activate_next_stage()
            time.sleep(0.05)

        # Gear down just before landing
        if self.next_burn_phase == 'S1_landing' and (9.5 < t_go < 10):
            self.control.gear = True

        # Gear up after liftoff
        if 1 < mission_time < 1.5:
            self.control.gear = False

        if self.next_burn_phase == 'S1_landing' and (t_go < 2):
            # This "touchdown guidance" control the last seconds of the landing burn to ensure a precise touchdown.

            a_ref = (self.phases[self.next_burn_phase]['outputs']['thrust_acceleration'][-1] * self.scale_acceleration) - 9.81
            v_ref = 0.5 # Touchdown speed
            t_ref = 1.5 # Duration of the transition phase from constant acceleration to constant speed
            vessel_height_trim = 10.0 # Height constant tuned for a particular vessel

            (self.touchdown_guidance_t_init, v_cmd, a_cmd) = solve_touchdown_guidance(
                self.touchdown_guidance_t_init,
                telemetry.surface_altitude - vessel_height_trim,
                t_ref,v_ref,a_ref)

            a_max = telemetry.available_thrust / telemetry.mass
            throttle_command = (a_cmd+9.81)/a_max + 0.1 * (v_cmd - telemetry.vertical_speed) # proportional controller for the descent speed
            self.control.throttle = throttle_command

            # Steer to point straight up
            ref_steering_body = np.array(quaternion_rotation(telemetry.rotation, np.array(telemetry.position)/np.linalg.norm(telemetry.position)))
            angular_acceleration_command_body = \
                + 6.0 * cross_y(ref_steering_body) \
                - 4.0 * telemetry.angular_velocity_body

            max_angular_acceleration = np.array(telemetry.available_torque[0]) / np.array(telemetry.moment_of_inertia)
            PRY_command = -angular_acceleration_command_body / max_angular_acceleration
            self.control.pitch = PRY_command[0]
            self.control.roll  = PRY_command[1]
            self.control.yaw   = PRY_command[2]

            # Stop all control inputs on ground contact
            if telemetry.situation.name == 'landed':
                self.control.throttle = 0.0
                self.control.rcs = False
                self.control.pitch = 0.0
                self.control.roll  = 0.0
                self.control.yaw   = 0.0
                self.should_reboot = True
                self.inactive = True
                time.sleep(1.0)
                return

        else:
            try:
                # This is the burn tracking controller

                vessel_ref_state = self.interpolate_phase(self.next_burn_phase, mission_time)
                ref_position = get_position(vessel_ref_state) * self.scale_length
                ref_velocity = get_velocity(vessel_ref_state) * self.scale_speed
                ref_steering = get_steering(vessel_ref_state)
                ref_thrust_acceleration = vessel_ref_state['thrust_acceleration'] * self.scale_acceleration
                ref_throttle = ref_thrust_acceleration * telemetry.mass / telemetry.available_thrust

                self.position_error = np.array(telemetry.position) - ref_position
                self.velocity_error = np.array(telemetry.velocity) - ref_velocity

                self.position_error_body = np.array(quaternion_rotation(telemetry.rotation, self.position_error))
                self.velocity_error_body = np.array(quaternion_rotation(telemetry.rotation, self.velocity_error))
                ref_steering_body = np.array(quaternion_rotation(telemetry.rotation, ref_steering))


                K = [0.13, 2.1, 6.0, 4.0]
                if -1 < t_go < 10: # Stop tracking position towards the end of the burn, so that the velocity error can be reduced
                    K[0] = 0.0

                # Controll law for pitch+yaw steering and roll damping
                saturation = lambda x,s: np.maximum(np.minimum(x,s),-s)
                angular_acceleration_command_body = \
                    - K[0] * saturation(cross_y(self.position_error_body / 20), 15.0) \
                    - K[1] * saturation(cross_y(self.velocity_error_body / 20), 1.0) \
                    + K[2] * cross_y(ref_steering_body) \
                    - K[3] * telemetry.angular_velocity_body

                max_angular_acceleration = np.array(telemetry.available_torque[0]) / np.array(telemetry.moment_of_inertia)
                PRY_command = -angular_acceleration_command_body / max_angular_acceleration
                self.control.pitch = PRY_command[0]
                self.control.roll  = PRY_command[1]
                self.control.yaw   = PRY_command[2]

                # Throttle controller, track the longitudinal speed and position error
                throttle_command = ref_throttle - 0.03 * self.velocity_error_body[1] - saturation(0.02 * self.position_error_body[1],0.3)
                self.control.throttle = throttle_command

            except BeforePhaseTimespanError:
                # This is the RCS attitude controller, which is active before each burn.
                # It orients the vessel to the initial steering direction.

                if self.next_burn_phase != 'ascent':
                    if not telemetry.rcs:
                        self.control.rcs = True

                    if self.estimated_RCS_angular_acceleration is None:
                        # The kRPC calculation for RCS torque is broken.
                        # This performs an empirical estimation of the vessels angular acceleration.
                        self.estimated_RCS_angular_acceleration = self.estimate_RCS_angular_acceleration()

                    t_start = self.phases[self.next_burn_phase]['start_time'] * self.scale_time
                    vessel_ref_state = self.interpolate_phase(self.next_burn_phase, t_start)
                    ref_steering = get_steering(vessel_ref_state)
                    ref_steering_body = np.array(quaternion_rotation(telemetry.rotation, ref_steering))

                    # Attitude control law
                    angular_acceleration_command_body = \
                        + 1.0 * cross_y(ref_steering_body) \
                        - 4.0 * telemetry.angular_velocity_body

                    max_angular_acceleration = np.array([1.0, 10, 1.0]) * self.estimated_RCS_angular_acceleration
                    PRY_command = -angular_acceleration_command_body / max_angular_acceleration
                    self.control.pitch = PRY_command[0]
                    self.control.roll  = PRY_command[1]
                    self.control.yaw   = PRY_command[2]
                    self.control.throttle = 0.0

            except AfterPhaseTimespanError:
                # Engine cutoff at the burn end
                self.control.rcs = False
                time.sleep(0.05)
                self.control.throttle = 0.0
                if self.next_burn_phase == 'ascent': # Separate the first and second stage
                    time.sleep(0.5)
                    self.control.activate_next_stage()
                time.sleep(1.0)
                self.should_reboot = True
                self.inactive = True


    def get_status_update(self, ut, delta_t):
        text = ""
        if not self.inactive:
            mission_time = ut - self.mission_time_offset

            try:
                target_ref_state = self.interpolate_phase('target', mission_time)
                target_ref_position = get_position(target_ref_state) * self.scale_length
                target_position = np.array(self.target_position_stream())
                target_position_error = float(np.linalg.norm(target_ref_position - target_position))
            except:
                target_position_error = math.nan

            t_end = (self.phases[self.next_burn_phase]['start_time']+self.phases[self.next_burn_phase]['duration']) * self.scale_time
            t_start = (self.phases[self.next_burn_phase]['start_time']) * self.scale_time

            text += "MET         (s)  " + f2s(mission_time) + "\n"
            text += "T IGN       (s)  " + f2s(t_start - mission_time) + "\n"
            text += "T GO        (s)  " + f2s(t_end - mission_time) + "\n"
            text += "TGT POS ERR (m)  " + f2s(target_position_error) + "\n"
            if self.position_error_body is not None:
                text += "POS ERR XYZ (m)  " + f2s(self.position_error_body) + "\n"
                text += "VEL ERR XYZ (m/s)" + f2s(self.velocity_error_body) + "\n"

        return text

    def find_planned_active_phase(self, mission_time):
        # Where are we in the mission, base on mission time?

        if mission_time < 0: return None

        active_list = list()
        for phase_name in self.phases:
            if phase_name != 'target':
                t_start = self.phases[phase_name]['start_time'] * self.scale_time
                t_end = t_start + self.phases[phase_name]['duration'] * self.scale_time
                if t_start <= mission_time and mission_time <= t_end:
                    active_list.append(phase_name)

        if active_list == ['S2_coast2']:    return ('S2_coast2', 'S2_rendezvous_burn')
        if len(active_list) == 0: return (None,None)
        if len(active_list) == 1: return (active_list[0],active_list[0])
        active_list.sort()
        if active_list == ['S1_coast1', 'S2_coast1']:     return ('S1_coast1', 'S1_boostback')
        if active_list == ['S1_boostback', 'S2_coast1']:  return ('S1_boostback', 'S1_boostback')
        if active_list == ['S1_coast2', 'S2_coast1']:     return ('S2_coast1', 'S2_burn1')
        if active_list == ['S1_coast2', 'S2_burn1']:      return ('S2_burn1', 'S2_burn1')
        if active_list == ['S1_coast2', 'S2_coast2']:     return ('S1_coast2', 'S1_landing')
        if active_list == ['S1_landing', 'S2_coast2']:    return ('S1_landing', 'S1_landing')

        raise NotImplementedError()

    def estimate_RCS_angular_acceleration(self):
        self.control.pitch = 0.0
        self.control.roll  = 0.0
        self.control.yaw   = 0.0
        self.control.rcs   = True
        self.control.throttle = 0.0
        time.sleep(1.0)

        delta_t = 1.5
        telemetry_start = self.telemetry_streams.read_telemetry()
        self.control.pitch = 1.0

        while True:
            telemetry_now = self.telemetry_streams.read_telemetry()
            if telemetry_now.ut - telemetry_start.ut > delta_t:
                break
            time.sleep(0.001)

        telemetry_end = self.telemetry_streams.read_telemetry()
        self.control.pitch = -1.0
        time.sleep(delta_t)
        self.control.pitch = 0.0
        delta_pitch_rate = telemetry_end.angular_velocity_body[0] - telemetry_start.angular_velocity_body[0]
        return math.fabs(delta_pitch_rate) / (telemetry_end.ut - telemetry_start.ut)


def get_position(s):
    return np.array([s['rx'],s['ry'],s['rz']])

def get_velocity(s):
    return np.array([s['vx'],s['vy'],s['vz']])

def get_steering(s):
    return np.array([s['ux'],s['uy'],s['uz']])


def f2s(f):
    if isinstance(f, np.ndarray):
        return f2s(f.tolist())
    if isinstance(f, list):
        return '; '.join([f2s(e) for e in f])
    return ("{:9.1f}".format(f))

def solve_touchdown_guidance(t_init,h,t_ref,v_ref,a_ref):
    t = t_init
    for i in range(3):
        h_v_a = touchdown_trajectory(t,t_ref,v_ref,a_ref)
        t -= 0.5 * (h_v_a[0]-h) / h_v_a[1] # Newtons method to find the relative time

    for i in range(3):
        h_v_a = touchdown_trajectory(t,t_ref,v_ref,a_ref)
        t -= (h_v_a[0]-h) / h_v_a[1] # Newtons method to find the relative time

    return (t, h_v_a[1], h_v_a[2])

def touchdown_trajectory(t,t_ref,v_ref,a_ref):
    # This implements a function h(t), which is the desired vessel height over time.
    # It transitions smoothly between constant acceleration (for t<0) and constant speed (for t>0).
    # The speed h'(t) and acceleration h''(t) are also returned.
    # v_ref is the constant touchdown speed.
    # a_ref is the constant descent deceleration.
    # t_ref is the duration of the transition phase.

    tmp_1=(0.5*a_ref)
    tmp_2=math.exp((-(t/t_ref)))
    tmp_3=(tmp_2+1.0)
    tmp_4=(math.log(tmp_3)*t_ref)
    tmp_5=((t_ref*((tmp_4+tmp_4)*tmp_1))/tmp_3)
    tmp_6=(tmp_2/t_ref)
    tmp_7=(tmp_1*(t_ref*(tmp_6/tmp_3)))

    return [((tmp_1*(tmp_4**2))-(v_ref*t)),
    (-(v_ref+((tmp_2*tmp_5)/t_ref))),
    (-((tmp_2*((((tmp_5/tmp_3)*tmp_6)-((t_ref*(tmp_7+tmp_7))/tmp_3))-(tmp_5/t_ref)))/t_ref))]


# Cross product with the e_y unit vector.
# The y body axis the vessel's forward direction.
def cross_y(x): return np.array([x[2],0.0,-x[0]])