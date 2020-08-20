import krpc
import collections
import numpy as np
from time import sleep

def quaternion_multiply(Q1,Q2):
    x0, y0, z0, w0 = Q1
    x1, y1, z1, w1 = Q2
    return [
        x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
        -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
        x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
        - x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
    ]

def quaternion_inverse(Q1):
    x0, y0, z0, w0 = Q1
    return [-x0, -y0, -z0, w0]

def quaternion_rotation(quat, vec):
    quat = list(quat)
    vec = list(vec)
    x = quaternion_multiply(quat, vec + [0])
    return quaternion_multiply(x, quaternion_inverse(quat))[0:3]

class TelemetryStreams:
    def __init__(self, connection, vessel):
        self.vessel = vessel
        self.body = self.vessel.orbit.body
        self.control = self.vessel.control
        self.planet_frame = self.body.reference_frame
        self.flight = self.vessel.flight(self.planet_frame)

        # data streams
        self.ut_stream                      = connection.add_stream(getattr, connection.space_center, 'ut')
        self.rails_warp_factor_stream       = connection.add_stream(getattr, connection.space_center, 'rails_warp_factor')
        self.position_stream                = connection.add_stream(self.vessel.position, self.planet_frame)
        self.velocity_stream                = connection.add_stream(self.vessel.velocity, self.planet_frame)
        self.direction_stream               = connection.add_stream(self.vessel.direction, self.planet_frame)
        self.angular_velocity_stream        = connection.add_stream(self.vessel.angular_velocity, self.planet_frame)
        self.mass_stream                    = connection.add_stream(getattr, self.vessel, 'mass')
        self.available_torque_stream        = connection.add_stream(getattr, self.vessel, 'available_torque')
        self.available_thrust_stream        = connection.add_stream(getattr, self.vessel, 'available_thrust')
        self.moment_of_inertia_stream       = connection.add_stream(getattr, self.vessel, 'moment_of_inertia')
        self.surface_altitude_stream        = connection.add_stream(getattr, self.flight, 'surface_altitude')
        self.mean_altitude_stream           = connection.add_stream(getattr, self.flight, 'mean_altitude')
        self.vertical_speed_stream          = connection.add_stream(getattr, self.flight, 'vertical_speed')
        self.situation_stream               = connection.add_stream(getattr, self.vessel, 'situation')
        self.throttle_command_stream        = connection.add_stream(getattr, self.control, 'throttle')
        self.pitch_command_stream           = connection.add_stream(getattr, self.control, 'pitch')
        self.roll_command_stream            = connection.add_stream(getattr, self.control, 'roll')
        self.yaw_command_stream             = connection.add_stream(getattr, self.control, 'yaw')
        self.up_command_stream              = connection.add_stream(getattr, self.control, 'up')
        self.right_command_stream           = connection.add_stream(getattr, self.control, 'right')
        self.forward_command_stream         = connection.add_stream(getattr, self.control, 'forward')
        self.rcs_stream                     = connection.add_stream(getattr, self.control, 'rcs')
        self.brakes_stream                  = connection.add_stream(getattr, self.control, 'brakes')
        self.rotation_stream                = connection.add_stream(self.vessel.rotation, self.planet_frame)

        dct = self.__dict__
        self.streams = {k:dct[k] for k in dct if k.endswith('_stream')}

    def angular_velocity_body(self):
        return quaternion_rotation(list(self.rotation_stream()), list(self.angular_velocity_stream()))

    def read_telemetry(self):
        values = {k[:-7]:self.streams[k]() for k in self.streams}
        values['angular_velocity_body'] = np.array(quaternion_rotation(values['rotation'], values['angular_velocity']))
        direction_up = np.array(values['position'])
        direction_up = direction_up / np.linalg.norm(direction_up)
        direction_north = np.array([0.0,1.0,0.0])
        direction_north = direction_north - np.dot(direction_north, direction_up) * direction_up
        direction_north = direction_north / np.linalg.norm(direction_north)
        direction_east = np.cross(direction_up, direction_north)
        values['direction_up'] = direction_up
        values['direction_north'] = direction_north
        values['direction_east'] = direction_east
        TelemetryValues = collections.namedtuple('TelemetryValues',sorted(list(values.keys())))
        return TelemetryValues(**values)

    def synced_loop(self, callback, samples_per_second):
        missed_sample_count = 0
        time_index_prev = 0

        while True:
            t = self.ut_stream()
            time_index = int(round(t * samples_per_second))

            if time_index > time_index_prev + 1:
                missed_sample_count += 1
                if missed_sample_count > 3:
                    print('missed samples', missed_sample_count)

            if time_index > time_index_prev:
                telemetry = self.read_telemetry()
                callback(time_index, telemetry, self, samples_per_second)
                time_index_prev = time_index

            sleep(1e-9)

