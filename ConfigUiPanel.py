
import json
import math
from MainUiPanel import panel_x_offset
from predict_orbit_BCBF import predict_orbit_BCBF
import numpy as np

class ConfigUiPanel:
    def __init__(self, connection):
        self.config = dict()

        try:
            with open('config.json', 'r') as f:
                self.config = json.load(f)
        except:pass

        self.canvas = connection.ui.stock_canvas
        self.connection = connection
        self.vessel = connection.space_center.active_vessel

        self.panel = self.canvas.add_panel()
        self.panel.rect_transform.size = (470, 645)
        self.panel.rect_transform.position = (panel_x_offset, -200)
        self.panel.rect_transform.anchor = (0.0, 1.0)
        self.panel.rect_transform.pivot = (0.0, 1.0)
        self.panel.visible = False

        row_spacing = 29
        margin_top = 58

        self.config_fields = {
            'total_mass_ascent':                  {'label': 'Liftoff Wet Mass', 'row': 0},
            'vacuum_thrust_ascent':               {'label': 'S1 Vacuum Thrust', 'row': 1},
            'exhaust_velocity_sea_level_ascent':  {'label': 'S1 Exhaust Velocity, Sea Level', 'row': 2},
            'exhaust_velocity_vacuum_ascent':     {'label': 'S1 Exhaust Velocity, Vacuum', 'row': 3},
            'total_mass_S2':                      {'label': 'S2 Wet Mass', 'row': 5.5},
            'vacuum_thrust_S2':                   {'label': 'S2 Vacuum Thrust', 'row': 6.5},
            'exhaust_velocity_sea_level_S2':      {'label': 'S2 Exhaust Velocity, Sea Level', 'row': 7.5},
            'exhaust_velocity_vacuum_S2':         {'label': 'S2 Exhaust Velocity, Vacuum', 'row': 8.5},
            'touchdown_mass':                     {'label': 'Touchdown Mass', 'row': 11},
            'landing_site_x':                     {'label': 'Landing Site X', 'row': 12},
            'landing_site_y':                     {'label': 'Landing Site Y', 'row': 13},
            'landing_site_z':                     {'label': 'Landing Site Z', 'row': 14},
            'dynamic_pressure_limit':             {'label': 'Dynamic Pressure Limit', 'row': 16.5},
            'lateral_dynamic_pressure_limit':     {'label': 'Lateral Dynamic Pressure Limit', 'row': 17.5},
            'drag_area':                          {'label': 'Drag Area', 'row': 18.5},
        }

        for name in self.config_fields:
            self.config_fields[name]['input'] = self.panel.add_input_field()
            self.config_fields[name]['input'].rect_transform.size = (200, 29)
            self.config_fields[name]['input'].rect_transform.position = (250, -margin_top-row_spacing*self.config_fields[name]['row'])
            self.config_fields[name]['input'].rect_transform.anchor = (0.0, 1.0)
            self.config_fields[name]['input'].rect_transform.pivot = (0.0, 1.0)
            if name in self.config:
                self.config_fields[name]['input'].value = str(self.config[name])
            else:
                self.config_fields[name]['input'].value = '0.0'
            self.config_fields[name]['input'].changed = False

            input_label = self.panel.add_text(self.config_fields[name]['label'])
            input_label.rect_transform.position = (20, -margin_top-row_spacing*self.config_fields[name]['row']-6)
            input_label.rect_transform.size = (300, 30)
            input_label.rect_transform.anchor = (0.0, 1.0)
            input_label.rect_transform.pivot = (0.0, 1.0)
            input_label.color = (1,1,1)
            input_label.size = 15


        group_label = self.panel.add_text("Ascent Parameters")
        group_label.rect_transform.position = (30, -margin_top-row_spacing*(-1.0)+5)
        group_label.rect_transform.size = (200, 30)
        group_label.rect_transform.anchor = (0.0, 1.0)
        group_label.rect_transform.pivot = (0.0, 1.0)
        group_label.color = (1,1,1)
        group_label.size = 20
        group_label.style = connection.ui.FontStyle.bold

        group_label = self.panel.add_text("Stage 2 Parameters")
        group_label.rect_transform.position = (30, -margin_top-row_spacing*(4.5)+5)
        group_label.rect_transform.size = (200, 30)
        group_label.rect_transform.anchor = (0.0, 1.0)
        group_label.rect_transform.pivot = (0.0, 1.0)
        group_label.color = (1,1,1)
        group_label.size = 20
        group_label.style = connection.ui.FontStyle.bold

        group_label = self.panel.add_text("Landing Parameters")
        group_label.rect_transform.position = (30, -margin_top-row_spacing*(10.0)+5)
        group_label.rect_transform.size = (200, 30)
        group_label.rect_transform.anchor = (0.0, 1.0)
        group_label.rect_transform.pivot = (0.0, 1.0)
        group_label.color = (1,1,1)
        group_label.size = 20
        group_label.style = connection.ui.FontStyle.bold

        group_label = self.panel.add_text("Other Parameters")
        group_label.rect_transform.position = (30, -margin_top-row_spacing*(15.5)+5)
        group_label.rect_transform.size = (200, 30)
        group_label.rect_transform.anchor = (0.0, 1.0)
        group_label.rect_transform.pivot = (0.0, 1.0)
        group_label.color = (1,1,1)
        group_label.size = 20
        group_label.style = connection.ui.FontStyle.bold


        self.button_copy_values_ascent = self.panel.add_button("Copy Current Values")
        self.button_copy_values_ascent.rect_transform.position = (270, -margin_top-row_spacing*(-1.0)+7)
        self.button_copy_values_ascent.rect_transform.size = (150, 28)
        self.button_copy_values_ascent.rect_transform.anchor = (0.0, 1.0)
        self.button_copy_values_ascent.rect_transform.pivot = (0.0, 1.0)
        self.button_copy_values_ascent_clicked = connection.add_stream(getattr,
        self.button_copy_values_ascent, 'clicked')

        self.button_copy_values_S2 = self.panel.add_button("Copy Current Values")
        self.button_copy_values_S2.rect_transform.position = (270, -margin_top-row_spacing*(4.5)+7)
        self.button_copy_values_S2.rect_transform.size = (150, 28)
        self.button_copy_values_S2.rect_transform.anchor = (0.0, 1.0)
        self.button_copy_values_S2.rect_transform.pivot = (0.0, 1.0)
        self.button_copy_values_S2_clicked = connection.add_stream(getattr,
        self.button_copy_values_S2, 'clicked')

        self.button_copy_values_landing = self.panel.add_button("Copy Current Values")
        self.button_copy_values_landing.rect_transform.position = (270, -margin_top-row_spacing*(10.0)+7)
        self.button_copy_values_landing.rect_transform.size = (150, 28)
        self.button_copy_values_landing.rect_transform.anchor = (0.0, 1.0)
        self.button_copy_values_landing.rect_transform.pivot = (0.0, 1.0)
        self.button_copy_values_landing_clicked = connection.add_stream(getattr,
        self.button_copy_values_landing, 'clicked')



    def update(self):
        self.vessel = self.connection.space_center.active_vessel

        if self.button_copy_values_ascent_clicked():
            self.button_copy_values_ascent.clicked = False
            self.config_fields['total_mass_ascent']['input'].value = str(self.vessel.mass)
            self.config_fields['vacuum_thrust_ascent']['input'].value = str(self.vessel.max_vacuum_thrust)
            self.config_fields['exhaust_velocity_sea_level_ascent']['input'].value = str(self.vessel.kerbin_sea_level_specific_impulse * 9.81)
            self.config_fields['exhaust_velocity_vacuum_ascent']['input'].value = str(self.vessel.vacuum_specific_impulse * 9.81)

        if self.button_copy_values_landing_clicked():
            self.button_copy_values_landing.clicked = False
            self.config_fields['touchdown_mass']['input'].value = str(self.vessel.mass)
            r = self.vessel.position(self.vessel.orbit.body.reference_frame)
            self.config_fields['landing_site_x']['input'].value = str(r[0])
            self.config_fields['landing_site_y']['input'].value = str(r[1])
            self.config_fields['landing_site_z']['input'].value = str(r[2])


        if self.button_copy_values_S2_clicked():
            self.button_copy_values_S2.clicked = False
            self.config_fields['total_mass_S2']['input'].value = str(self.vessel.mass)
            self.config_fields['vacuum_thrust_S2']['input'].value = str(self.vessel.max_vacuum_thrust)
            self.config_fields['exhaust_velocity_sea_level_S2']['input'].value = str(self.vessel.kerbin_sea_level_specific_impulse * 9.81)
            self.config_fields['exhaust_velocity_vacuum_S2']['input'].value = str(self.vessel.vacuum_specific_impulse * 9.81)

        if any([e['input'].changed for e in self.config_fields.values()]):
            for f in self.config_fields:
                self.config_fields[f]['input'].changed = False
            self.write_config()

    def write_config(self, **kwargs):
        filename = kwargs.get('filename', 'config.json')

        for f in self.config_fields:
            value = math.nan
            try:
                value = float(self.config_fields[f]['input'].value)
            except:pass
            self.config[f] = value

        body = self.vessel.orbit.body
        body_radius = body.equatorial_radius
        body_frame = body.reference_frame
        ut = self.connection.space_center.ut
        launch_site = self.vessel.position(body_frame)
        target_vessel = self.connection.space_center.target_vessel

        if not target_vessel is None:
            target_position = target_vessel.position(body_frame)
            target_velocity = target_vessel.velocity(body_frame)

            self.config['target_position_x'] = float(target_position[0])
            self.config['target_position_y'] = float(target_position[1])
            self.config['target_position_z'] = float(target_position[2])

            self.config['target_velocity_x'] = float(target_velocity[0])
            self.config['target_velocity_y'] = float(target_velocity[1])
            self.config['target_velocity_z'] = float(target_velocity[2])

            # For later re-identification
            self.config['target_spawn_time'] = self.connection.space_center.ut - target_vessel.met
            self.config['target_name'] = target_vessel.name

            # Find launch direction guess, orbit speed guess
            y = predict_orbit_BCBF(target_vessel, target_vessel.orbit.body.reference_frame)
            i_closest_approach = np.argmin(np.linalg.norm(y[:,0:3] - np.expand_dims(launch_site,0),axis=1))
            v_closest_approach = y[i_closest_approach,3:6]
            launch_direction_guess = v_closest_approach / np.linalg.norm(v_closest_approach)
            up_direction = launch_site / np.linalg.norm(launch_site)
            launch_direction_guess = launch_direction_guess - np.dot(up_direction,launch_direction_guess)*up_direction
            orbit_speed_guess = np.linalg.norm(v_closest_approach)
            self.config['launch_direction_guess_x'] = float(launch_direction_guess[0])
            self.config['launch_direction_guess_y'] = float(launch_direction_guess[1])
            self.config['launch_direction_guess_z'] = float(launch_direction_guess[2])
            self.config['orbit_speed_guess'] = float(orbit_speed_guess)

        self.config['t0'] = ut

        self.config['launch_site_x'] = float(launch_site[0])
        self.config['launch_site_y'] = float(launch_site[1])
        self.config['launch_site_z'] = float(launch_site[2])

        self.config['body_angular_rate'] = self.vessel.orbit.body.rotational_speed
        self.config['body_radius'] = body_radius
        self.config['gravitational_parameter'] = self.vessel.orbit.body.gravitational_parameter

        # Get scale height
        d0 = np.array(launch_site) / np.linalg.norm(launch_site)
        r0 = body_radius
        r1 = body_radius + 10000.0

        rho0 = body.atmospheric_density_at_position((d0*r0).tolist(), body_frame)
        rho1 = body.atmospheric_density_at_position((d0*r1).tolist(), body_frame)

        self.config['air_density_MSL'] = rho0
        self.config['scale_height'] = -10000.0/math.log(rho1/rho0)

        with open(filename, 'w') as f:
            json.dump(self.config, f, indent=4)


