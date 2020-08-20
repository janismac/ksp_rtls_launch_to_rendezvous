import math
import numpy as np
from predict_orbit_BCBF import predict_orbit_BCBF

class PrePlanningChecklist:
    def __init__(self, connection, get_config_callback):
        self.connection = connection
        self.get_config_callback = get_config_callback
        self.vessel = connection.space_center.active_vessel

    def get_checklist(self):
        left_pad = lambda s: ((' '*100)+s)[-53:]
        pad_dots = lambda a,b: a+' '+('.'*(49-len(a)-len(b)))+' '+b+'  '
        all_ok = True
        situation = self.vessel.situation
        body_name = self.vessel.orbit.body.name
        text = "Pre-Planning Checklist\n\n"

        text += pad_dots('Celestial Body','Kerbin')
        if body_name == 'Kerbin':
            text += "✓\n\n"
        else:
            text += "✗\n"
            text += left_pad("(actual: " + body_name + ")\n")
            all_ok = False

        text += pad_dots('Situation','landed or pre_launch')
        if situation.name == 'landed' or situation.name == 'pre_launch':
            text += "✓\n\n"
        else:
            text += "✗\n"
            text += left_pad("(actual: " + situation.name + ")\n")
            all_ok = False

        target_vessel = self.connection.space_center.target_vessel
        target_vessel_str = 'None' if target_vessel is None else target_vessel.name
        text += pad_dots('Target Vessel', target_vessel_str)
        if not target_vessel is None:
            text += "✓\n\n"
        else:
            text += "✗\n\n"
            all_ok = False


        target_SOI_str = 'None' if target_vessel is None else target_vessel.orbit.body.name
        text += pad_dots('Target SOI', 'Kerbin')
        if target_SOI_str == 'Kerbin':
            text += "✓\n\n"
        else:
            text += "✗\n"
            text += left_pad("(actual: " + target_SOI_str + ")\n")
            all_ok = False

        launch_window_ok = False

        if all_ok: # Run launch window check
            r_self_BCBF = np.expand_dims(self.vessel.position(target_vessel.orbit.body.reference_frame),0)
            target_trajectory = predict_orbit_BCBF(target_vessel, target_vessel.orbit.body.reference_frame)
            delta_r = target_trajectory[:,0:3] - r_self_BCBF
            is_overhead = np.sum(delta_r * r_self_BCBF,axis=1) > 0
            is_close = np.linalg.norm(delta_r, axis=1) < 300000.0
            launch_window_ok = True == np.any(np.logical_and(is_overhead, is_close)).tolist()

        text += pad_dots('Target passing overhead in 10 to 20 minutes?','')
        if launch_window_ok:
            text += "✓\n\n"
        else:
            text += "✗\n\n"
            all_ok = False

        config = self.get_config_callback()
        expected_liftoff_mass = config.get('total_mass_ascent', -1e9)
        actual_liftoff_mass = self.vessel.mass

        text += pad_dots('Liftoff Mass','as configured')
        if math.fabs(actual_liftoff_mass/(expected_liftoff_mass+1e-15)-1) < 0.01:
            text += "✓\n\n"
        else:
            text += "✗\n"
            text += left_pad("(expected: " + str(expected_liftoff_mass) + "; actual: " + str(actual_liftoff_mass) + ")\n")
            all_ok = False

        return (text, all_ok)
