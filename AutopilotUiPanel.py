from krpc_utils import TelemetryStreams
import time
import math
import numpy as np
import json
from MainUiPanel import panel_x_offset
from krpc_utils import quaternion_rotation
from Autopilot import Autopilot

class AutopilotUiPanel:
    def __init__(self, connection):
        self.autopilot = None
        self.canvas = connection.ui.stock_canvas
        self.connection = connection
        self.screen_size = self.canvas.rect_transform.size
        self.ut_previous_control_update = 0.0
        self.ut_previous_status_update = 0.0
        self.ut_stream = connection.add_stream(getattr, connection.space_center, 'ut')

        self.panel = self.canvas.add_panel()
        self.panel.rect_transform.size = (600, 300)
        self.panel.rect_transform.position = (panel_x_offset, -200)
        self.panel.rect_transform.anchor = (0.0, 1.0)
        self.panel.rect_transform.pivot = (0.0, 1.0)
        self.panel.visible = False

        self.label_status = self.panel.add_text("")
        self.label_status.rect_transform.position = (25, -60)
        self.label_status.rect_transform.size = (500, 300)
        self.label_status.rect_transform.anchor = (0.0, 1.0)
        self.label_status.rect_transform.pivot = (0.0, 1.0)
        self.label_status.color = (1,1,1)
        self.label_status.alignment = connection.ui.TextAnchor.upper_left
        self.label_status.size = 15
        self.label_status.font = 'Courier New'
        self.label_status.style = connection.ui.FontStyle.bold


        self.button_start = self.panel.add_button("Start")
        self.button_start.rect_transform.position = (20, -20)
        self.button_start.rect_transform.size = (70, 30)
        self.button_start.rect_transform.anchor = (0.0, 1.0)
        self.button_start.rect_transform.pivot = (0.0, 1.0)
        self.button_start_clicked = connection.add_stream(getattr,
        self.button_start, 'clicked')

        self.button_disable = self.panel.add_button("Disable")
        self.button_disable.rect_transform.position = (100, -20)
        self.button_disable.rect_transform.size = (70, 30)
        self.button_disable.rect_transform.anchor = (0.0, 1.0)
        self.button_disable.rect_transform.pivot = (0.0, 1.0)
        self.button_disable_clicked = connection.add_stream(getattr,
        self.button_disable, 'clicked')

        self.restore_state()


    def save_state(self):
        with open('autopilot_state.json', 'w') as f:
            json.dump({'is_running': self.is_running()}, f)

    def restore_state(self):
        is_running = False
        try:
            with open('autopilot_state.json', 'r') as f:
                data = json.load(f)
            if data.get('is_running', False) == True:
                is_running = True
        except:
            pass

        if is_running:
            self.autopilot = Autopilot(self.connection, self.ut_stream)


    def is_running(self):
        return self.autopilot is not None

    def update(self):

        if self.button_start_clicked():
            self.button_start.clicked = False
            if not self.is_running():
                self.autopilot = Autopilot(self.connection, self.ut_stream)
                self.save_state()

        if self.button_disable_clicked():
            self.button_disable.clicked = False
            if self.is_running():
                self.autopilot = None
                self.save_state()

        # Timed updates
        if self.is_running():

            ut = self.ut_stream()
            if ut > self.ut_previous_control_update + 1.0/20:
                delta_t = ut - self.ut_previous_control_update
                self.autopilot.control_update(ut, delta_t)
                self.ut_previous_control_update = ut
            if ut > self.ut_previous_status_update + 0.5:
                delta_t = ut - self.ut_previous_status_update
                self.label_status.content = self.autopilot.get_status_update(ut, delta_t)
                self.ut_previous_status_update = ut

            if self.autopilot.should_disable:
                self.autopilot = None
                self.save_state()
                exit()

