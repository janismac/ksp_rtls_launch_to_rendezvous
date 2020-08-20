import time
panel_x_offset = 160

class MainUiPanel:
    def __init__(self, connection, ui_config, ui_planner, ui_autopilot):
        self.do_restart = False
        self.initial_vessel = connection.space_center.active_vessel
        self.active_vessel_stream = connection.add_stream(getattr, connection.space_center, 'active_vessel')
        self.canvas = connection.ui.stock_canvas
        self.connection = connection
        self.ui_config = ui_config
        self.ui_planner = ui_planner
        self.ui_autopilot = ui_autopilot

        self.panel = self.canvas.add_panel()
        self.panel.rect_transform.size = (350, 90)
        self.panel.rect_transform.position = (panel_x_offset, -100)
        self.panel.rect_transform.anchor = (0.0, 1.0)
        self.panel.rect_transform.pivot = (0.0, 1.0)

        self.label_header = self.panel.add_text("RTLS Launch To Rendezvous Tool")
        self.label_header.rect_transform.position = (0, 18)
        self.label_header.rect_transform.size = (400, 20)
        self.label_header.color = (1,1,1)
        self.label_header.alignment = connection.ui.TextAnchor.middle_center
        self.label_header.size = 15

        self.button_toggle_config = self.panel.add_button("Config")
        self.button_toggle_config.rect_transform.position = (-110, -16)
        self.button_toggle_config.rect_transform.size = (100, 30)
        self.button_toggle_config_clicked = connection.add_stream(getattr, self.button_toggle_config, 'clicked')

        self.button_toggle_planner = self.panel.add_button("Planner")
        self.button_toggle_planner.rect_transform.position = (0, -16)
        self.button_toggle_planner.rect_transform.size = (100, 30)
        self.button_toggle_planner_clicked = connection.add_stream(getattr, self.button_toggle_planner, 'clicked')

        self.button_toggle_autopilot = self.panel.add_button("Autopilot")
        self.button_toggle_autopilot.rect_transform.position = (110, -16)
        self.button_toggle_autopilot.rect_transform.size = (100, 30)
        self.button_toggle_autopilot_clicked = connection.add_stream(getattr, self.button_toggle_autopilot, 'clicked')

        self.button_left = self.panel.add_button("<<")
        self.button_left.rect_transform.position = (-140, 18)
        self.button_left.rect_transform.size = (40, 25)
        self.button_left_clicked = connection.add_stream(getattr, self.button_left, 'clicked')

        self.button_right = self.panel.add_button(">>")
        self.button_right.rect_transform.position = (140, 18)
        self.button_right.rect_transform.size = (40, 25)
        self.button_right_clicked = connection.add_stream(getattr, self.button_right, 'clicked')

        if self.ui_autopilot.is_running():
            self.ui_config.panel.visible = False
            self.ui_planner.panel.visible = False
            self.ui_autopilot.panel.visible = True


    def update(self):
        if not (self.initial_vessel == self.active_vessel_stream()):
            self.do_restart = True
            return

        if self.ui_autopilot.autopilot is not None and self.ui_autopilot.autopilot.should_reboot:
            self.do_restart = True
            return

        self.ui_autopilot.update()

        slow_mode = not self.ui_autopilot.is_running()

        if slow_mode:
            self.ui_config.update()
            self.ui_planner.update()

            if self.button_toggle_config_clicked():
                self.button_toggle_config.clicked = False
                self.ui_config.panel.visible = not self.ui_config.panel.visible
                self.ui_planner.panel.visible = False
                self.ui_autopilot.panel.visible = False

            if self.button_toggle_planner_clicked():
                self.button_toggle_planner.clicked = False
                self.ui_config.panel.visible = False
                self.ui_planner.panel.visible = not self.ui_planner.panel.visible
                self.ui_autopilot.panel.visible = False

        if self.button_toggle_autopilot_clicked():
            self.button_toggle_autopilot.clicked = False
            self.ui_config.panel.visible = False
            self.ui_planner.panel.visible = False
            self.ui_autopilot.panel.visible = not self.ui_autopilot.panel.visible

        if self.button_left_clicked():
            self.button_left.clicked = False
            transforms = [self.panel.rect_transform, self.ui_config.panel.rect_transform, self.ui_planner.panel.rect_transform, self.ui_autopilot.panel.rect_transform]
            for t in transforms:
                pos = t.position
                t.position = (pos[0]-50,pos[1])

        if self.button_right_clicked():
            self.button_right.clicked = False
            transforms = [self.panel.rect_transform, self.ui_config.panel.rect_transform, self.ui_planner.panel.rect_transform, self.ui_autopilot.panel.rect_transform]
            for t in transforms:
                pos = t.position
                t.position = (pos[0]+50,pos[1])

        if slow_mode:
            time.sleep(0.2)
        else:
            time.sleep(0.001)

