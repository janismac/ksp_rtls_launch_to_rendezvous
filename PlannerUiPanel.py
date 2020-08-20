import sys
import subprocess
import time
from MainUiPanel import panel_x_offset

class PlannerUiPanel:
    def __init__(self, connection, get_checklist_callback, ui_config):
        self.canvas = connection.ui.stock_canvas
        self.connection = connection
        self.ui_config = ui_config
        self.get_checklist_callback = get_checklist_callback
        self.screen_size = self.canvas.rect_transform.size

        self.panel = self.canvas.add_panel()
        self.panel.rect_transform.size = (550, 500)
        self.panel.rect_transform.position = (panel_x_offset, -200)
        self.panel.rect_transform.anchor = (0.0, 1.0)
        self.panel.rect_transform.pivot = (0.0, 1.0)
        self.panel.visible = True

        self.label_checklist = self.panel.add_text("")
        self.label_checklist.rect_transform.position = (20, -20)
        self.label_checklist.rect_transform.size = (500, 450)
        self.label_checklist.rect_transform.anchor = (0.0, 1.0)
        self.label_checklist.rect_transform.pivot = (0.0, 1.0)
        self.label_checklist.color = (1,1,1)
        self.label_checklist.alignment = connection.ui.TextAnchor.upper_left
        self.label_checklist.size = 15
        self.label_checklist.font = 'Courier New'
        self.label_checklist.style = connection.ui.FontStyle.bold

        self.button_start_optimization = self.panel.add_button("Start Flight Optimization")
        self.button_start_optimization.rect_transform.position = (20, -453)
        self.button_start_optimization.rect_transform.size = (180, 30)
        self.button_start_optimization.rect_transform.anchor = (0.0, 1.0)
        self.button_start_optimization.rect_transform.pivot = (0.0, 1.0)
        self.button_start_optimization_clicked = connection.add_stream(getattr,
        self.button_start_optimization, 'clicked')

        self.button_reset = self.panel.add_button("Reset")
        self.button_reset.rect_transform.position = (210, -453)
        self.button_reset.rect_transform.size = (70, 30)
        self.button_reset.rect_transform.anchor = (0.0, 1.0)
        self.button_reset.rect_transform.pivot = (0.0, 1.0)
        self.button_reset_clicked = connection.add_stream(getattr,
        self.button_reset, 'clicked')

        self.optimizer_process = None

    def update(self):
        if self.optimizer_process is None:
            (checklist_text, all_ok) = self.get_checklist_callback()
            self.label_checklist.content = checklist_text
        else:
            with open("stdout.txt","r") as f:
                out_txt = f.read()
            lines = out_txt.split("\n")
            lines = lines[-40:]
            out_txt = "\n".join(lines)
            self.label_checklist.content = out_txt
            self.label_checklist.size = 8
            return_code = self.optimizer_process.poll()
            if return_code == 42:
                self.connection.ui.message('Optimization successful! Switch to Autopilot to start the mission.', duration=8.0)
                self.label_checklist.size = 15
                self.label_checklist.content = ''
                self.optimizer_process = None

        if self.button_start_optimization_clicked():
            self.button_start_optimization.clicked = False
            if self.optimizer_process is None:
                if all_ok:
                    self.ui_config.write_config()
                    self.ui_config.write_config(filename='mission.json')
                    time.sleep(0.5)

                    with open("stdout.txt","wb") as out:
                        self.optimizer_process = subprocess.Popen((sys.executable, "optimizer.py"), stdout=out, stderr=out)

                else:
                    self.connection.ui.message('Error: Checklist not complete.', duration=3.0)
            else:
                self.connection.ui.message('Optimization is running.', duration=3.0)

        if self.button_reset_clicked():
            self.button_reset.clicked = False
            self.label_checklist.size = 15
            self.label_checklist.content = ''
            if not self.optimizer_process is None:
                self.optimizer_process.terminate()
                self.optimizer_process = None

