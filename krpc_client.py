import sys
import subprocess
import time
import json
import krpc
import math
import scipy.integrate
import numpy as np
from PrePlanningChecklist import PrePlanningChecklist
from PlannerUiPanel import PlannerUiPanel
from MainUiPanel import MainUiPanel
from ConfigUiPanel import ConfigUiPanel
from AutopilotUiPanel import AutopilotUiPanel
from predict_orbit_BCBF import predict_orbit_BCBF


def main():
    conn = krpc.connect()

    ui_config = ConfigUiPanel(conn)
    get_config_callback = lambda: ui_config.config
    pre_planning_checklist = PrePlanningChecklist(conn, get_config_callback)
    get_checklist_callback = lambda: pre_planning_checklist.get_checklist()

    ui_planner = PlannerUiPanel(conn, get_checklist_callback, ui_config)
    ui_autopilot = AutopilotUiPanel(conn)
    ui_main = MainUiPanel(conn, ui_config, ui_planner, ui_autopilot)

    while True:
        ui_main.update()

        if ui_main.do_restart:
            conn.close()
            time.sleep(1.0)
            return

while True:
    try:
        main()
        #time.sleep(2.0)
    except krpc.error.RPCError:
        time.sleep(4.0)
    #except ValueError:
    #    time.sleep(4.0)
