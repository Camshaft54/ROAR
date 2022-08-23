import datetime

from ROAR.agent_module.agent import Agent
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import Vehicle, VehicleControl
from ROAR.configurations.configuration import Configuration as AgentConfig
from pathlib import Path

# TODO: Instead of recording every step, record a step when it exceeds a certain distance from the last step
#  (helps with waypoint lookahead if I'm moving slowly around a turn)
#  Also modify carla_client/util/keyboard_control.py to have a redo shortcut that pauses the waypoint recording,
#  deletes the last n waypoints and teleports the car to the newest waypoint not deleted


class WaypointGeneratingAgent(Agent):
    def __init__(self, vehicle: Vehicle, agent_settings: AgentConfig, **kwargs):
        super().__init__(vehicle=vehicle, agent_settings=agent_settings, **kwargs)
        self.output_file_path: Path = self.output_folder_path / f"major_map_waypoints_{datetime.datetime.now().strftime('%Y-%m-%dT%H-%M-%S')} .txt"
        if self.output_folder_path.exists() is False:
            self.output_folder_path.mkdir(exist_ok=True, parents=True)
        self.output_file = self.output_file_path.open('w')
        self.time_counter = 0

    def run_step(self, sensors_data: SensorsData,
                 vehicle: Vehicle) -> VehicleControl:
        super(WaypointGeneratingAgent, self).run_step(sensors_data=sensors_data,
                                                      vehicle=vehicle)
        if self.time_counter > 1:
            print(f"Writing to [{self.output_file_path}]: {self.vehicle.transform}")
            self.output_file.write(self.vehicle.transform.record() + "\n")
        self.time_counter += 1
        return VehicleControl()
