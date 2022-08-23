from ROAR.agent_module.agent import Agent
from pathlib import Path
from ROAR.control_module.pid_controller import PIDController
from ROAR.planning_module.local_planner.simple_waypoint_following_local_planner import \
    SimpleWaypointFollowingLocalPlanner
from ROAR.planning_module.behavior_planner.behavior_planner import BehaviorPlanner
from ROAR.planning_module.mission_planner.waypoint_following_mission_planner import WaypointFollowingMissionPlanner
from ROAR.utilities_module.data_structures_models import SensorsData
from ROAR.utilities_module.vehicle_models import VehicleControl, Vehicle
import logging
import json


class RegionalPIDAgent(Agent):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.logger = logging.getLogger("Regional PID Agent")
        self.route_file_path = Path(self.agent_settings.waypoint_file_path)
        self.pid_controller = PIDController(agent=self, steering_boundary=(-1, 1), throttle_boundary=(-0.5, 1))
        self.mission_planner = WaypointFollowingMissionPlanner(agent=self) # hardcoded_waypoint_start=850
        # initiated right after mission plan
        self.behavior_planner = BehaviorPlanner(agent=self)
        self.local_planner = SimpleWaypointFollowingLocalPlanner(
            agent=self,
            controller=self.pid_controller,
            mission_planner=self.mission_planner,
            behavior_planner=self.behavior_planner,
            closeness_threshold=1)
        self.logger.debug(
            f"Waypoint Following Agent Initiated. Reading f"
            f"rom {self.route_file_path.as_posix()}")
        self.regions_config = json.load(Path(self.agent_settings.regions_file_path).open(mode='r'))
        self.pid_controller.max_speed = self.regions_config["default"]["speed"]

    def run_step(self, vehicle: Vehicle,
                 sensors_data: SensorsData) -> VehicleControl:
        super(RegionalPIDAgent, self).run_step(vehicle=vehicle,
                                               sensors_data=sensors_data)

        # Iterate through regions from file
        for i, region in enumerate(self.regions_config["regions"]):
            # If vehicle is within x and z range, set pid controller max speed to region speed
            if region["x_min"] <= vehicle.transform.location.x <= region["x_max"] \
                    and region["z_min"] <= vehicle.transform.location.z <= region["z_max"]:
                print(f"{region['description']} {self.vehicle.transform.location}")
                self.pid_controller.max_speed = region["speed"]
                break
        else:  # If vehicle is not in a region, set speed to default
            print(f"default {self.vehicle.transform.location}")
            self.pid_controller.max_speed = self.regions_config["default"]["speed"]

        self.transform_history.append(self.vehicle.transform)

        control = self.local_planner.run_in_series()
        return control
