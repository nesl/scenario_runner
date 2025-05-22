#!/usr/bin/env python

import carla
import time
import py_trees
import random

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ChangeAutoPilot, ActorTransformSetter, KeepVelocity, ActorDestroy, StopVehicle, WaypointFollower
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenariomanager.timer import TimeOut
from srunner.tools.scenario_helper import get_waypoint_in_distance

class OpenLoopScenario(BasicScenario):
    """
    Scenario class for a high-speed vehicle moving in a straight line.
    :param world: CARLA world object
    :param ego_vehicles: List of ego vehicles (not used here since there's no ego vehicle)
    :param config: Scenario configuration (ScenarioConfiguration)
    :param randomize: Select random parameters (optional, default=False)
    :param debug_mode: Enable debug mode for detailed output (optional, default=False)
    :param criteria_enable: Enable evaluation criteria (optional, default=True)
    :param timeout: Overall scenario timeout in seconds (default=1000)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=100):
        self._map = CarlaDataProvider.get_map()
        self.vehicle_speed = 10.0
        self.timeout = timeout

        for actor in world.get_actors():
            #if actor.type_id.startswith("vehicle."):
            actor.destroy()

        # Call constructor of BasicScenario
        super(OpenLoopScenario, self).__init__(
            "OpenLoopScenario",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

        self.vehicle = None
        #self._initialize_actors(config)

    def _initialize_actors(self, config):
        """
        Initialize the leading vehicle with a randomized or predefined spawn point.
        """
        # Get a list of spawn points and select one (e.g., random or specific index)
        spawn_points = self._map.get_spawn_points()
        for i, spawn_point in enumerate(spawn_points):
            print(f"Spawn Point {i}: Location = {spawn_point.location}, Rotation = {spawn_point.rotation}")
        #spawn_point = random.choice(spawn_points)  # or use spawn_points[5] for a specific index
        spawn_location = carla.Transform(
            carla.Location(x=9.530025, y=302.570007, z=0.500000),
            carla.Rotation(pitch=0.000000, yaw=0, roll=0.000000))
        # Optionally adjust the Z coordinate if necessary
        #spawn_location.location.z += 1.0

        # Try spawning the vehicle with a different model if necessary
        try:
            self.vehicle = CarlaDataProvider.request_new_actor(model = 'vehicle.audi.a2', spawn_point=spawn_location)
        except RuntimeError as e:
            print(f"Failed to spawn 'vehicle.nissan.patrol' at {spawn_point}. Error: {e}")
            print("Trying with a different vehicle model...")
            self.vehicle = CarlaDataProvider.request_new_actor('vehicle.audi.a2', spawn_location)
        
        self.vehicle.set_simulate_physics(enabled=True)
        #self.vehicle.set_autopilot(enabled=True)
        self.vehicle.set_target_velocity(carla.Vector3D(self.vehicle_speed, 0, 0))
        self.other_actors.append(self.vehicle)


    def _create_behavior(self):
        """
        Define the behavior of the high-speed vehicle.
        """
        # Sequence of behaviors for the vehicle
        behavior = py_trees.composites.Sequence("VehicleBehaviorSequence")

        # Set the vehicle to the initial transform and apply the speed
        start_transform = ActorTransformSetter(self.vehicle, self.vehicle.get_transform())
        maintain_speed = KeepVelocity(self.vehicle, self.vehicle_speed)
        #custom_waypoint_follower = CustomWaypointFollower(self.vehicle, target_speed=30.0, target_distance=500)
        #waypoint_follower = WaypointFollower(self.vehicle, self.vehicle_speed, avoid_collision=True)
        drive_distance_condition = DriveDistance(self.vehicle, distance=50, name="DriveDistance")
        stop_behavior = ActorDestroy(self.vehicle)

        # Build the behavior sequence
        behavior.add_child(start_transform)
        print("start transform")
        #behavior.add_child(custom_waypoint_follower)
        behavior.add_child(drive_distance_condition)
        behavior.add_child(stop_behavior)
        print("stop")
        
        return behavior

    def _create_test_criteria(self):
        """
        Setup evaluation criteria (optional, could be expanded).
        """
        criteria = []

        # Example criterion: Ensure the vehicle does not collide
        collision_criterion = CollisionTest(self.vehicle)
        criteria.append(collision_criterion)

        return criteria

    def remove_all_actors(self):
        """
        Remove all actors when the scenario ends.
        """
        super().remove_all_actors()
        if self.vehicle is not None:
            CarlaDataProvider.remove_actor_by_id(self.vehicle.id)


# Define a custom waypoint-following behavior
class CustomWaypointFollower(py_trees.behaviour.Behaviour):
    def __init__(self, vehicle, target_speed, target_distance, name="CustomWaypointFollower"):
        super(CustomWaypointFollower, self).__init__(name)
        self.vehicle = vehicle
        self.target_speed = target_speed
        self.target_distance = target_distance
        self.current_distance = 0
        self.world = vehicle.get_world()
        self.map = self.world.get_map()
        self.start_location = None

    def initialise(self):
        # Record the initial position to calculate travel distance
        self.start_location = self.vehicle.get_location()

    def update(self):
        # Calculate the distance traveled
        current_location = self.vehicle.get_location()
        self.current_distance = current_location.distance(self.start_location)
        
        if self.current_distance >= self.target_distance:
            return py_trees.common.Status.SUCCESS  # Stop after reaching target distance

        # Get the current and next waypoints
        waypoint = self.map.get_waypoint(current_location)
        next_waypoint = waypoint.next(10.0)[0]  # 2 meters ahead

        # Calculate the direction to the next waypoint
        vehicle_transform = self.vehicle.get_transform()
        forward_vector = vehicle_transform.get_forward_vector()
        direction_to_waypoint = next_waypoint.transform.location - vehicle_transform.location

        # Calculate steering based on the lateral error (simplified proportional control)
        lateral_error = direction_to_waypoint.y * forward_vector.x - direction_to_waypoint.x * forward_vector.y
        steer = max(-1.0, min(1.0, 0.05 * lateral_error))  # Adjust 0.05 based on turn sharpness

        # Apply throttle and steering
        control = carla.VehicleControl()
        control.throttle = min(self.target_speed / max(1, self.vehicle.get_velocity().x), 0.5)
        control.steer = steer
        self.vehicle.apply_control(control)

        return py_trees.common.Status.RUNNING
