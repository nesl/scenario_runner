#!/usr/bin/env python

import carla
import py_trees
import random

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorTransformSetter, KeepVelocity, ActorDestroy
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
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
                 timeout=1000):
        self._map = CarlaDataProvider.get_map()
        self.vehicle_speed = 40.0
        self.timeout = timeout

        for actor in world.get_actors():
            if actor.type_id.startswith("vehicle."):
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
        self._initialize_actors(config)

    def _print_spawn_points(self, world):
        #Print all spawn points in the CARLA world.
        spawn_points = self._map.get_spawn_points()
        for i, spawn_point in enumerate(spawn_points):
            print(f"Spawn Point {i}: Location = {spawn_point.location}, Rotation = {spawn_point.rotation}")
    '''
    def _initialize_actors(self, config):
        """
        Initialize the high-speed vehicle and print all spawn points.
        """
        # Print all available spawn points
        print("Available Spawn Points:")
        self._print_spawn_points(self._map)
    '''
    def _initialize_actors(self, config):
        """
        Initialize the leading vehicle with a randomized or predefined spawn point.
        """
        # Get a list of spawn points and select one (e.g., random or specific index)
        spawn_points = self._map.get_spawn_points()
        spawn_point = random.choice(spawn_points)  # or use spawn_points[5] for a specific index

        # Optionally adjust the Z coordinate if necessary
        spawn_point.location.z += 1.0

        # Try spawning the vehicle with a different model if necessary
        try:
            self.vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', spawn_point)
        except RuntimeError as e:
            print(f"Failed to spawn 'vehicle.nissan.patrol' at {spawn_point}. Error: {e}")
            print("Trying with a different vehicle model...")
            self.vehicle = CarlaDataProvider.request_new_actor('vehicle.audi.a2', spawn_point)

        self.other_actors.append(self.vehicle)
        '''
        # Get the spawn point from the config or choose a default
        spawn_point = carla.Transform(
            carla.Location(x=111.080048, y=302.570007, z=1),
            carla.Rotation(pitch=0.000000, yaw=-179.999634, roll=0.000000))
        #config.trigger_points[0] if config.trigger_points else carla.Transform()

        # Spawn the high-speed vehicle at the chosen spawn point
        self.vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', spawn_point)
        
        # Set the vehicle to the desired speed in a straight line
        self.vehicle.set_target_velocity(carla.Vector3D(self.vehicle_speed, 0, 0))

        # Add the vehicle to other actors to manage its lifecycle
        self.other_actors.append(self.vehicle)
        '''

    def _create_behavior(self):
        """
        Define the behavior of the high-speed vehicle.
        """
        # Sequence of behaviors for the vehicle
        behavior = py_trees.composites.Sequence("VehicleBehaviorSequence")

        # Set the vehicle to the initial transform and apply the speed
        start_transform = ActorTransformSetter(self.vehicle, self.vehicle.get_transform())
        maintain_speed = KeepVelocity(self.vehicle, self.vehicle_speed)
        stop_behavior = ActorDestroy(self.vehicle)

        # Build the behavior sequence
        behavior.add_child(start_transform)
        behavior.add_child(maintain_speed)
        behavior.add_child(stop_behavior)
        
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
