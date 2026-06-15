#!/usr/bin/env python3

import os
import math
import importlib
import importlib.util
import carla
import py_trees

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.timer import GameTime

from pylot.carla_wrapper import v2i_braking_params as params

# =====================
# Parameters (kept pythonic to match your client/server style)
# =====================
VEHICLE_NAME       = getattr(params, "vehicle_name", "audi.tt")              # ego vehicle blueprint suffix (e.g., "audi.tt")
OCCLUDER_MODEL     = getattr(params, "occluder_model", "vehicle.carlamotors.carlacola")
MAP_NAME           = getattr(params, "map_name", "Town03")

# Ego spawn (lane-snapped nearby)
EGO_SPAWN_X        = float(getattr(params, "spawn_x", -122.958664))
EGO_SPAWN_Y        = float(getattr(params, "spawn_y", 130.0))
EGO_SPAWN_Z        = float(getattr(params, "spawn_z", 0.60))
EGO_SPAWN_YAW      = float(getattr(params, "spawn_yaw", -90.0))

# Occluder placement near the crossing
OCCLUDER_X         = float(getattr(params, "occluder_x",    -122.958664))
OCCLUDER_Y         = float(getattr(params, "occluder_y",     100.0))
OCCLUDER_Z         = float(getattr(params, "occluder_z",      0.6))
OCCLUDER_YAW       = float(getattr(params, "occluder_yaw",    -90.0))

# Pedestrian start spot and crossing direction/speed/timing
PED_START_X        = float(getattr(params, "ped_start_x",   -12.0))
PED_START_Y        = float(getattr(params, "ped_start_y",    24.0))
PED_START_Z        = float(getattr(params, "ped_start_z",     0.6))
PED_START_YAW      = float(getattr(params, "ped_start_yaw",   90.0))  # yaw around Z
PED_SPEED_MPS      = float(getattr(params, "ped_speed_mps",   1.4))    # ~1.2–1.5 m/s
T_PED_START_S      = float(getattr(params, "t_ped_start_s",   4.0))    # sim-time to start crossing

# Global scenario timeout (seconds)
SCENARIO_TIMEOUT_S = float(getattr(params, "scenario_timeout_s", 600.0))

WEATHER_VISIBILITY = getattr(params, "occlusion", "none")

def apply_weather_from_params():
    world = CarlaDataProvider.get_world()
    name = getattr(params, "occlusion", "clear").lower()

    profile = getattr(params, "WEATHER_PROFILES", {}).get(name)
    if profile is None:
        raise ValueError(f"Unknown WEATHER_VISIBILITY '{name}'. "
                         f"Valid: {list(getattr(params, 'WEATHER_PROFILES', {}).keys())}")

    wp = carla.WeatherParameters(
        cloudiness=profile.get("cloudiness", 0.0),
        precipitation=profile.get("precipitation", 0.0),
        precipitation_deposits=profile.get("precipitation_deposits", 0.0),
        wind_intensity=profile.get("wind_intensity", 0.0),
        sun_azimuth_angle=profile.get("sun_azimuth_angle", 45.0),
        sun_altitude_angle=profile.get("sun_altitude_angle", 45.0),
        fog_density=profile.get("fog_density", 0.0),
        fog_distance=profile.get("fog_distance", 0.0),
        fog_falloff=profile.get("fog_falloff", 0.0),
        wetness=profile.get("wetness", 0.0),
    )
    world.set_weather(wp)

# =====================
# Pedestrian behavior: gate start by sim-time, then cross at constant velocity
# =====================
class TimedPedestrianCross(py_trees.behaviour.Behaviour):
    """
    Idle until T_PED_START_S, then cross along the pedestrian's current yaw.
    Uses controller.ai.walker when available; otherwise falls back to velocity set.
    """
    def __init__(self, world: carla.World, ped: carla.Actor, ped_ctrl: carla.Actor = None):
        super().__init__(name="TimedPedestrianCross")
        self.world = world
        self.ped = ped
        self.ctrl = ped_ctrl
        self.released = False

    def initialise(self):
        try:
            self.ped.set_simulate_physics(True)
        except Exception:
            pass
        # hold still until release
        try:
            self.ped.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        except Exception:
            pass

    def update(self):
        t_now = GameTime.get_time()
        if self.released or t_now < T_PED_START_S:
            return py_trees.common.Status.RUNNING

        # Compute a far goal along current yaw (don’t change spawn/pose)
        # yaw_deg = PED_START_YAW 
        # yaw = math.radians(yaw_deg)
        # fwd = carla.Vector3D(math.cos(yaw), math.sin(yaw), 0.0)
        # start = self.ped.get_transform().location
        # goal  = carla.Location(start.x - 30.0,
        #                        start.y,
        #                        start.z)

        print(f"[Scenario] BEFORE START: PED@{self.ped.get_transform()}")
        fwd  = carla.Vector3D(-1.0, 0.0, 0.0)
        start = self.ped.get_transform().location
        # goal  = carla.Location(start.x - 30.0, start.y, start.z)
        probe = carla.Location(x=-110.4, y=-80.79, z=start.z)

        # Snap goal to a Sidewalk waypoint near probe
        wp_goal = self.world.get_map().get_waypoint(
            probe, project_to_road=True, lane_type=carla.LaneType.Sidewalk
        )
        if wp_goal is None:
            # nudge a bit left/right to find a sidewalk across
            for off in (-1.5, 1.5, -3.0, 3.0):
                test = carla.Location(probe.x, probe.y + off, probe.z)
                wp_goal = self.world.get_map().get_waypoint(
                    test, project_to_road=True, lane_type=carla.LaneType.Sidewalk
                )
                if wp_goal: break

        goal = wp_goal.transform.location if wp_goal else probe  # fall back to probe
        print(f"[Scenario] GOAL: PED@{goal}")

        if self.ctrl is not None:
            print("Running controller")
            try:
                self.ctrl.start()
                self.ctrl.set_max_speed(PED_SPEED_MPS)   # ~1.2–1.5 m/s typical
                self.ctrl.go_to_location(goal)           # will “jaywalk” if needed
                self.released = True
                return py_trees.common.Status.RUNNING
            except Exception:
                print("Running controller failed")
                # fall through to velocity mode if controller failed
                pass

        # Fallback: straight-line walk with velocity set (older CARLA)
        print("Running manual")
        v = carla.Vector3D(fwd.x * PED_SPEED_MPS, fwd.y * PED_SPEED_MPS, 0.0)
        try:
            self.ped.set_target_velocity(v)
        except Exception:
            # Ultra-old builds: drive with WalkerControl every tick
            self.ped.apply_control(carla.WalkerControl(direction=fwd, speed=PED_SPEED_MPS))
        self.released = True
        return py_trees.common.Status.RUNNING


# =====================
# Scenario
# =====================
class V2IBrakingScenario(BasicScenario):
    """
    Minimal V2I scene setup (actors only):
      - Spawn EGO vehicle (no autopilot; left unmanaged for client control)
      - Spawn OCCLUDER truck to block ego's line-of-sight to the crossing
      - Spawn PEDESTRIAN and gate its start by sim-time for deterministic overlap

    Cameras (ego + infrastructure) are spawned and managed by the client process.
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=SCENARIO_TIMEOUT_S):
        self.world = world
        self.timeout = timeout
        super().__init__(
            "V2IBrakingScenario",
            ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable)
        self.ego = None
        self.occluder = None
        self.occluder2 = None
        self.ped = None
        self.ped_ctrl = None

    # -- Spawn actors (no camera here) --
    def _initialize_actors(self, _config):
        # Ego vehicle
        ego_spawn = carla.Transform(
            carla.Location(x=EGO_SPAWN_X, y=EGO_SPAWN_Y, z=EGO_SPAWN_Z),
            carla.Rotation(pitch=0.0, yaw=EGO_SPAWN_YAW, roll=0.0)
        )
        self.ego = CarlaDataProvider.request_new_actor(f"vehicle.{VEHICLE_NAME}", ego_spawn)
        if self.ego is None:
            # small nudge retry to avoid overlaps
            ego_spawn.location.x += 0.3
            ego_spawn.location.y += 0.3
            self.ego = CarlaDataProvider.request_new_actor(f"vehicle.{VEHICLE_NAME}", ego_spawn)
        assert self.ego is not None, "Failed to spawn ego vehicle"
        self.ego.set_autopilot(False)

        # Snap ego to center of a Driving lane (best-effort)
        w = self.world.get_map().get_waypoint(
            self.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if w is not None:
            self.ego.set_transform(w.transform)

        # Occluder: a large truck/van parked near the crossing to block ego view
        occ_loc = carla.Location(x=OCCLUDER_X, y=OCCLUDER_Y+15, z=OCCLUDER_Z)

        wp = None
        for lt in (carla.LaneType.Parking, carla.LaneType.Shoulder, carla.LaneType.Driving):
            try:
                wp = self.world.get_map().get_waypoint(
                    occ_loc, project_to_road=True, lane_type=lt
                )
            except TypeError:
                # very old CARLA builds: no lane_type kwarg
                wp = self.world.get_map().get_waypoint(occ_loc, project_to_road=True)
            if wp:
                break

        if wp is None:
            raise RuntimeError("No nearby lane for occluder; tweak OCCLUDER_X/Y.")

        occ_tf = wp.transform  # or curb_tf if you applied the lateral nudge
        occ_tf.rotation.yaw = OCCLUDER_YAW
        self.occluder = CarlaDataProvider.request_new_actor(OCCLUDER_MODEL, occ_tf)
        assert self.occluder is not None, "Failed to spawn occluder vehicle"
        try:
            self.occluder.set_autopilot(False)
        except Exception:
            pass

        # Occluder: a large truck/van parked near the crossing to block ego view
        occ2_loc = carla.Location(x=OCCLUDER_X, y=OCCLUDER_Y+15, z=OCCLUDER_Z)

        wp = None
        for lt in (carla.LaneType.Driving, carla.LaneType.Shoulder):
            try:
                wp = self.world.get_map().get_waypoint(
                    occ2_loc, project_to_road=True, lane_type=lt
                )
            except TypeError:
                # very old CARLA builds: no lane_type kwarg
                wp = self.world.get_map().get_waypoint(occ2_loc, project_to_road=True)
            if wp:
                break

        if wp is None:
            raise RuntimeError("No nearby lane for occluder; tweak OCCLUDER_X/Y.")

        occ_tf = wp.transform  # or curb_tf if you applied the lateral nudge
        occ_tf.rotation.yaw = OCCLUDER_YAW
        self.occluder2 = CarlaDataProvider.request_new_actor(OCCLUDER_MODEL, occ_tf)
        assert self.occluder2 is not None, "Failed to spawn 2nd occluder vehicle"
        try:
            self.occluder2.set_autopilot(False)
        except Exception:
            pass

        # Pedestrian: spawn at start edge, initially idle
        ped_bp = self.world.get_blueprint_library().filter("walker.pedestrian.0001")[0]
        # 1) Start from the nearest Sidewalk waypoint
        start_loc = carla.Location(x=PED_START_X, y=PED_START_Y, z=PED_START_Z)
        wp = self.world.get_map().get_waypoint(start_loc, project_to_road=True,
                                            lane_type=carla.LaneType.Sidewalk)
        if wp is None:
            raise RuntimeError("No Sidewalk waypoint near PED_START_X/Y; adjust params.")

        # 2) Build a few candidate transforms: current, a bit forward/back, and slight lateral nudges
        cands = []

        def tf_from_wp(w, dz=0.3):  # small Z lift to clear curb/mesh
            tf = w.transform
            tf.location.z += dz
            return tf

        cands.append(tf_from_wp(wp))
        # along sidewalk direction
        for d in (0.6, -0.6, 1.2, -1.2):
            nxts = wp.next(abs(d)) if d > 0 else wp.previous(abs(d))
            if nxts:
                cands.append(tf_from_wp(nxts[0]))
        # small lateral nudge away from curb/street furniture
        rv = wp.transform.get_right_vector()
        for off in (0.3, -0.3):
            t = tf_from_wp(wp)
            t.location.x += rv.x * off
            t.location.y += rv.y * off
            cands.append(t)

        # Preserve your desired crossing yaw
        for t in cands:
            t.rotation.yaw = PED_START_YAW

        # 3) Try to spawn on the first free pose
        self.ped = None
        for t in cands:
            ped = self.world.try_spawn_actor(ped_bp, t)
            if ped:
                self.ped = ped
                break
        
        self.world.tick()
        print(f"[Scenario] PED@{self.ped.get_transform()}")

        # Force your yaw after spawn (some walker spawns align to sidewalk heading)
        pose = self.ped.get_transform()
        pose.rotation = carla.Rotation(pitch=0.0, yaw=PED_START_YAW, roll=0.0)
        self.ped.set_transform(pose)
        self.world.tick()
        print(f"[Scenario] PED@{self.ped.get_transform()}")

        assert self.ped is not None, "Failed to spawn pedestrian after multiple sidewalk candidates"

        # Attach AI walker controller (no change to spawn pose)
        self.ped_ctrl = None
        try:
            ctrl_bp = self.world.get_blueprint_library().find("controller.ai.walker")
            self.ped_ctrl = self.world.try_spawn_actor(ctrl_bp, carla.Transform(), self.ped)
        except Exception:
            self.ped_ctrl = None

        apply_weather_from_params()

        # w = carla.WeatherParameters(
        #     fog_density=95.0, fog_distance=18.0, fog_falloff=3.0,
        #     cloudiness=0.0, precipitation=0.0, sun_altitude_angle=10.0)
        # self.world.set_weather(w)
        print(self.world.get_weather())

        print(f"[Scenario] EGO@{self.ego.get_transform().location}  "
              f"Occluder='{OCCLUDER_MODEL}'@{self.occluder.get_transform().location}  "
              f"PED@{self.ped.get_transform().location}")

    # -- Behavior: gate pedestrian start; hold until scenario timeout; clean up non-ego actors --
    def _create_behavior(self):
        class WaitUntilTimeout(py_trees.behaviour.Behaviour):
            def __init__(self, timeout_s=float(self.timeout)):
                super().__init__(name=f"Wait({timeout_s:.0f}s)")
                self.timeout_s = timeout_s
                self.t0 = None
            def initialise(self):
                self.t0 = GameTime.get_time()
            def update(self):
                if GameTime.get_time() - self.t0 >= self.timeout_s:
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING

        seq = py_trees.composites.Sequence("V2ISequence")
        # Pedestrian timing gate (walks across after T_PED_START_S)
        # seq.add_child(TimedPedestrianCross(self.world, self.ped, self.ped_ctrl))
        # Keep the world running for client/server processes
        seq.add_child(WaitUntilTimeout(timeout_s=self.timeout))
        # Destroy occluder + pedestrian; keep ego for client
        seq.add_child(ActorDestroy(self.occluder))
        seq.add_child(ActorDestroy(self.ped))
        return seq

    def _create_test_criteria(self):
        # Optional: keep standard collision checks
        return [CollisionTest(self.ego)]

    def remove_all_actors(self):
        super().remove_all_actors()
        # Keep ego alive for the client; best-effort cleanup of the others is handled in the tree.
        pass
