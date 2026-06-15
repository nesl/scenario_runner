#!/usr/bin/env python3

import math
import carla
import py_trees

from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter, ActorDestroy
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.timer import GameTime

from pylot.carla_wrapper import v2v_braking_params

# =====================
# Config
# =====================
config            = v2v_braking_params.active_config
VEHICLE_NAME      = config.get("vehicle_name", "tesla.model3")   # follower (V2)
SPEED_MPS         = float(config.get("speed_mps", 35.0))         # V1 cruise speed (m/s)
T_V1_START_S      = float(config.get("t_v1_start_s", 0.0))       # start gate (sim time)
SPAWN_X           = float(config.get("spawn_x", -122.958664))
SPAWN_Y           = float(config.get("spawn_y", 100.0))
SPAWN_Z           = float(config.get("spawn_z", 0.60))
SPAWN_YAW         = float(config.get("spawn_yaw", -90.0))
V2_HEADWAY_M      = float(config.get("v2_headway_m", 30.0))
TARGET_STOP_Y     = float(config.get("target_stop_y", 10.0))     # world Y to stop at
STOP_BRAKE_HOLD_S = float(config.get("stop_hold_s", 0.5))        # hold full brake after stop

# Safety clamps (avoid zero/negative targets bricking V1)
SPEED_MPS = max(0.1, SPEED_MPS)

# =====================
# Single Behavior: Start → KeepVelocity → Stop@Y
# =====================
class StartThenCruiseUntilYStop(py_trees.behaviour.Behaviour):
    """
    V1:
      1) Hold brake until CARLA sim time >= T_V1_START_S
      2) Keep constant speed using set_target_velocity along forward vector
      3) When reaching/passing TARGET_STOP_Y (by heading), apply full brake,
         hold for STOP_BRAKE_HOLD_S, then SUCCESS.
    """
    def __init__(self, world: carla.World, v1: carla.Vehicle):
        super().__init__(name="StartThenCruiseUntilYStop")
        self.world = world
        self.v1 = v1
        self._released = False
        self._dir_sign = None   # +1 if heading toward +Y, -1 if toward -Y
        self._t_brake_start = None
        self._stopped = False

    def _arm_direction(self):
        if self._dir_sign is None:
            yaw = math.radians(self.v1.get_transform().rotation.yaw)
            fwd_y = math.sin(yaw)
            self._dir_sign = 1.0 if fwd_y >= 0.0 else -1.0

    def _at_or_past_target(self, y_now: float) -> bool:
        return (self._dir_sign < 0 and y_now <= TARGET_STOP_Y) or (self._dir_sign > 0 and y_now >= TARGET_STOP_Y)

    def update(self):
        # snap = self.world.get_snapshot()
        # if snap is None:
        #     return py_trees.common.Status.RUNNING
        # t_now = snap.timestamp.elapsed_seconds
        t_now = GameTime.get_time()
        print(t_now)
        print("out", self._released)
        self._arm_direction()

        # 1) Start gate: hold brakes until sim time >= T_V1_START_S
        if not self._released:
            if t_now < T_V1_START_S:
                self.v1.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
                return py_trees.common.Status.RUNNING
            print("one", self._released)
            self._released = True
            self.v1.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
            print(f"[Scenario] V1 gate released @ {t_now:.2f}s")

        # 2) Cruise at constant speed until reaching TARGET_STOP_Y
        y_now = self.v1.get_transform().location.y
        print(y_now)
        if not self._at_or_past_target(y_now):
            print("in two")
            fwd = self.v1.get_transform().get_forward_vector()
            self.v1.set_target_velocity(carla.Vector3D(fwd.x * SPEED_MPS, fwd.y * SPEED_MPS, 0.0))
            # ensure no unwanted brake
            # self.v1.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
            return py_trees.common.Status.RUNNING

        # 3) Stop & hold
        if not self._stopped:
            self.v1.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
            self.v1.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
            if self._t_brake_start is None:
                self._t_brake_start = t_now
            if (t_now - self._t_brake_start) >= STOP_BRAKE_HOLD_S:
                self._stopped = True
                print(f"[Scenario] V1 STOPPED at y={y_now:.2f} (target_y={TARGET_STOP_Y:.2f})")
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


# =====================
# Scenario
# =====================
class V2VBrakingScenario(BasicScenario):
    """
    Minimal V2V:
      - V1 lane-snapped; autopilot OFF; single behavior: Start→Cruise(keep velocity)→Stop@Y
      - V2 spawned behind V1 and left unmanaged (external controller handles it)
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=100):
        self.world = world
        self.timeout = timeout
        super(V2VBrakingScenario, self).__init__(
            "V2VBrakingScenario",
            ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable)
        self.v1 = None
        self.v2 = None

    def _initialize_actors(self, _config):
        # V1 spawn from config
        v1_spawn = carla.Transform(
            carla.Location(x=SPAWN_X, y=SPAWN_Y, z=SPAWN_Z),
            carla.Rotation(pitch=0.0, yaw=SPAWN_YAW, roll=0.0)
        )
        self.v1 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', v1_spawn)
        if self.v1 is None:
            # retry with a tiny nudge to avoid overlaps
            v1_spawn.location.x += 0.3
            v1_spawn.location.y += 0.3
            self.v1 = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', v1_spawn)
        assert self.v1 is not None, "Failed to spawn V1"

        # Snap V1 to driving lane center
        w = self.world.get_map().get_waypoint(
            self.v1.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        if w is not None:
            self.v1.set_transform(w.transform)
        v1_tf = self.v1.get_transform()

        # Make sure TM/autopilot is OFF so we fully control V1
        self.v1.set_autopilot(False)
        fwd0 = self.v1.get_transform().get_forward_vector()
        kick = 1.0  # m/s (small nudge)
        self.v1.set_target_velocity(carla.Vector3D(fwd0.x * kick, fwd0.y * kick, 0.0))
        self.v1.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
        print(f"[KICK] set_target_velocity ~{kick:.1f} m/s", flush=True)

        # V2 spawn at headway behind V1 (left unmanaged)
        fwd = v1_tf.get_forward_vector()
        back = carla.Location(x=-fwd.x * V2_HEADWAY_M, y=-fwd.y * V2_HEADWAY_M, z=0.0)
        v2_tf = carla.Transform(
            carla.Location(
                x=v1_tf.location.x + back.x,
                y=v1_tf.location.y + back.y,
                z=v1_tf.location.z
            ),
            v1_tf.rotation
        )
        self.v2 = CarlaDataProvider.request_new_actor(f"vehicle.{VEHICLE_NAME}", v2_tf)
        if self.v2 is None:
            v2_tf.location.x -= 0.5
            v2_tf.location.y -= 0.5
            self.v2 = CarlaDataProvider.request_new_actor(f"vehicle.{VEHICLE_NAME}", v2_tf)
        assert self.v2 is not None, "Failed to spawn V2"
        self.v2.set_autopilot(False)

        print(f"[Scenario] V1 snapped to lane at {v1_tf.location} (yaw={v1_tf.rotation.yaw:.1f})")
        print(f"[Scenario] V2 spawn headway={V2_HEADWAY_M:.1f} m behind V1 (no SR control)")

    def _create_behavior(self):
        import py_trees
        from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorTransformSetter, ActorDestroy
        from srunner.scenariomanager.timer import GameTime

        class WaitUntilTimeout(py_trees.behaviour.Behaviour):
            def __init__(self, timeout_s=100):
                super().__init__(name=f"Wait({timeout_s}s)")
                self.timeout_s = timeout_s
                self.t0 = None

            def initialise(self):
                self.t0 = GameTime.get_time()

            def update(self):
                if GameTime.get_time() - self.t0 >= self.timeout_s:
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.RUNNING

        seq = py_trees.composites.Sequence("ScenarioSequence")

        # Lock initial poses, then just wait while the client controls the cars
        seq.add_child(ActorTransformSetter(self.v1, self.v1.get_transform()))
        seq.add_child(ActorTransformSetter(self.v2, self.v2.get_transform()))
        seq.add_child(WaitUntilTimeout(timeout_s=self.timeout))  # uses scenario timeout

        # Optional: clean up V1 when done (V2 left for client)
        seq.add_child(ActorDestroy(self.v1))
        return seq


    def _create_test_criteria(self):
        return [CollisionTest(self.v1), CollisionTest(self.v2)]

    def remove_all_actors(self):
        super().remove_all_actors()
        for a in [self.v1]:
            try:
                if a is not None:
                    CarlaDataProvider.remove_actor_by_id(a.id)
            except Exception:
                pass
        # Intentionally do NOT destroy V2
