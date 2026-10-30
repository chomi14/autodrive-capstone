"""Mode-specific settings, shared parameter GUI and W/S heartbeat protocol."""
import rclpy
from rclpy.executors import ExternalShutdownException
from .track_tuner_node import TrackTunerNode
from .mode_parameters import MISSION, PERPENDICULAR, PARALLEL, LEGACY_OBSTACLE_PARAMETERS, trackbars


def run(mode, args=None):
    rclpy.init(args=args)
    node = None
    try:
        parking = mode != 'mission'
        specs = {'mission': MISSION, 'perpendicular': PERPENDICULAR, 'parallel': PARALLEL}[mode]
        if mode == 'mission':
            specs = {name: spec for name, spec in specs.items() if name not in LEGACY_OBSTACLE_PARAMETERS}
        node = TrackTunerNode(
            node_name=f'{mode}_tuner_node', extra_trackbars=trackbars(specs),
            parameter_root=f'{mode}_parking_controller_node' if parking else 'mission_controller_node',
            window_prefix=mode.capitalize(), parking=parking,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def mission_main(args=None):
    run('mission', args)

def perpendicular_main(args=None):
    run('perpendicular', args)

def parallel_main(args=None):
    run('parallel', args)
