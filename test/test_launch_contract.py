"""Test the public launch interface of ground_vehicle_twist_odometry."""

import importlib.util
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest
import ros2_launch_helpers as rlh

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _load_launch_module() -> ModuleType:
    path = PACKAGE_ROOT / 'launch' / 'ground_vehicle_twist_odometry.launch.py'
    spec = importlib.util.spec_from_file_location('ground_vehicle_twist_odometry_launch', path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_exposes_file_configuration_and_clock_selection() -> None:
    """Expose the parameter file and use_sim_time without individual functional overrides."""
    module = _load_launch_module()
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }

    assert set(declarations) == {
        'namespace',
        'params_file',
        'params_file_allow_substs',
        'use_sim_time',
        'node_args',
    }

    context = LaunchContext()
    declarations['node_args'].visit(context)
    assert context.launch_configurations['node_args'] == (
        '{"output":"both","ros_arguments":["--log-level","info"]}'
    )


@pytest.mark.parametrize(('allow_substs', 'use_sim_time'), [('True', 'False'), ('False', 'True')])
def test_launch_passes_parameter_file_and_clock_selection_to_the_node(
    allow_substs: str, use_sim_time: str, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    """Keep use_sim_time as the only node parameter set outside the YAML file."""
    module = _load_launch_module()
    params_file = tmp_path / 'params.yaml'
    params_file.write_text('/**:\n  ros__parameters:\n    use_sim_time: false\n', encoding='utf-8')
    captured: dict[str, object] = {}

    class FakeParameterFile:
        def __init__(self, path: str, *, allow_substs: bool) -> None:
            self.path = path
            self.allow_substs = allow_substs

    class FakeNode:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    class FakeParameterValue:
        def __init__(self, value: object, *, value_type: type[bool]) -> None:
            self.value = value
            self.value_type = value_type

    monkeypatch.setattr(module, 'ParameterFile', FakeParameterFile)
    monkeypatch.setattr(module, 'ParameterValue', FakeParameterValue)
    monkeypatch.setattr(module, 'Node', FakeNode)

    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': 'robot',
            'params_file': str(params_file),
            'params_file_allow_substs': allow_substs,
            'use_sim_time': use_sim_time,
            'node_args': '{}',
        }
    )

    actions = module.launch_ground_vehicle_twist_odometry_node(context)

    assert len(actions) == 1
    assert len(captured['parameters']) == 2
    parameter_file = captured['parameters'][0]
    assert parameter_file.path.perform(context) == str(params_file)
    assert parameter_file.allow_substs is (allow_substs == 'True')

    use_sim_time_override = captured['parameters'][1]
    assert set(use_sim_time_override) == {'use_sim_time'}
    parameter_value = use_sim_time_override['use_sim_time']
    assert parameter_value.value.perform(context) == use_sim_time
    assert parameter_value.value_type is bool


@pytest.mark.parametrize(('allow_substs', 'use_sim_time'), [('True', 'False'), ('False', 'True')])
def test_launch_constructs_a_real_node_with_installed_helpers(
    allow_substs: str, use_sim_time: str, tmp_path: Path
) -> None:
    """Resolve the selected helper APIs and construct the actual launch Node action."""
    module = _load_launch_module()
    params_file = tmp_path / 'params.yaml'
    params_file.write_text('/**:\n  ros__parameters:\n    publish_tf: true\n', encoding='utf-8')

    context = LaunchContext()
    context.launch_configurations.update(
        {
            'namespace': 'robot',
            'params_file': str(params_file),
            'params_file_allow_substs': allow_substs,
            'use_sim_time': use_sim_time,
            'node_args': '{"name":"custom_odom","remappings":[["twist","velocity_echo"]]}',
        }
    )

    rlh.RequireFile(path=params_file).execute(context)
    actions = module.launch_ground_vehicle_twist_odometry_node(context)
    assert len(actions) == 1
    assert isinstance(actions[0], Node)

    resolved_arguments = rlh.resolve_node_arguments(
        context.launch_configurations['node_args'],
        default_arguments={'name': 'ground_vehicle_twist_odometry'},
        extra_rejected_arguments={'namespace'},
    )
    assert resolved_arguments['name'] == 'custom_odom'
    assert resolved_arguments['remappings'] == [('twist', 'velocity_echo')]

    with pytest.raises(ValueError, match='namespace'):
        rlh.resolve_node_arguments('{"namespace":"other"}', extra_rejected_arguments={'namespace'})

    params_file.unlink()
    with pytest.raises(FileNotFoundError):
        rlh.RequireFile(path=params_file).execute(context)
