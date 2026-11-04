"""Checks the launch files against the nodes and the example parameter file.
The launch packages are replaced by small stand-ins, so no ROS installation is needed."""
import importlib.util
import pathlib
import re
import sys
import types

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent

# launch file, node name, node module
LAUNCHES = [
    ("joy_teleop.launch.py", "joy_teleop", "joy_node"),
    ("web_teleop.launch.py", "web_teleop", "web_node"),
]

# Parameters that bound what the robot can do, by node.
SAFETY_PARAMS = {
    "joy_teleop": ["deadman_button", "max_linear", "max_angular"],
    "web_teleop": ["watchdog_s", "max_linear", "max_angular"],
}


class Recorded:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class LaunchDescription(Recorded):
    pass


class DeclareLaunchArgument(Recorded):
    pass


class LaunchConfiguration(Recorded):
    pass


class Node(Recorded):
    pass


@pytest.fixture
def fake_launch(monkeypatch):
    launch = types.ModuleType("launch")
    launch.LaunchDescription = LaunchDescription
    actions = types.ModuleType("launch.actions")
    actions.DeclareLaunchArgument = DeclareLaunchArgument
    substitutions = types.ModuleType("launch.substitutions")
    substitutions.LaunchConfiguration = LaunchConfiguration
    launch_ros = types.ModuleType("launch_ros")
    launch_ros_actions = types.ModuleType("launch_ros.actions")
    launch_ros_actions.Node = Node
    for name, mod in [("launch", launch), ("launch.actions", actions), ("launch.substitutions", substitutions),
                      ("launch_ros", launch_ros), ("launch_ros.actions", launch_ros_actions)]:
        monkeypatch.setitem(sys.modules, name, mod)


def load_launch(filename):
    path = ROOT / "launch" / filename
    spec = importlib.util.spec_from_file_location(filename.replace(".", "_"), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


def declared_params(module):
    # Every node parameter is declared as p("name", default).
    source = (ROOT / "failsafe_teleop" / (module + ".py")).read_text()
    return set(re.findall(r'\bp\("(\w+)"', source))


def read_params(path):
    """Read the node: / ros__parameters: / key: value layout used in config/."""
    params = {}
    node = None
    for line in path.read_text().splitlines():
        text = line.split("#")[0].rstrip()
        if not text or text.strip() == "ros__parameters:":
            continue
        if not line.startswith(" "):
            node = text.rstrip(":")
            params[node] = {}
        else:
            key, value = text.strip().split(":", 1)
            params[node][key] = value.strip()
    return params


@pytest.mark.parametrize("filename,name,module", LAUNCHES)
def test_launch_file_starts_node_with_params_file(fake_launch, filename, name, module):
    entities = load_launch(filename).args[0]
    args = [e for e in entities if isinstance(e, DeclareLaunchArgument)]
    nodes = [e for e in entities if isinstance(e, Node)]
    assert [a.args[0] for a in args] == ["params"]
    assert len(nodes) == 1

    node = nodes[0].kwargs
    assert node["package"] == "failsafe_teleop"
    assert node["executable"] == name
    assert node["name"] == name
    # The YAML file given as params=... is the only parameter source.
    assert [p.args for p in node["parameters"]] == [("params",)]

    # The executable exists, and the node uses the same name as the launch file.
    assert "%s = failsafe_teleop.%s:main" % (name, module) in (ROOT / "setup.py").read_text()
    assert 'super().__init__("%s")' % name in (ROOT / "failsafe_teleop" / (module + ".py")).read_text()


@pytest.mark.parametrize("filename,name,module", LAUNCHES)
def test_example_params_only_use_declared_names(filename, name, module):
    params = read_params(ROOT / "config" / "mega_rover.yaml")[name]
    declared = declared_params(module)
    for key in SAFETY_PARAMS[name]:
        assert key in declared
        assert key in params
    # A misspelt key would be ignored by ROS and the node would fall back to its default.
    assert set(params) <= declared
    assert float(params["max_linear"]) > 0.0
    assert float(params["max_angular"]) > 0.0
