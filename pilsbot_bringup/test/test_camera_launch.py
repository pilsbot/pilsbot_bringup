# What camera.launch.py asks of the CSI camera (jetson_camera): its topic names
# and its frame. The launch description is expanded without starting anything.
#
# Two depths. The include-level tests run everywhere: they stop at the
# jetson_camera include and check what camera.launch.py hands it (its launch
# arguments and the global parameters set in its scope). The node-level tests
# also load jetson_camera's own launch file (it holds the container and the
# component) and check the LoadNode request; they need jetson_camera installed
# and are skipped without it. The OAK's launch file is only identified by its
# package.
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions.load_composable_nodes import get_composable_node_load_request
import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE / 'launch' / 'camera.launch.py'

# JetsonCameraNode's default for its topic_name parameter (jetson_camera,
# src/JetsonCameraNode.cpp), used when nobody sets it.
JETSON_CAMERA_DEFAULT_TOPIC = 'image_raw'


def jetson_camera_installed():
    try:
        get_package_share_directory('jetson_camera')
    except PackageNotFoundError:
        return False
    return True


needs_jetson_camera = pytest.mark.skipif(
    not jetson_camera_installed(),
    reason='jetson_camera is not installed (it builds only where the CSI camera runs)')


def evaluate(context, substitutions):
    return perform_substitutions(context, normalize_to_list_of_substitutions(substitutions))


def included_package(include):
    # Humble's launch prints an unresolved substitution path as
    # "...FindPackageShare(pkg='<name>')..."; no public accessor.
    [package] = re.findall(r"FindPackageShare\(pkg='([^']+)'\)",
                           include.launch_description_source.location)
    return package


def expand(follow_jetson_camera=False, **arguments):
    """
    Visit camera.launch.py like `ros2 launch` does, without starting processes.

    Returns the includes that are not followed, as {package: (launch arguments,
    global parameters in force at the include)}, the LoadNode requests that
    would go to the containers, and the context after the launch file, in which
    the scopes of its groups are closed again. jetson_camera's launch file is
    followed only with follow_jetson_camera (it must be installed for that).
    """
    spec = importlib.util.spec_from_file_location('camera_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    includes, requests = {}, []

    def visit(entity):
        if isinstance(entity, IncludeLaunchDescription) and not (
                follow_jetson_camera and included_package(entity) == 'jetson_camera'):
            if entity.condition is None or entity.condition.evaluate(context):
                # SetParameter keeps its (name, value) pairs in the
                # 'global_params' launch configuration; launch_ros applies
                # them to every node started from here on, in this scope.
                includes[included_package(entity)] = (
                    {evaluate(context, k): evaluate(context, v)
                     for k, v in entity.launch_arguments},
                    dict(context.launch_configurations.get('global_params', [])))
            return
        if isinstance(entity, ComposableNodeContainer):
            if entity.condition is None or entity.condition.evaluate(context):
                # ComposableNodeContainer._ComposableNodeContainer__composable_node_descriptions:
                # no public accessor in Humble's launch_ros.
                for description in entity._ComposableNodeContainer__composable_node_descriptions:
                    requests.append(get_composable_node_load_request(description, context))
            return
        for sub_entity in entity.visit(context) or []:
            visit(sub_entity)

    visit(module.generate_launch_description())
    return includes, requests, context


def parameters(request):
    return {p.name: p.value for p in request.parameters}


def resolve(name, namespace):
    return name if name.startswith('/') else namespace.rstrip('/') + '/' + name


def csi_topics(request):
    """
    Return the topics the CSI node publishes, for a LoadNode request.

    This is a model of image_transport, not an observation: its CameraPublisher
    resolves the base topic (topic_name) in the node's namespace and puts
    camera_info next to it and the transports below it. A remap rule then
    renames only the topic it names: in Humble a remap of image_raw leaves
    image_raw/compressed where it was (checked against image_transport 3.1.13
    in a component container). Newer image_transport releases apply remaps to
    the sub-topics too; recheck this model when the distro changes.
    """
    namespace = request.node_namespace or '/'
    params = parameters(request)
    base = params['topic_name'].string_value if 'topic_name' in params \
        else JETSON_CAMERA_DEFAULT_TOPIC
    topics = camera_topics(resolve(base, namespace))
    remaps = {}
    for rule in request.remap_rules:
        source, target = rule.split(':=')
        remaps[resolve(source, namespace)] = resolve(target, namespace)
    return {kind: remaps.get(topic, topic) for kind, topic in topics.items()}


def camera_topics(image):
    """Image topic, its compressed transport and camera_info beside it (image_transport)."""
    return {'image': image,
            'compressed': image + '/compressed',
            'camera_info': image.rsplit('/', 1)[0] + '/camera_info'}


def csi_include(includes):
    """Launch arguments and global parameters camera.launch.py gives jetson_camera."""
    return includes['jetson_camera']


def recorded_camera_topics():
    config = yaml.safe_load((PACKAGE / 'config' / 'recording.yaml').read_text())
    return config['groups']['camera']


def robot_links():
    """Links of the real robot's URDF (pilsbot_description, hardware:=real)."""
    urdf = Path(get_package_share_directory('pilsbot_description'), 'urdf', 'pilsbot.urdf.xacro')
    result = subprocess.run([shutil.which('xacro'), str(urdf), 'hardware:=real'],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    return {link.get('name') for link in ET.fromstring(result.stdout).findall('link')}


def csi_request(requests):
    [request] = [r for r in requests if r.package_name == 'jetson_camera']
    return request


# --- CSI camera (jetson_camera), what camera.launch.py hands it ------------

def test_csi_include_sets_the_front_camera_topic():
    includes, _, _ = expand()

    _, global_params = csi_include(includes)
    assert global_params == {'topic_name': '/front_camera/image_raw'}


def test_csi_include_gets_front_camera_optical_frame():
    # An image is stamped with an optical frame (z forward, x right, y down;
    # REP 103), not with the x-forward front_camera_link.
    includes, _, _ = expand()

    arguments, _ = csi_include(includes)
    assert arguments == {'frame_id': 'front_camera_optical_frame'}


def test_the_csi_frame_is_a_link_of_the_robot():
    includes, _, _ = expand()

    arguments, _ = csi_include(includes)
    assert arguments['frame_id'] in robot_links()


def test_recording_records_the_topics_the_csi_include_asks_for():
    includes, _, _ = expand()

    _, global_params = csi_include(includes)
    topics = camera_topics(global_params['topic_name'])
    assert topics['compressed'] in recorded_camera_topics()
    assert topics['camera_info'] in recorded_camera_topics()


def test_the_csi_topic_stays_in_the_csi_scope():
    _, _, context = expand()

    assert not context.launch_configurations.get('global_params')


def test_csi_camera_off_starts_no_csi_node():
    includes, _, _ = expand(csi_camera='false')

    assert set(includes) == {'pilsbot_depthai'}


# --- CSI camera (jetson_camera), the node it starts --------------------------

@needs_jetson_camera
def test_csi_camera_publishes_under_front_camera():
    _, requests, _ = expand(follow_jetson_camera=True)

    assert csi_topics(csi_request(requests)) == {
        'image': '/front_camera/image_raw',
        'compressed': '/front_camera/image_raw/compressed',
        'camera_info': '/front_camera/camera_info',
    }


@needs_jetson_camera
def test_csi_camera_frame_is_front_camera_optical_frame():
    _, requests, _ = expand(follow_jetson_camera=True)

    frame = parameters(csi_request(requests))['frame_id'].string_value
    assert frame == 'front_camera_optical_frame'


@needs_jetson_camera
def test_recording_records_the_csi_camera():
    _, requests, _ = expand(follow_jetson_camera=True)

    topics = csi_topics(csi_request(requests))
    assert topics['compressed'] in recorded_camera_topics()
    assert topics['camera_info'] in recorded_camera_topics()


@needs_jetson_camera
def test_the_csi_settings_stay_in_the_csi_scope():
    _, _, context = expand(follow_jetson_camera=True)

    assert 'frame_id' not in context.launch_configurations
    assert not context.launch_configurations.get('global_params')
    assert not context.launch_configurations.get('ros_remaps')


@needs_jetson_camera
def test_csi_camera_off_loads_no_csi_node():
    _, requests, _ = expand(follow_jetson_camera=True, csi_camera='false')

    assert requests == []


# --- OAK (pilsbot_depthai) ---------------------------------------------------

def test_oak_gets_the_decoder_arguments():
    includes, _, _ = expand(csi_camera='false', enable_raw_decode='true')

    arguments, _ = includes['pilsbot_depthai']
    assert arguments == {'nvdecoder': 'nvv4l2decoder', 'enable_raw_decode': 'true'}
