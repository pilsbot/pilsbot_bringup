# What camera.launch.py asks of the CSI camera (jetson_camera): its topic names
# and its frame. The launch description is expanded without starting anything.
# jetson_camera's own launch file is loaded (it holds the container and the
# component), so these tests need jetson_camera installed and are skipped
# without it; the OAK's launch file is only identified by its package.
import importlib.util
from pathlib import Path
import re

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


def expand(**arguments):
    """
    Visit camera.launch.py like `ros2 launch` does, without starting processes.

    Returns the launch arguments of the includes that are not followed (the OAK),
    the LoadNode requests that would go to the containers, and the context after
    the launch file, in which the scopes of its groups are closed again.
    """
    spec = importlib.util.spec_from_file_location('camera_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    context = LaunchContext()
    context.launch_configurations.update(arguments)
    includes, requests = {}, []

    def visit(entity):
        if isinstance(entity, IncludeLaunchDescription) and \
                included_package(entity) != 'jetson_camera':
            if entity.condition is None or entity.condition.evaluate(context):
                includes[included_package(entity)] = {
                    evaluate(context, k): evaluate(context, v)
                    for k, v in entity.launch_arguments}
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

    image_transport's CameraPublisher resolves the base topic (topic_name) in the
    node's namespace and puts camera_info next to it and the transports below it.
    A remap rule then renames only the topic it names: in Humble a remap of
    image_raw leaves image_raw/compressed where it was (checked against
    image_transport 3.1.13 in a component container).
    """
    namespace = request.node_namespace or '/'
    params = parameters(request)
    base = params['topic_name'].string_value if 'topic_name' in params \
        else JETSON_CAMERA_DEFAULT_TOPIC
    image = resolve(base, namespace)
    topics = {'image': image,
              'compressed': image + '/compressed',
              'camera_info': image.rsplit('/', 1)[0] + '/camera_info'}
    remaps = {}
    for rule in request.remap_rules:
        source, target = rule.split(':=')
        remaps[resolve(source, namespace)] = resolve(target, namespace)
    return {kind: remaps.get(topic, topic) for kind, topic in topics.items()}


def csi_request(requests):
    [request] = [r for r in requests if r.package_name == 'jetson_camera']
    return request


# --- CSI camera (jetson_camera) ----------------------------------------------

@needs_jetson_camera
def test_csi_camera_publishes_under_front_camera():
    _, requests, _ = expand()

    assert csi_topics(csi_request(requests)) == {
        'image': '/front_camera/image_raw',
        'compressed': '/front_camera/image_raw/compressed',
        'camera_info': '/front_camera/camera_info',
    }


@needs_jetson_camera
def test_csi_camera_frame_is_front_camera_link():
    _, requests, _ = expand()

    assert parameters(csi_request(requests))['frame_id'].string_value == 'front_camera_link'


@needs_jetson_camera
def test_recording_records_the_csi_camera():
    _, requests, _ = expand()
    config = yaml.safe_load((PACKAGE / 'config' / 'recording.yaml').read_text())

    topics = csi_topics(csi_request(requests))
    assert topics['compressed'] in config['groups']['camera']
    assert topics['camera_info'] in config['groups']['camera']


@needs_jetson_camera
def test_the_csi_settings_stay_in_the_csi_scope():
    _, _, context = expand()

    assert 'frame_id' not in context.launch_configurations
    assert not context.launch_configurations.get('global_params')
    assert not context.launch_configurations.get('ros_remaps')


def test_csi_camera_off_starts_no_csi_node():
    includes, requests, _ = expand(csi_camera='false')

    assert requests == []
    assert set(includes) == {'pilsbot_depthai'}


# --- OAK (pilsbot_depthai) ---------------------------------------------------

def test_oak_gets_the_decoder_arguments():
    includes, _, _ = expand(csi_camera='false', enable_raw_decode='true')

    assert includes['pilsbot_depthai'] == {'nvdecoder': 'nvv4l2decoder',
                                           'enable_raw_decode': 'true'}
