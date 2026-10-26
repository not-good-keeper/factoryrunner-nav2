from factoryrunner.obstacle import main, map_to_gazebo, remove_command, spawn_command
import pytest


def test_map_origin_is_robot_spawn_point():
    assert map_to_gazebo(0.0, 0.0) == (-8.0, 0.0)
    assert map_to_gazebo(6.5, -1.65) == pytest.approx((-1.5, -1.65))


def test_spawn_command_uses_gazebo_coordinates():
    cmd = spawn_command('/pkg/models/pallet_box.sdf', 'box_lap3', 6.5, -1.65, 0.25)
    assert cmd[:4] == ['ros2', 'run', 'ros_gz_sim', 'create']
    args = dict(zip(cmd[4::2], cmd[5::2]))
    assert args == {'-world': 'depot', '-file': '/pkg/models/pallet_box.sdf', '-name': 'box_lap3',
                    '-x': '-1.500', '-y': '-1.650', '-z': '0.25'}


def test_remove_command_names_the_model():
    cmd = remove_command('box_lap3')
    assert '/world/depot/remove' in cmd
    assert cmd[-1] == 'name: "box_lap3" type: MODEL'


def test_cli_requires_position_for_spawn():
    with pytest.raises(SystemExit):
        main(['spawn', '--x', '1.0'])


def test_cli_rejects_unknown_model():
    with pytest.raises(SystemExit):
        main(['spawn', '--x', '1', '--y', '2', '--model', 'forklift'])
