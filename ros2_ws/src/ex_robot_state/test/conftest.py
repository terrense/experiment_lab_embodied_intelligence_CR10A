import math
import os
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))  # allow `pytest` without colcon install
REPO = PKG.parents[2]

SMALL_URDF = """<robot name="toy">
  <link name="world"/><link name="base_link"/><link name="Link1"/><link name="Link2"/><link name="tcp"/>
  <joint name="fix" type="fixed"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0.5"/></joint>
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="Link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" velocity="2"/></joint>
  <joint name="joint2" type="continuous"><parent link="Link1"/><child link="Link2"/>
    <origin xyz="1 0 0"/><axis xyz="0 1 0"/></joint>
  <joint name="tool" type="fixed"><parent link="Link2"/><child link="tcp"/><origin xyz="0 0 0.2"/></joint>
</robot>"""


@pytest.fixture
def toy_model():
    from ex_robot_state.core.urdf_model import UrdfModel
    return UrdfModel.from_string(SMALL_URDF)


def _expand_cr10():
    xacro = pytest.importorskip("xacro")
    path = PKG.parent / "ex_description" / "urdf" / "ex_cr10.urdf.xacro"
    # resolve $(find ex_description) without an installed workspace
    os.environ.setdefault("AMENT_PREFIX_PATH", "")
    text = path.read_text().replace("$(find ex_description)", str(PKG.parent / "ex_description"))
    tmp = Path(os.environ.get("PYTEST_TMP", "/tmp")) / "ex_cr10_expanded.xacro"
    tmp.write_text(text)
    return xacro.process_file(str(tmp)).toxml()


@pytest.fixture(scope="session")
def cr10_urdf():
    return _expand_cr10()


@pytest.fixture(scope="session")
def cr10_model(cr10_urdf):
    from ex_robot_state.core.urdf_model import UrdfModel
    return UrdfModel.from_string(cr10_urdf)


@pytest.fixture(scope="session")
def official_header():
    p = REPO / "third_party" / "dobot_main" / "dobot_bringup_v4" / "include" / "dobot_bringup" / "command.h"
    if not p.exists():
        pytest.skip("third_party/dobot_main not fetched (scripts/fetch_third_party.sh)")
    return p.read_text(encoding="utf-8")
