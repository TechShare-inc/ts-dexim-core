# Type stubs for pinocchio
# This file provides basic type hints for the most commonly used Pinocchio types and functions

import enum
from typing import Any, overload

import numpy as np
import numpy.typing as npt
import pinocchio as pin

class ArgumentPosition(enum.IntEnum):
    ARG0 = 0
    ARG1 = 1
    ARG2 = 2
    ARG3 = 3
    ARG4 = 4

class AssignmentOperatorType(enum.IntEnum):
    SETTO = 0
    ADDTO = 1
    RMTO = 2

class FrameType(enum.IntEnum):
    BODY = 0
    FIXED_JOINT = 1
    JOINT = 2
    OP_FRAME = 3
    SENSOR = 4

class ReferenceFrame(enum.IntEnum):
    """Enumeration of available reference frames in Pinocchio."""

    LOCAL = 0
    WORLD = 1
    LOCAL_WORLD_ALIGNED = 2

def SE3ToXYZQUAT(se3: pin.SE3) -> npt.NDArray:
    """
    Convert an SE3 transformation matrix to XYZ position and quaternion representation.

    Args:
        se3 (pin.SE3): A Pinocchio SE3 transformation object representing a 3D rigid body
                    transformation (rotation and translation).

    Returns:
        npt.NDArray: A numpy array containing the transformation data in the format
                    [x, y, z, qx, qy, qz, qw] where:
                    - x, y, z: translation components
                    - qx, qy, qz, qw: quaternion components (x, y, z, w format)

    Note:
        The quaternion follows the convention (qx, qy, qz, qw) where qw is the scalar part.
        The SE3 object contains both rotation (as SO3) and translation components which
        are extracted and converted to the appropriate format.
    """
    ...

def XYZQUATToSE3(xyzq: npt.NDArray) -> pin.SE3:
    """
    Convert a vector in the format [x, y, z, qx, qy, qz, qw] to a Pinocchio SE3 object.

    Args:
        xyzq (npt.NDArray): A numpy array containing the transformation data in the format
                           [x, y, z, qx, qy, qz, qw].

    Returns:
        pin.SE3: A Pinocchio SE3 object representing the 3D rigid body transformation.
    """
    ...

def computeFrameJacobian(
    model: Model,
    data: Data,
    q: np.ndarray,
    frame_id: int,
    reference_frame: ReferenceFrame = ReferenceFrame.LOCAL,
) -> np.ndarray: ...
def updateGeometryPlacements(
    model: Model, data: Data, geometry_model: GeometryModel, geometry_data: GeometryData
) -> None: ...
def dIntegrate(
    model: Model, q: npt.NDArray, v: npt.NDArray, arg: int
) -> npt.NDArray: ...
def normalize(model: Model, q: npt.NDArray) -> npt.NDArray: ...
def difference(model: Model, q1: npt.NDArray, q2: npt.NDArray) -> npt.NDArray: ...
def computeJointJacobians(model: Model, data: Data, q: npt.NDArray) -> None: ...
def getFrameJacobian(
    model: Model, data: Data, frame_id: int, reference_frame: ReferenceFrame
) -> np.ndarray: ...

class SE3:
    translation: npt.NDArray
    rotation: npt.NDArray
    homogeneous: npt.NDArray
    def actInv(self, other: SE3) -> SE3: ...
    def inverse(self) -> SE3: ...
    @staticmethod
    def Identity() -> SE3: ...
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, rotation: npt.NDArray, translation: npt.NDArray) -> None: ...
    def copy(self) -> SE3: ...
    def __mul__(self, other: SE3) -> SE3: ...

class SO3: ...
class SO2: ...
class R3: ...
class SE2: ...

liegroups = [
    R3(),
    SO3(),
    SO2(),
    SE3(),
    SE2(),
]

class JointModel:
    def extract(self) -> Any: ...
    def shortname(self) -> str: ...
    def __init__(self) -> None: ...
    nq: int
    nv: int
    idx_q: int
    idx_v: int
    nvExtended: int
    ...

class JointData:
    v: npt.NDArray
    a: npt.NDArray
    S: npt.NDArray
    U: npt.NDArray
    Dinv: npt.NDArray
    def __init__(self, joint_model: JointModel) -> None: ...
    def extract(self) -> Any: ...
    def shortname(self) -> str: ...

class Model:
    """
    Pinocchio Model class representing a robotic system.

    This class contains the kinematic and dynamic model of a robot, including
    joint configuration spaces, frames, and limits.

    Attributes:
        nq (int): Number of configuration variables. The size of the configuration
            vector q describing all variables needed for the robot's pose (position
            and orientation of all joints). For revolute/prismatic joints: 1 value
            per joint. For floating-base robots: extra values (usually 7 for SE3:
            3 for position, 4 for quaternion orientation).
        nv (int): Number of velocity variables. The size of the velocity vector,
            which may differ from nq due to constraints (e.g., quaternion normalization).
        njoints (int): Total number of joints in the model, including the universe joint.
        nframes (int): Number of operational frames defined in the model.
        name (str): Name identifier for the robot model.
        names (list[str]): List of joint names in the model.
        lowerPositionLimit (npt.NDArray): Lower bounds for joint position limits.
        upperPositionLimit (npt.NDArray): Upper bounds for joint position limits.
        frames (list[Frame]): List of all operational frames in the model.
        mimicked_joints (list[int]): Vector of mimicked joints in the tree (can be any joint type).
            The i-th element of this vector corresponds to the mimicked joint of the i-th
            mimicking vector in mimicking_joints.
        mimicking_joints (list[int]): Vector of mimicking joints in the tree (with type MimicTpl).

    Methods:
        existFrame(name): Check if a frame with the given name exists.
        getJointId(name): Get the index of a joint by its name.
        getFrameId(name): Get the index of a frame by its name.
        addFrame(frame): Add a new frame to the model.
        createData(): Create a Data object compatible with this model for computations.
    """

    nq: int
    nv: int
    nvExtended: int
    njoints: int

    nqs: list[int]
    nvs: list[int]
    nvExtendeds: list[int]

    nframes: int
    name: str
    names: list[str]
    lowerPositionLimit: npt.NDArray
    upperPositionLimit: npt.NDArray
    frames: list[Frame]
    mimicked_joints: list[int]
    mimicking_joints: list[int]
    joints: list[pin.JointModel]
    idx_qs: list[int]
    idx_vs: list[int]
    idx_vExtendeds: list[int]
    parents: list[int]
    jointPlacements: list[SE3]
    def existFrame(self, name: str) -> bool: ...
    def getJointId(self, name: str) -> int: ...
    def getFrameId(self, name: str) -> int: ...
    def getBodyId(self, name: str) -> int: ...
    def addFrame(self, frame: Frame, append_inertia: bool = False) -> int: ...
    def addJoint(
        self, parent: int, joint: JointModel, placement: SE3, name: str
    ) -> int: ...
    def appendBodyToJoint(
        self,
        joint_id: int,
        inertia: Inertia,
        body_placement: SE3 = pin.SE3.Identity(),
    ) -> None: ...
    def addJointFrame(self, joint_id: int, frame_id: int = -1) -> int: ...
    def addBodyFrame(
        self,
        body_name: str,
        parent_joint_id: int,
        body_placement: SE3 = pin.SE3.Identity(),
        parent_frame_id: int = -1,
    ) -> int: ...
    def createData(self) -> Data: ...

class Data:
    oMf: list[SE3]
    oMi: list[SE3]
    J: npt.NDArray
    joints: list[Any]
    def __init__(self, model: Model) -> None: ...

class Quaternion:
    @staticmethod
    def Identity() -> Quaternion: ...
    def coeffs(self) -> npt.NDArray: ...
    def __init__(self, rotation: npt.NDArray) -> None: ...
    def __mul__(self, other: Quaternion) -> Quaternion: ...

class Inertia:
    @staticmethod
    def Zero() -> Inertia: ...
    @staticmethod
    def Random() -> Inertia: ...
    @staticmethod
    def Identity() -> Inertia: ...
    mass: float
    com: npt.NDArray
    inertia: npt.NDArray
    def __init__(self, mass: float, com: npt.NDArray, inertia: npt.NDArray) -> None: ...
    def copy(self) -> Inertia: ...
    def __mul__(self, other: Inertia) -> Inertia: ...
    ...

class Frame:
    name: str
    # parent: int
    parentJoint: int
    parentFrame: int
    placement: SE3
    type: FrameType
    def __init__(
        self,
        name: str,
        parent_joint: int,
        parent_frame: int,
        frame_placement: SE3,
        type: int,
        inertia: Inertia = Inertia.Zero(),
    ) -> None: ...

class GeometryData:
    oMg: list[SE3]
    def __init__(self, model: GeometryModel) -> None: ...

class CollisionPair:
    first: int
    second: int
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, geom_id1: int, geom_id2: int) -> None: ...

class GeometryModel:
    geometryObjects: list[GeometryObject]
    collisionPairs: list[CollisionPair]
    ngeoms: int
    collisionPairMapping: np.typing.NDArray
    def addGeometryObject(self, object: GeometryObject) -> int: ...
    def removeGeometryObject(self, name: str) -> None: ...
    def getGeometryId(self, name: str) -> int: ...
    def existGeometryName(self, name: str) -> bool: ...
    def addCollisionPair(self, geom_id1: int, geom_id2: int) -> None: ...
    def addAllCollisionPairs(self) -> None: ...
    def removeCollisionPair(self, object: CollisionPair) -> None: ...
    def removeAllCollisionPairs(self) -> None: ...
    def existCollisionPair(self, object: CollisionPair) -> bool: ...
    def findCollisionPair(self, object: CollisionPair) -> int: ...
    def createData(self) -> GeometryData: ...

class GeometryObject:
    @overload
    def __init__(
        self,
        name: str,
        parent_joint: int,
        parent_frame: int,
        placement: pin.SE3,
        collision_geometry: Any,
        meshPath: str = "",
        meshScale: np.ndarray = ...,
        overrideMaterial: bool = False,
        meshColor: np.ndarray = ...,
        meshTexturePath: str = "",
        meshMaterial: Any = ...,
    ) -> None: ...
    @overload
    def __init__(self, otherGeometryObject: GeometryObject) -> None: ...

    geometry: Any
    # Properties
    @property
    def meshScale(self) -> np.ndarray: ...
    @meshScale.setter
    def meshScale(self, value: np.ndarray) -> None: ...
    @property
    def name(self) -> str: ...
    @name.setter
    def name(self, value: str) -> None: ...
    @property
    def parentJoint(self) -> int: ...
    @parentJoint.setter
    def parentJoint(self, value: int) -> None: ...
    @property
    def parentFrame(self) -> int: ...
    @parentFrame.setter
    def parentFrame(self, value: int) -> None: ...
    @property
    def placement(self) -> pin.SE3: ...
    @placement.setter
    def placement(self, value: pin.SE3) -> None: ...
    @property
    def meshPath(self) -> str: ...
    @meshPath.setter
    def meshPath(self, value: str) -> None: ...
    @property
    def meshColor(self) -> np.ndarray: ...
    @meshColor.setter
    def meshColor(self, value: np.ndarray) -> None: ...
    @property
    def meshTexturePath(self) -> str: ...
    @meshTexturePath.setter
    def meshTexturePath(self, value: str) -> None: ...
    @property
    def overrideMaterial(self) -> bool: ...
    @overrideMaterial.setter
    def overrideMaterial(self, value: bool) -> None: ...

class Log6:
    vector: npt.NDArray
    linear: npt.NDArray
    angular: npt.NDArray

def buildModelFromUrdf(filename: str, mimic: bool = False) -> Model: ...
def buildGeomFromUrdf(model: Model, filename: str, type: Any) -> GeometryModel: ...
def buildModelsFromUrdf(
    filename, *args, **kwargs
) -> tuple[pin.Model, pin.GeometryModel, pin.GeometryModel]: ...
def appendModel(
    modelA: Model,
    modelB: Model,
    geomModelA: GeometryModel,
    geomModelB: GeometryModel,
    frame_id: int,
    placement: SE3,
) -> tuple[Model, GeometryModel]: ...
def forwardKinematics(model: Model, data: Data, q: npt.NDArray) -> None: ...
def updateFramePlacements(model: Model, data: Data) -> None: ...
def computeJointJacobian(
    model: Model, data: Data, q: npt.NDArray, joint_id: int
) -> npt.NDArray: ...
def neutral(model: Model) -> npt.NDArray: ...
def integrate(model: Model, q: npt.NDArray, v: npt.NDArray) -> npt.NDArray: ...
def log6(se3: SE3) -> Log6: ...
def Jlog6(se3: SE3) -> npt.NDArray: ...
def exp3(omega: npt.NDArray) -> npt.NDArray: ...
def randomConfiguration(model: Model) -> npt.NDArray: ...
@overload
def buildReducedModel(
    model: Model, joints_to_lock: list[int], q_reference: npt.NDArray
) -> Model: ...
@overload
def buildReducedModel(
    model: Model,
    geometry_models: list[GeometryModel],
    joints_to_lock: list[int],
    q_reference: npt.NDArray,
) -> tuple[Model, list[GeometryModel]]: ...
def unreduceConfiguration(
    model: Model,
    locked_joints: list[int],
    q_reference: npt.NDArray,
    q_reduced: npt.NDArray,
) -> npt.NDArray: ...

class GeometryType(enum.IntEnum):
    COLLISION: int
    VISUAL: int

class rpy:
    @staticmethod
    def rpyToMatrix(r: float, p: float, y: float) -> npt.NDArray: ...
    @staticmethod
    def matrixToRpy(matrix: npt.NDArray) -> npt.NDArray: ...

class visualize:
    class MeshcatVisualizer:
        viewer: Any
        def __init__(
            self,
            model: Model,
            collision_model: GeometryModel | None = None,
            visual_model: GeometryModel | None = None,
        ) -> None: ...
        def initViewer(self, open: bool = False) -> None: ...
        def loadViewerModel(self) -> None: ...
        def display(self, q: npt.NDArray) -> None: ...

class JointModelFreeFlyer(JointModel): ...
class JointModelRZ(JointModel): ...
class JointModelRY(JointModel): ...
class JointModelRX(JointModel): ...
class JointModelSpherical(JointModel): ...

class JointModelRevoluteUnaligned(JointModel):
    def __init__(self, axis: npt.NDArray) -> None: ...
