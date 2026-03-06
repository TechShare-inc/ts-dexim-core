"""Spatial transformation utilities for joint transforms and poses."""

from dataclasses import dataclass
from typing import Any, Dict, overload

import numpy as np
import numpy.typing as npt
from scipy.spatial.transform import Rotation as R

# Optional dependencies for robotics integrations
try:
    import pinocchio as pin

    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False

try:
    from viser.transforms import SE3 as ViserSE3

    HAS_VISER = True
except ImportError:
    HAS_VISER = False


# =================== EXCEPTIONS ===================


class SrcDstSizeMismatchError(Exception):
    """Raised when source and destination point sets have different sizes."""

    pass


class InvalidPointDimError(Exception):
    """Raised when points have invalid dimensions (not 2D or 3D)."""

    pass


class NotEnoughPointsError(Exception):
    """Raised when there are not enough points for transformation calculation."""

    pass


class RankDeficiencyError(Exception):
    """Raised when the point set has insufficient rank for transformation."""

    pass


# =================== RIGID TRANSFORM FUNCTION ===================


def rigid_transform(
    src_pts: np.ndarray, dst_pts: np.ndarray, calc_scale: bool = False
) -> tuple[np.ndarray, np.ndarray, float]:
    """Calculates the optimal rigid transform from src_pts to dst_pts.

    The returned transform minimizes the following least-squares problem
        r = dst_pts - (R @ src_pts + t)
        s = sum(r**2))

    If calc_scale is True, the similarity transform is solved, with the residual being
        r = dst_pts - (scale * R @ src_pts + t)
    where scale is a scalar.

    Parameters
    ----------
    src_pts: matrix of points stored as rows (e.g. Nx3)
    dst_pts: matrix of points stored as rows (e.g. Nx3)
    calc_scale: if True solve for scale

    Returns
    -------
    R: rotation matrix
    t: translation column vector
    scale: scalar, scale=1.0 if calc_scale=False
    """

    dim = src_pts.shape[1]

    if src_pts.shape != dst_pts.shape:
        raise SrcDstSizeMismatchError(
            f"src and dst points aren't the same matrix size {src_pts.shape=} != {dst_pts.shape=}"
        )

    if not (dim == 2 or dim == 3):
        raise InvalidPointDimError(f"Points must be 2D or 3D, src_pts.shape[1] = {dim}")

    if src_pts.shape[0] < dim:
        raise NotEnoughPointsError(f"Not enough points, expect >= {dim} points")

    # find mean/centroid
    centroid_src = np.mean(src_pts, axis=0)
    centroid_dst = np.mean(dst_pts, axis=0)

    centroid_src = centroid_src.reshape(-1, dim)
    centroid_dst = centroid_dst.reshape(-1, dim)

    # subtract mean
    # NOTE: doing src_pts -= centroid_src will modifiy input!
    src_pts = src_pts - centroid_src
    dst_pts = dst_pts - centroid_dst

    # the cross-covariance matrix minus the mean calculation for each element
    # https://en.wikipedia.org/wiki/Cross-covariance_matrix
    H = src_pts.T @ dst_pts

    rank = np.linalg.matrix_rank(H)

    if dim == 2 and rank == 0:
        raise RankDeficiencyError(
            f"Insufficent matrix rank. For 2D points expect rank >= 1 but got {rank}. Maybe your points are all the same?"
        )
    elif dim == 3 and rank <= 1:
        raise RankDeficiencyError(
            f"Insufficent matrix rank. For 3D points expect rank >= 2 but got {rank}. Maybe your points are collinear?"
        )

    # find rotation
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    # https://en.wikipedia.org/wiki/Kabsch_algorithm
    det = np.linalg.det(R)
    if det < 0:
        print(f"det(R) = {det}, reflection detected!, correcting for it ...")
        S = np.eye(dim)
        S[-1, -1] = -1
        R = Vt.T @ S @ U.T

    if calc_scale:
        scale = np.sqrt(np.mean(dst_pts**2) / np.mean(src_pts**2))
    else:
        scale = 1.0

    t = -scale * R @ centroid_src.T + centroid_dst.T

    return R, t, scale


# =================== TRANSFORM3D CLASS ===================


@dataclass
class Transform3D:
    """
    A 3D transformation with position and rotation.

    Internal Storage:
        position (npt.NDArray): 3D position vector [x, y, z]
        _rotation (R): scipy.spatial.transform.Rotation object

    Public Properties:
        rotation_matrix: 3x3 rotation matrix
        w_quat: quaternion with scalar-first [w, x, y, z]
        quat_w: quaternion with scalar-last [x, y, z, w]
        quat: alias for w_quat (backward compatibility)
        homogeneous: 4x4 transformation matrix

    Initialization accepts ONE of:
        - rotation_matrix: 3x3 rotation matrix
        - w_quat: scalar-first quaternion [w, x, y, z]
        - quat_w: scalar-last quaternion [x, y, z, w]
        - homogeneous: 4x4 transformation matrix
    """

    position: npt.NDArray
    """npt.NDArray: The 3D position vector [x, y, z]."""
    _rotation: R
    """R: Internal scipy Rotation object (private)."""

    def __init__(
        self,
        position: npt.NDArray | None = None,
        rotation_matrix: npt.NDArray | None = None,
        w_quat: npt.NDArray | None = None,
        quat_w: npt.NDArray | None = None,
        homogeneous: npt.NDArray | None = None,
    ):
        # Handle position
        if position is None:
            self.position = np.zeros(3, dtype=float)
        else:
            self.position = np.array(position, dtype=float).reshape(
                3,
            )

        # Handle rotation - count how many rotation arguments provided
        rotation_args = [
            rotation_matrix is not None,
            w_quat is not None,
            quat_w is not None,
            homogeneous is not None,
        ]

        if sum(rotation_args) > 1:
            raise ValueError(
                "Only one rotation format should be provided: "
                "rotation_matrix, w_quat, quat_w, or homogeneous"
            )

        # Parse rotation from provided format
        if homogeneous is not None:
            if homogeneous.shape != (4, 4):
                raise ValueError("homogeneous must be a 4x4 transformation matrix")
            self.position = homogeneous[:3, 3].copy()
            self._rotation = R.from_matrix(homogeneous[:3, :3])

        elif rotation_matrix is not None:
            if rotation_matrix.shape != (3, 3):
                raise ValueError("rotation_matrix must be a 3x3 rotation matrix")
            self._rotation = R.from_matrix(rotation_matrix)

        elif w_quat is not None:
            if w_quat.shape != (4,):
                raise ValueError(
                    "w_quat must be a quaternion of shape (4,) [w, x, y, z]"
                )
            self._rotation = R.from_quat(
                [w_quat[1], w_quat[2], w_quat[3], w_quat[0]]  # Convert to scipy format
            )

        elif quat_w is not None:
            if quat_w.shape != (4,):
                raise ValueError(
                    "quat_w must be a quaternion of shape (4,) [x, y, z, w]"
                )
            self._rotation = R.from_quat(quat_w)  # Already in scipy format

        else:
            # Default to identity rotation
            self._rotation = R.from_quat([0, 0, 0, 1])

        np.set_printoptions(precision=3, suppress=True)

    def __str__(self) -> str:
        return f"Transform3D(position={self.position}, w_quat={self.w_quat})"

    # =================== PROPERTIES ===================

    @property
    def rotation_matrix(self) -> npt.NDArray:
        """Get the 3x3 rotation matrix."""
        return self._rotation.as_matrix()

    @property
    def w_quat(self) -> npt.NDArray:
        """Get quaternion in scalar-first format [w, x, y, z]."""
        return self._rotation.as_quat(scalar_first=True)

    @property
    def quat_w(self) -> npt.NDArray:
        """Get quaternion in scalar-last format [x, y, z, w]."""
        return self._rotation.as_quat(scalar_first=False)

    @property
    def quat(self) -> npt.NDArray:
        """Get quaternion in scalar-first format [w, x, y, z] (alias for w_quat)."""
        return self.w_quat

    @property
    def homogeneous(self) -> npt.NDArray:
        """Get the 4x4 homogeneous transformation matrix."""
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation_matrix
        matrix[:3, 3] = self.position
        return matrix

    # =================== CLASS METHODS ===================

    @classmethod
    def from_matrix(cls, matrix: npt.NDArray) -> "Transform3D":
        """Create Transform3D from a 4x4 transformation matrix."""
        if matrix.shape != (4, 4):
            raise ValueError("matrix must be a 4x4 transformation matrix.")
        return cls(homogeneous=matrix)

    @classmethod
    def from_pin_SE3(cls, se3) -> "Transform3D":
        """Create Transform3D from a Pinocchio SE3 object.

        Requires pinocchio to be installed.
        """
        if not HAS_PINOCCHIO:
            raise ImportError(
                "pinocchio is required for this method. "
                "Install it via: conda install -c conda-forge pinocchio"
            )
        transform_matrix = se3.homogeneous
        return cls(homogeneous=transform_matrix)

    @classmethod
    def from_json_node(cls, node_data: Dict[str, Any]) -> "Transform3D":
        """Create Transform3D from JSON node data."""
        pos = node_data["position"]
        rot = node_data["rotation"]

        position = np.array([pos["x"], pos["y"], pos["z"]], dtype=float)
        w_quat = np.array([rot["w"], rot["x"], rot["y"], rot["z"]], dtype=float)

        return cls(position=position, w_quat=w_quat)

    @classmethod
    def from_translation_rpy(
        cls, translation: npt.NDArray, rpy: npt.NDArray
    ) -> "Transform3D":
        """Create Transform3D from translation and roll-pitch-yaw angles.

        Parameters
        ----------
        translation: 3D translation vector [x, y, z]
        rpy: roll-pitch-yaw angles in radians [roll, pitch, yaw]

        Returns
        -------
        Transform3D object
        """
        rotation = R.from_euler("xyz", rpy)
        return cls(position=translation, rotation_matrix=rotation.as_matrix())

    # =================== OPERATIONS ===================

    def __mul__(self, other: "Transform3D") -> "Transform3D":
        """Combine two Transform3D objects (self * other)."""
        # Combine rotations
        combined_rot = (self._rotation * other._rotation).as_matrix()

        # Rotate other's position by self's rotation and add positions
        rotated_pos = self._rotation.apply(other.position)
        combined_pos = self.position + rotated_pos

        return Transform3D(position=combined_pos, rotation_matrix=combined_rot)

    def inverse(self) -> "Transform3D":
        """Compute the inverse of the Transform3D."""
        inv_rot = self._rotation.inv()
        inv_pos = -inv_rot.apply(self.position)

        return Transform3D(position=inv_pos, rotation_matrix=inv_rot.as_matrix())

    def diff(self, other: "Transform3D") -> "Transform3D":
        """Compute the difference between this Transform3D and another.

        Computes: result = inv(self) * other
        """
        inv_self = self.inverse()
        return inv_self * other

    # =================== REBASE AND CONVERSIONS ===================

    @overload
    def rebase(self, local_transform: "Transform3D") -> "Transform3D": ...
    @overload
    def rebase(self, local_transform: npt.NDArray) -> "Transform3D": ...

    def rebase(self, local_transform) -> "Transform3D":
        """Rebase this Transform3D with a local transform.

        Computes: result = oM_self @ inv(oM_src) @ oM_dst
        All inputs are expected to be 4x4 homogeneous transform matrices.
        """
        if isinstance(local_transform, Transform3D):
            return self * local_transform
        elif isinstance(local_transform, np.ndarray):
            if local_transform.shape != (4, 4):
                raise ValueError(
                    "local_transform must be a 4x4 homogeneous transformation matrix."
                )
            return self * Transform3D(homogeneous=local_transform)
        else:
            raise TypeError(
                "local_transform must be either a Transform3D or a 4x4 numpy array."
            )

    def to_matrix(self) -> npt.NDArray:
        """Convert the Transform3D to a 4x4 transformation matrix."""
        return self.homogeneous

    def to_pin_SE3(self):
        """Convert the Transform3D to a Pinocchio SE3 object.

        Requires pinocchio to be installed.
        """
        if not HAS_PINOCCHIO:
            raise ImportError(
                "pinocchio is required for this method. "
                "Install it via: conda install -c conda-forge pinocchio"
            )
        return pin.SE3(self.rotation_matrix, self.position)

    def to_viser_SE3(self):
        """Convert the Transform3D to a Viser's SE3 object.

        Requires viser to be installed.
        """
        if not HAS_VISER:
            raise ImportError(
                "viser is required for this method. "
                "Install it via: pip install viser"
            )
        return ViserSE3.from_matrix(self.homogeneous)

    def to_xyzquat(self, scalar_first: bool = False) -> npt.NDArray:
        """Convert the Transform3D to a 7D vector [x, y, z, quat]."""
        if scalar_first:
            quat = self.w_quat
        else:
            quat = self.quat_w

        return np.concatenate([self.position, quat])

    @staticmethod
    def identity() -> "Transform3D":
        """Return an identity Transform3D."""
        return Transform3D(
            position=np.zeros(3), w_quat=np.array([1, 0, 0, 0], dtype=float)
        )
