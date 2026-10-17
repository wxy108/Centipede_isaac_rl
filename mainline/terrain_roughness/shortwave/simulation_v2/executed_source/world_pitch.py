"""Convert sampled world pitch feedback into local implicit-drive targets.

The controller acts on the parent body through its pitch joint, with an equal
reaction on the child. It aims to keep the parent's local +X direction horizontal
in the world frame. It does not constrain yaw, roll, or height, and it is not an
exact world-frame constraint: callers refresh these targets each physics step.

This module initializes no simulator or array library. Pass NumPy or Torch as
the array namespace, with arrays or tensors belonging to that namespace.
"""

from typing import Any, TypeVar


Array = TypeVar("Array")


def world_pitch_references(
    array_namespace: Any,
    quaternion_wxyz: Array,
    angular_velocity_world: Array,
    joint_position: Array,
    joint_velocity: Array,
    joint_limits: Array,
) -> tuple[Array, Array, Array, Array, Array]:
    """Return position/velocity targets, world pitch/rate, and validity.

    Array shapes use a shared leading shape, usually ``(environments, joints)``:

    * ``quaternion_wxyz``: ``(..., 4)``, parent orientation in world coordinates.
      Quaternions are normalized here; callers must supply nonzero quaternions.
    * ``angular_velocity_world``: ``(..., 3)``, world angular velocity in rad/s.
    * ``joint_position`` and ``joint_velocity``: ``(...)``, in rad and rad/s.
    * ``joint_limits``: ``(..., 2)``, lower/upper limits in rad, broadcastable
      against the leading shape. For example, ``(1, joints, 2)`` is supported.

    The returned arrays have that leading shape, in this order:
    ``(position_target, velocity_target, world_pitch, world_pitch_rate, valid)``.
    The first four use rad, rad/s, rad, and rad/s; ``valid`` is Boolean.

    The pitch axis is the parent's local +Y axis in this model. World pitch is
    the elevation of local +X with the sign of positive rotation about +Y.
    The local target correction accounts for that axis's pitch sensitivity.
    Near-vertical or low-sensitivity states retain the measured position and
    velocity as targets; the final position-limit clamp still applies. There
    is no velocity-target clamp and no additional proportional/damping gain.
    The implicit joint drive retains its configured gains.

    Arithmetic order, thresholds, and clipping intentionally match the dated
    ``world_feedback.references`` implementation, including for float32 input.
    """
    normalized_quaternion = quaternion_wxyz / array_namespace.sqrt(
        (quaternion_wxyz * quaternion_wxyz).sum(-1, keepdims=True)
    )
    w, x, y, z = [normalized_quaternion[..., index] for index in range(4)]

    # Parent local +X expressed in world coordinates.
    forward_x = 1 - 2 * (y * y + z * z)
    forward_y = 2 * (x * y + w * z)
    forward_z = 2 * (x * z - w * y)
    horizontal_projection = array_namespace.sqrt(
        forward_x * forward_x + forward_y * forward_y
    )
    safe_horizontal_projection = array_namespace.clip(horizontal_projection, 0.1, None)
    world_pitch = array_namespace.arctan2(-forward_z, horizontal_projection)

    # Project world angular velocity onto the direction that changes pitch.
    pitch_gradient_x = -forward_y / safe_horizontal_projection
    pitch_gradient_y = forward_x / safe_horizontal_projection
    world_pitch_rate = (
        pitch_gradient_x * angular_velocity_world[..., 0]
        + pitch_gradient_y * angular_velocity_world[..., 1]
    )

    # Parent local +Y is the pitch joint axis; only its world X/Y components
    # enter the pitch-rate projection above.
    pitch_axis_x = 2 * (x * y - w * z)
    pitch_axis_y = 1 - 2 * (x * x + z * z)
    pitch_sensitivity = (
        pitch_gradient_x * pitch_axis_x + pitch_gradient_y * pitch_axis_y
    )
    valid = (horizontal_projection > 0.1) & (
        array_namespace.abs(pitch_sensitivity) > 0.2
    )
    denominator = array_namespace.where(
        valid, pitch_sensitivity, array_namespace.ones_like(pitch_sensitivity)
    )

    # Increasing relative joint position rotates the parent in the opposite
    # direction when the child is held, hence the positive correction here.
    position_target = array_namespace.where(
        valid, joint_position + world_pitch / denominator, joint_position
    )
    velocity_target = array_namespace.where(
        valid, joint_velocity + world_pitch_rate / denominator, joint_velocity
    )
    position_target = array_namespace.clip(
        position_target, joint_limits[..., 0], joint_limits[..., 1]
    )
    return position_target, velocity_target, world_pitch, world_pitch_rate, valid


def references(
    xp: Any,
    quat: Array,
    omega: Array,
    q: Array,
    qdot: Array,
    limits: Array,
) -> tuple[Array, Array, Array, Array, Array]:
    """Compatibility entry point retaining the original argument names/order."""
    return world_pitch_references(xp, quat, omega, q, qdot, limits)
