# DimOS Control and Teleoperation

This context defines language for robot control and teleoperation concepts used across DimOS.

## Language

**Planar base twist**:
A velocity command for mobile/base-like motion constrained to planar movement, conventionally using linear.x, linear.y, and angular.z.
_Avoid_: 2D twist, 3D twist, bare twist

**Spatial EEF twist**:
A velocity command for end-effector motion with translational and rotational components in 3D space.
_Avoid_: 3D twist, end-effector velocity, bare twist
