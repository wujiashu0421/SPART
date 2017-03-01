# SPART - SPAcecraft Robotics Toolkit

SPART is a MATLAB/SIMULINK open-source modeling and control toolkit for orbiting spacecraft with robotic manipulators (i.e., manipulators with a floating base).

SPART is organized as a collection of MATLAB functions and Simulink blocks. These can be used to build plants, forward/inverse dynamic solvers, and controllers.

SPART allows to compute:

* Kinematics - rotation matrices, position vectors and homogeneous transformation matrices.
* Differential kinematics - Jacobians and operational space velocities.
* Dynamics - Generalized inertia and convective inertia matrices.
* Forward/Inverse dynamics (including the floating base case).

Additionally SPART supports:

* Symbolic computation of all the kinematic and dynamic magnitudes.
* Automatic Code Generation support.
	* Code can be integrated in Simulink models.
	* Efficient C/C++ code can be automatically generated and integrated into with other exixting projects (e.g., with ROS).
* URDF files (experimental support).


## Installation

Just clone or download the toolkit and run the `SPART2path.m` script. This will add all the SPART MATLAB functions and the Simulink library to the path and save it. Then you can use it as any other MATLAB toolbox.

To run an example go to `Examples/URDF_Tutorial` and run:

	URDF_Tutorial

## Documentation

Up to date documentation and tutorials are available at [spart.readthedocs.org](http://spart.readthedocs.org).

## Examples

Here is an example of a planar Desired-Reaction-Maneuver, where the kinematic redundancy of a manipulator is exploited to make the floating base point towards the end-effector, while this one is moving along a prescribed path.

![DRM](docs/source/Figures/DRM.gif "Desired-Reaction-Maneuver")

SPART can also be used to control real manipulators. Here is an example of resolved motion-rate control of the R5D3 manipulator (the tip of the end-effector is describing a triangle in space).

![R5D3](docs/source/Figures/R5D3.gif "R5D3 resolved motion-rate control")

## License

This software is released under the LGPLv3 license.


