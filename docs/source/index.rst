SPART - SPAcecraft Robotics Toolkit
===================================

SPART is a MATLAB/SIMULINK open-source modeling and control toolkit for orbiting spacecraft with robotic manipulators (i.e., manipulators with a floating base).

SPART is organized as a collection of MATLAB functions and Simulink blocks. These can be used to build plants, forward/inverse dynamic solvers, and controllers.

SPART allows to compute:

* Kinematics - rotation matrices, position vectors and homogeneous transformation matrices.
* Differential kinematics - Jacobians and operational space velocities.
* Dynamics - Generalized inertia and convective inertia matrices.
* Forward/Inverse dynamics (including the floating base case).

Additionally, SPART supports:

* Symbolic computation of all the kinematic and dynamic magnitudes.
* Automatic Code Generation support:

	* Code can be integrated in Simulink models.
	* Efficient C/C++ code can be automatically generated and integrated into with other existing projects (`e.g.`, with ROS).

* URDF files (experimental support).



Contents:
=========

.. toctree::
   :maxdepth: 1

   Installation
   Tutorial
   Robot_Model
   URDF
   DH
   URDF_Models
   Cite


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

