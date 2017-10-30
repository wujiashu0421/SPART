SPART
=====

SPART is a MATLAB modeling and control toolkit for mobile base robotic multibody systems with a kinematic tree topology (`i.e.`, open-loop multi-branched systems).

SPART is organized as a collection of MATLAB functions and Simulink blocks. These can be used to build plants, forward/inverse dynamic solvers, and controllers.

SPART allows to compute:

* Kinematics -- rotation matrices, position vectors, and homogeneous transformation matrices.
* Differential kinematics -- Jacobians and Natural Orthogonal Complement matrix and their time derivatives as well as operational space velocities and accelerations.
* Dynamics -- Generalized inertia and convective inertia matrices.
* Forward/Inverse dynamics solvers (including the floating base case).

Additionally, SPART supports:

* Symbolic computation of all the kinematic and dynamic magnitudes.
* Automatic Code Generation.

   * Code can be integrated in Simulink models.
   * Efficient C/C++ code can be automatically generated and integrated in other existing projects (`e.g.`, with ROS).

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
   Functions
   Help
   Cite


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

