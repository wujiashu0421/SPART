=====
SPART
=====

SPART is an open-source modeling and control toolkit for mobile-base robotic multibody systems with kinematic tree topologies (`i.e.`, open-loop multi-branched systems).
SPART is MATLAB-based and ROS-compatible, allowing to prototype in simulation and deploy to hardware controllers for robotic systems.

Given a URDF or a Denavit-Hartenberg (DH) description of a multibody system, SPART allows to compute:

* Kinematics -- pose of the links and joints (`i.e.`, homogeneous transformation matrices or rotation matrices and position vectors).
* Differential kinematics -- Operational space velocities and accelerations, as well as the analytic Jacobians and their time derivatives.
* Dynamics -- Generalized inertia and convective inertia matrices.
* Forward/Inverse dynamics solvers (including the floating base case).

Analytical expressions for all these kinematic and dynamic quantities can be obtained with SPART, as it supports symbolic computation.

Contents
========

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


License
=======

SPART is released under the `LGPLv3 <https://www.gnu.org/licenses/lgpl.html>`_ license.

.. image:: https://www.gnu.org/graphics/lgplv3-147x51.png
   :height: 51px
   :width: 147px
   :scale: 50 %
   :alt: LGPLv3

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

