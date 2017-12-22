============================
SPART Tutorial -- Kinematics
============================

The code for this tutorial can be found in ``examples/URDF_Tutorial/URDF_Tutorial.m``.

Direct Kinematics
=================

SPART can compute the position and orientation of all the links and joints. To do so, the base-link position :math:`\mathrm{r}_{0}\in\mathbb{R}^{3}` and orientation, as a rotation matrix :math:`\mathrm{R}_{0}\in\mathrm{SO}\left(3\right)`, are first specified.

.. code-block:: matlab

	%Base-link position and orientation
	R0=eye(3);  %Rotation from base-link with respect to the inertial CCS.
	r0=[0;0;0]; %Position of the base-link with respect to the origin of the inertial frame, projected in the inertial CCS.

In SPART, the vectors are represented by a 3-by-1 column matrix containing the components of the projection to the inertial CCS. Projection to other CCS are explicitly marked.

The joint displacements, :math:`\mathbf{q}_{m}\in\mathbb{R}^{n}`, also needs to be defined.

.. code-block:: matlab

	%Joint displacements [rad or m]
	qm=[0;0;0;0;0]; %Adjust the length according to the number of joints in the robot model.

If the :math:`i\mathrm{th}` joint is revolute, ``qm(i)`` denotes a rotation, whether if the :math:`i\mathrm{th}` joint is prismatic ``qm(i)`` denotes a translation.

The set of ``R0,r0,qm`` constitute a set of variables :math:`\mathcal{Q}`, known as *generalized variables*, that fully define the state of the multibody system,

.. math::

	\mathcal{Q}=\left\lbrace\mathbf{R}_{0},\mathbf{r}_{0},q_{1},\ldots,q_{n}\right\rbrace


With the generalized variables specified, SPART is now ready to compute the kinematics of the system.

.. code-block:: matlab

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

The output of the function is as follows:

* RJ -- Joint 3x3 rotation matrices -- as a [3x3xn] matrix.
* RL -- Links 3x3 rotation matrices -- as a [3x3xn] matrix.
* rJ -- Positions of the joints projected in the inertial CCS -- as a [3xn] matrix.
* rL -- Positions of the links projected in the inertial CCS -- as a [3xn] matrix.
* e -- Joint rotation/sliding axis projected in the inertial CCS -- as a [3xn] matrix.
* g -- Vector from the origin of the ith joint CCS to the origin of the ith link CCS, projected in the inertial CCS -- as a [3xn] matrix.

The geometric definitions of these quantities are shown in the following figure.

.. figure:: Figures/KinematicsDef.png
   :scale: 50 %
   :align: center
   :alt: Definition of the kinematic quantities.

   Schematic disposition of a generic link and its associated joint.


If the joint variables :math:`\mathbf{q}_{m}` or base-link position :math:`\mathbf{r}_{0}` or orientation :math:`\mathbf{R}_{0}` are changed, re-running the ``Kinematics`` function afterwards computes the link and joint positions and orientations with that particular configuration.

.. code-block:: matlab

	%Joint displacements [rad or m]
	qm=[45;10;-45;20;-90]*pi/180; %Assumes revolute joints

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

If your Matlab installation includes the `Symbolic Math Toolbox <https://www.mathworks.com/products/symbolic.html>`_ SPART is able to obtain the analytic expressions of the kinematic quantities. To do so, just define the generalized variables as *symbolic expressions*.

.. code-block:: matlab

	%Joint displacements [rad or m]
	qm=sym('qm',[robot.n_q,1],'real');

	%Base-link position
	r0=sym('r0',[3,1],'real');

	%Base-link orientation
	Euler_Ang=sym('Euler_Ang',[3,1],'real');
	R0 = Angles321_DCM(Euler_Ang)';

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

.. warning::
   To obtain analytic expressions, all inputs must be symbolic. Otherwise, errors can occur.

Differential kinematics
=======================

The angular and linear velocity of the :math:`i\mathrm{th}` link with respect to the inertial frame, projected into the inertial CCS, is encapsulated into the **twist** :math:`\mathbf{t}_{i}\in\mathbb{R}^{6}`.

.. math::

	\mathbf{t}_{i}=\begin{bmatrix}\mathbf{\omega}_{i}\\ \dot{\mathbf{r}}_{i}\end{bmatrix}

The twist can be recursively propagated outward from one link to the next, using the 6-by-6 :math:`\mathbf{B}_{ij}` twist--propagation matrix and the 6-by-1 :math:`\mathbf{p}_{i}` twist--propagation "vector":

.. math::
	
	\mathbf{t}_{i}=\mathbf{B}_{ij}\mathbf{t}_{j}+\mathbf{p}_{i}\dot{q}_{i}\quad\text{for}\quad j=i-1

These matrices, which form the basis of the differential kinematics, can be computed with the ``DiffKinematics`` function.

.. code-block:: matlab

	%Differential kinematics
	[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

The output of the differential kinematics are as follows:

* Bij -- Twist--propagation [6x6xn] matrix (for manipulator i>0 and j>0).
* Bi0 -- Twist--propagation [6x6xn] matrix (for i>0 and j=0).
* P0 -- Base--link twist--propagation [6x6] matrix.
* pm -- Manipulator twist--propagation [6xn] vector.

The set of generalized velocities :math:`\mathbf{u}` contains the base-link velocities :math:`\mathbf{u}_{0}\in\mathbb{R}^{6}` and the joint velocities :math:`\mathbf{u}_{m}\in\mathbb{R}^{n}`. 

.. math::

	\mathbf{u} = \begin{bmatrix}\mathbf{u}_{0} \\ \mathbf{u}_{m} \end{bmatrix}

With the base-link and joint velocities defined as:

.. math::

	\mathbf{u}_{0} = \begin{bmatrix}\mathbf{\omega}^{\left\{\mathcal{L}_{0}\right\}}_{0} \\ \dot{\mathbf{r}}_{0} \end{bmatrix}

	\mathbf{u}_{m} = \begin{bmatrix}\dot{q}_{1} \\ \vdots \\ \dot{q}_{n} \end{bmatrix}

Note that :math:`\mathbf{\omega}^{\left\{\mathcal{L}_{0}\right\}}_{0}` denotes the angular velocity of the base-link, with respect to the inertial frame, projected in the base-link body-fixed CCS.

For the base-link, the twist is computed only using a modified 6-by-6 :math:`\mathbf{P}_{0}` twist-propagation matrix.

.. math::
	
	\mathbf{t}_{0}=\mathbf{P}_{0}u_{0}


With this quantities the velocities of all the links can be determined.
	
.. code-block:: matlab

	%Velocities (joint space)
	u0=zeros(6,1); %Base-link angular (projected into the base-link body-fixed CCS) and linear velocities.
	um=[4;-1;5;2;1]*pi/180; %Joint velocities (adjust the length according to your robot model)

	%Velocities (operational space)
	[t0,tL]=Velocities(Bij,Bi0,P0,pm,u0,um,robot);

The output of the operational space velocities are as follows:

* t0 -- Base--link twist vector projected in the inertial CCS -- as a [6x1] matrix.
* tL -- Manipulator twist vector projected in the inertial CCS -- as a [6xn] matrix.

Jacobians
=========

The analytic Jacobian of a point :math:`p` maps the joint-space velocities :math:`\mathbf{u}` into theoperational-space velocities of that point :math:`\mathbf{t}_{p}`.

.. math::
	
	\mathbf{t}_{p}=\mathbf{J}_{0p}\mathbf{u}_{0}+\mathbf{J}_{mp}\mathbf{u}_{m}

The analytical Jacobians of the :math:`i\mathrm{th}` link of the multibody system is computed as follows.

.. code-block:: matlab

	%Jacobian of the ith Link
	[J0i, Jmi]=Jacob(rL(1:3,i),r0,rL,P0,pm,i,robot);

To compute the Jacobian of a point 'p' in the :math:`i\mathrm{th}` use the following snippet.

.. code-block:: matlab

	%Jacobian of the a point p in the ith link
	%rp is the position of the point p, projected into the inertial CCS -- as a [3x1] matrix.
	[J0p, Jmp]=Jacob(rp,r0,rL,P0,pm,i,robot);


Continue the tutorial with the :doc:`/Tutorial_Dynamics` section.

