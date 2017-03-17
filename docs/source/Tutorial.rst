==============
SPART Tutorial
==============


This SPART tutorial will cover the basic operation of the library. Before starting make sure you have correctly installed and configured SPART (:doc:`/Installation`).

Getting the Robot Model
=======================

The first step is to create the robot model. This model contains all the kinematic and dynamic data of the robot. Take a look at (:doc:`/Robot_Model`) to get a detailed description of how this model is structured.

There are 3 different ways to create the robot model:
	* Use an Unified Robot Description Format (URDF file). Check (:doc:`/URDF`) for a description on how to convert a URDF file into a SPART robot model.
	* Use the Denavit-Hartenberg (DH) convention to define the geometry of your robot. Check (:doc:`/DH`) for a description on how to convert a URDF file into a SPART robot model.
	* Manually create the robot model. Refer to (:doc:`/Robot_Model`) for further guidance.

Weather if you choose to start from a URDF file or from a DH description this tutorial code can be found either in ``examples/URDF_Tutorial/URDF_Tutorial.m`` or in ``examples/DH_Tutorial/DH_Tutorial.m``.

Kinematics
==========

After creating the robot model the next step is to compute the kinematics of the multi-body system.

First we need to define the position and orientation of the base.

.. code-block:: matlab

	%Base position
	R0=eye(3);  %Rotation from base-spacecraft to the inertial frame
	r0=[0;0;0]; %Position of the base-spacecraft in the inertial frame

Also the generalized joint variables need to be defined.

.. code-block:: matlab

	%Joint variables [rad]
	qm=[0;0;0;0;0]; %Adjust the length according to your robot model

We can now compute the kinematics of the system.

.. code-block:: matlab

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

The output of the function is as follows:
	* RJ -- Joint 3x3 rotation matrices -- as a [3x3xn] matrix.
	* RL -- Links 3x3 rotation matrices -- as a [3x3xn] matrix.
	* rJ -- Positions of the joints in the inertial frame -- as a [3xn] matrix.
	* rL -- Positions of the links in the inertial frame -- as a [3xn] matrix.
	* e -- Joints rotations axis in the inertial frame -- as a [3xn] matrix.
	* g -- Vector from the origin of the ith joint to the ith link in the inertial frame -- as a [3xn] matrix. 

Some of the geometric definitions are shown in following figure.

.. figure:: Figures/GenLinksJoints.png
   :scale: 50 %
   :align: center
   :alt: DH text parameters

   Schematic disposition of links and joints.


If you change the joint variables ``qm`` or base-spacecraft position ``r_{0}`` and orientation ``R_{0}`` to then re-run the kinematic function ``Kinematics`` you will get the new positions and orientation with that particular configuration.

.. code-block:: matlab

	%Joint variables [rad]
	qm=[45;10;-45;20;-90]*pi/180;

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

SPART also allows symbolic computation. To obtain symbolic expressions just define the joint variables as symbolic.

.. code-block:: matlab

	%Joint variables [rad]
	qm=sym('qm',[robot.n_q,1],'real');

	%Base-spacecraft position
	r0=sym('r0',[3,1],'real');

	%Base-spacecraft orientation
	Euler_Ang=sym('Euler_Ang',[3,1],'real');
	R0 = Angles321_DCM(Euler_Ang)';

	%Kinematics
	[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);

Differential Kinematics
=======================

To compute the differential kinematics the twist propagation matrices and twist propagation vectors need to be computed first.

.. code-block:: matlab

	%Differential Kinematics
	[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);

The output of the differential kinematics are as follows:
	* Bij -- Twist--propagation [6x6] matrix (for manipulator i>0 and j>0).
	* Bi0 -- Twist--propagation [6x6] matrix (for i>0 and j=0).
	* P0 -- Base--spacecraft twist--propagation [6x6] matrix.
	* pm -- Manipulator twist--propagation [6x1] vector.

With this quantities the velocities of all the links can be determined if the base ``q0dot`` and joint velocities ``qmdot`` are previously defined.
	
.. code-block:: matlab

	%Velocities (joint space)
	q0dot=zeros(6,1); %Base-spacecraft velocity [wx,wy,wz,vx,vy,vz].
	qmdot=[4;-1;5;2;1]*pi/180; %Joint velocities (adjust the length according to your robot model)


	%Velocities (operational space)
	[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);

The output of the operational space velocities are as follows:
	* t0 -- Base--spacecraft twist vector [wx,wy,wz,vx,vy,vz].
	* tm -- Manipulator twist vector [wx,wy,wz,vx,vy,vz].

The twist vector encapsulates the angular and linear velocities in a vector.

.. math::

	t_{i}=\left[\begin{array}{c}\omega_{i}\\\dot{r}_{i}\end{array}\right]

The twist vector can be propagated, from a link to the next one, using the 3x3 :math:`B_{ij}` twist--propagation matrix and the 6x1 :math:`p_{i}` twist--propagation vector as follows:

.. math::
	
	t_{i}=B_{ij}t_{j}+p_{i}\dot{q}_{i}

For the base-spacecraft, the twist--propagation only uses the a modified 6x6 :math:`P_{0}` twist-propagation matrix.

.. math::
	
	t_{0}=P_{0}\dot{q}_{0}

The analytical Jacobians of any point on the spacecraft-manipulator system can also be easily computed as follows:

.. code-block:: matlab

	%Jacobian of the 3rd Link
	[J03, Jm3]=Jacob(rL(1:3,3),r0,rL,P0,pm,3,robot);

In general for the Jacobian of the ``i`` linke:

.. code-block:: matlab

	%Jacobian of the ith Link
	[J03, Jm3]=Jacob(rL(1:3,i),r0,rL,P0,pm,i,robot);

The Jacobians map joint space velocities into operational space velocities.

.. math::
	
	t_{x}=J_{0x}\dot{q}_{0}+J_{mx}\dot{q}_{m}

Equations of Motion and Inertia Matrices
========================================

The generic equations of motion can be written as follows:

.. math::
	
	H\left(q\right)\ddot{q}+C\left(q,\dot{q}\right)\dot{q}=\mathcal{\tau}

with :math:`H` being the Generalized Inertia Matrix (GIM), :math:`C` the Convective Inertia Matrix (CIM), :math:`q` the generalized joint variables and :math:`\tau` the generalized joint forces.

The generalized joint variables are composed by the base-spacecraft variables :math:`q_{0}` and the manipulator joint variables :math:`q_{m}`.
The contributions of the base-spacecraft and the manipulator can be made explicit when writing the equations of motion.

.. math::
	
	\left[\begin{array}{cc} H_{0} & H_{0m}\\ H_{0m}^{T} & H_{m} \end{array}\right]
	\left[\begin{array}{c} \ddot{q}_{0}\\ \ddot{q}_{m} \end{array}\right]+
	\left[\begin{array}{cc} C_{0} & C_{0m}\\ C_{m0} & C_{m} \end{array}\right]
	\left[\begin{array}{c} \dot{q}_{0}\\ \dot{q}_{m} \end{array}\right]=
	\left[\begin{array}{c} \tau_{0}\\ \tau_{m} \end{array}\right]

To obtain the inertia matrices we need to specify the mass and inertia of the base--spacecraft and of the different manipulator links.

You can now compute these inertia matrices as follows.

.. code-block:: matlab

	%Inertias in inertial frames
	[I0,Im]=I_I(R0,RL,robot);
	%Mass Composite Body matrix
	[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
	%Generalized Inertia matrix
	[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
	%Generalized Convective Inertia matrix
	[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

Although the equations of motion can be used to solve the forward dynamic problem (determining the motion of the system given a set of applied forces :math:`\tau\rightarrow\ddot{q}`) and the inverse dynamic problem (determining the forces required to produce a prescribe motion :math:`\ddot{q}\rightarrow\tau`) there are more efficient ways of doing so.

Forward Dynamics
================

To solve the forward dynamics you will need to specify the forces acting on the spacecraft--manipulator system. There are two ways of specifying them. Choose the one that is easier for your particular application (or both of them simultaneously).

The joint forces :math:`\tau` are the forces acting on the joints :math:`\tau_{m}` (thus is an ``nx1`` vector) and also at the base-spacecraft :math:`tau_{0}` (thus a ``6x1`` vector). For :math:`\tau_{0}`, as in the twist vector, the torques come first and then the linear forces.

.. math::

	\tau_{0}=\left[\tau_{x},\tau_{y},\tau_{z},f_{x},f_{y},f_{z}\right]^{T}

Also, you can specify the wrenches :math:`w` (torques and forces) that are applied at their center-of-mass of each link. Again these can be decomposed into base-spacecraft 6x1 wrenches :math:`w_{0}` and manipulator {6xn} wrenches :math:`w_{n}`.

.. math::

	\w_{i}=\left[\tau_{x},\tau_{y},\tau_{z},f_{x},f_{y},f_{z}\right]^{T}

Here is an example of how to do it:

.. code-block:: matlab

	%External forces
	wF0=zeros(6,1);
	wFm=zeros(6,data.n);

	%Joint torques
	tauq0=zeros(6,1);
	tauqm=zeros(robot.n_links,1);

After these forces are defined, a forward dynamic solver is available.

.. code-block:: matlab
	
	%Forward Dynamics
	[q0ddot_FD,qmddot_FD] = FD(tau0,taum,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,robot);


As an example, if you need to incorporate the weight of the links (with z being the vertical direction), set the wrenches as follows:

.. code-block:: matlab

	%Gravity
	g=9.8; %[m s-2]

	%External forces (includes gravity and assumes z is the vertical direction)
	wF0=zeros(6,1);
	wFm=zeros(6,robot.n_links);
	for i=1:robot.n_links
    	wFm(1:6,6)=-robot.links(i).mass*g;
	end

Inverse Dynamics
================

For the inverse dynamics, the acceleration of the base-spacecraft and the joints need to be specified and then a function to compute the inverse dynamics is available.

.. code-block:: matlab
	
	%Accelerations
	q0ddot=zeros(6,1);
	qmddot=zeros(robot.n_q,1);

	%Accelerations
	[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot);

	%Inverse Dynamics - Flying base
	[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);


If the base-spacecraft is left uncontrolled (floating-base case) and thus its acceleration is unknown a different routine is available.

.. code-block:: matlab
	
	%Accelerations
	qmddot=zeros(robot.n_q,,1);

	%Inverse Dynamics - Floating Base
	[taum_floating,q0ddot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,robot);

