==============
SPART Tutorial
==============


This tutorial covers the basic functionality of the library and introduces basic concepts of multibody kinematics and dynamics. Before starting this tutorial make sure you have correctly installed and configured SPART (:doc:`/Installation`).

Getting the robot model
=======================

The first step is to create the model of the robotic multibody system. In SPART this model, containing all the kinematic and dynamic properties of the robot, is stored into the ``robot`` structure. A description of the contents of this ``robot`` structure can be found in the  :doc:`/Robot_Model` page.

SPART allows to populate the ``robot`` structure using three different input methods:

* **(Recommended method)** With a Unified Robot Description Format (URDF file). Refer to the :doc:`/URDF` page for the procedure to convert a URDF file into a SPART ``robot`` structure.
* With the Denavit-Hartenberg (DH) parameters. Refer to the :doc:`/DH` page for the procedure to create a SPART ``robot`` structure using DH parameters.
* Manually create the ``robot`` structure. Refer to the :doc:`/Robot_Model` page for further guidance.

The complete code for this tutorial can be found in ``examples/URDF_Tutorial/URDF_Tutorial.m``, when using a URDF file, or in ``examples/DH_Tutorial/DH_Tutorial.m``, when using DH parameters.


Kinematic tree topology
=======================

A multibody system refers to a collection of bodies coupled by joints. The bodies of the system – known as links – are arranged in one of the two types of kinematic chains:

* Kinematic trees (also known as open-loop kinematic chains). If the path between any two links is unique.
* Closed-loop kinematic chains. The path between any two links is not unique.

SPART is only able to handle multibody systems with *kinematic trees*.

.. figure:: Figures/KinematicTree.png
   :scale: 50 %
   :align: center
   :alt: Illustration of a kinematic tree.

   Illustration of a generic kinematic tree.

Numbering scheme
================

SPART uses a regular numbering scheme, identifying each link and joint with a number. Joints are  denoted by :math:`\mathcal{J}_{i}` and the links by :math:`\mathcal{L}_{i}`. One of the links on the kinematic tree is designated as the base link, with :math:`i=0` and denoted :math:`\mathcal{L}_{0}`. The base link can be selected arbitrarily among all the links, yet an obvious choice usually exists.

A link :math:`\mathcal{L}_{i}` can be connected to an arbitrary number of links via an equal number of joints. Given the assumption of a kinematic tree topology, only one of these links lies within the path connecting :math:`\mathcal{L}_{i}` and the **base-link** :math:`\mathcal{L}_{0}`. This *previous/upstream* link is referred to as the *parent link* of :math:`\mathcal{L}_{i}` and the joint connecting these two links is denoted by :math:`\mathcal{J}_{i}`. The rest of links directly connected to link :math:`\mathcal{J}_{i}` are called *child links* of :math:`\mathcal{L}_{i}`. Each link :math:`\mathcal{L}_{i}` has one and only one parent, but can have an arbitrary number of children links. When a link has more than one children this is called a *branching event*.

In a regular number scheme each children link is given a higher number than its parent, with the base link given the number :math:`i=0`. If the kinematic tree has multiple branches, multiple numbering options exist and they can be chosen arbitrarily among them.

.. figure:: Figures/RegularNumberingScheme.png
   :scale: 50 %
   :align: center
   :alt: Regular numbering scheme.

   Regular numbering scheme.

The notation :math:`i+1` and :math:`i-1` is abused to denote the child and parent link or joint even when they are not sequentially numbered. Additionally, the last joint and link on a branch is denoted by :math:`i=n`. This last link is also referred to as **end-effector**.

Kinematics
==========

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
* g -- Vector from the origin of the origin of the ith joint to origin of the ith link projected in the inertial CCS -- as a [3xn] matrix. 

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



Equations of motion and inertia matrices
========================================

The generic equations of motion, written in a canonical form, take the following form:

.. math::
	
	\mathbf{H}\dot{\mathbf{u}}+\mathbf{C}\mathbf{u}=\mathbf{\tau}

with :math:`\mathbf{H}\left(\mathcal{Q}\right)` being the Generalized Inertia Matrix (GIM), :math:`\mathbf{C}\left(\mathcal{Q},\mathbf{u}\right)` the Convective Inertia Matrix (CIM) and :math:`\mathbf{\tau}` the generalized forces.

The contributions of the base-link and the manipulator can be made explicit when writing the equations of motion.

.. math::
	
	\left[\begin{array}{cc} \mathbf{H}_{0} & \mathbf{H}_{0m}\\ \mathbf{H}_{0m}^{T} & \mathbf{H}_{m} \end{array}\right]
	\left[\begin{array}{c} \dot{\mathbf{u}}_{0}\\ \dot{\mathbf{u}}_{m} \end{array}\right]+
	\left[\begin{array}{cc} \mathbf{C}_{0} & \mathbf{C}_{0m}\\ \mathbf{C}_{m0} & \mathbf{C}_{m} \end{array}\right]
	\left[\begin{array}{c} \mathbf{u}_{0}\\ \mathbf{u}_{m} \end{array}\right]=
	\left[\begin{array}{c} \mathbf{\tau}_{0}\\ \mathbf{\tau}_{m} \end{array}\right]

These GIM and CIM are computes as follows.

.. code-block:: matlab

	%Inertias projected in inertial frame
	[I0,Im]=I_I(R0,RL,robot);
	%Mass Composite Body matrix
	[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
	%Generalized Inertia matrix
	[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
	%Generalized Convective Inertia matrix
	[C0, C0m, Cm0, Cm] = CIM(t0,tL,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

Although the equations of motion can be used to solve the forward dynamic problem (determining the motion of the system given a set of applied forces :math:`\mathbf{\tau}\rightarrow\dot{\mathbf{u}}`) and the inverse dynamic problem (determining the forces required to produce a prescribe motion :math:`\dot{\mathbf{u}}\rightarrow\mathbf{\tau}`) there are more computationally efficient ways of doing so.

Forward dynamics
================

To solve the forward dynamics, the forces acting on the multibody system are specified as an input. 

There are two methods of specifying them. Choose the one that is easier for your particular application (or both of them simultaneously).

The generalized forces :math:`\mathbf{\tau}` are the forces acting on the joints :math:`\mathbf{\tau}_{m}\in\mathbb{R}^{n}` and on the base-link :math:`\mathbf{\tau_{0}}\in\mathbb{R}^{6}`. For :math:`\mathbf{\tau}_{0}`, as in the twist vector, the torques :math:`\mathbf{n}_{0}\in\mathbb{R}^{3}`, projected in the base-link body-fixed CCS, come first and are followed by forces :math:`\mathbf{f}_{0}\in\mathbb{R}^{3}`.

.. math::

	\mathbf{\tau}_{0}=\begin{bmatrix}\mathbf{n}^{\left\{\mathcal{L}_{0}\right\}}_{0}\\ \mathbf{f}_{0} \end{bmatrix}

The wrench applied to the :math:`i\mathrm{th}` link, :math:`\mathbf{w}_{i}\in\mathbb{R}^{6}`, encapsulates the torques and forces, projected into the inertial CCS, applied to the center-of-mass of each link.

.. math::

	\mathbf{w}_{i}=\begin{bmatrix}\mathbf{n}_{i}\\ \mathbf{f}_{i} \end{bmatrix}


Here is an example of how to define them:

.. code-block:: matlab

	%Wrenches
	wF0=zeros(6,1);
	wFm=zeros(6,data.n);

	%Generalized forces
	tauq0=zeros(6,1);
	tauqm=zeros(robot.n_links,1);

After these forces are defined, a forward dynamic solver is available.

.. code-block:: matlab
	
	%Forward dynamics
	[u0dot_FD,umdot_FD] = FD(tau0,taum,wF0,wFm,t0,tL,P0,pm,I0,Im,Bij,Bi0,u0,um,robot);


As an example, if you need to incorporate the weight of the links (with the :math:`z`-axis being the vertical direction), set the wrenches as follows:

.. code-block:: matlab

	%Gravity
	g=9.8; %[m s-2]

	%Wrenches (includes gravity and assumes z is the vertical direction)
	wF0=zeros(6,1);
	wF0(6)=-robot.base_link(i).mass*g;
	wFm=zeros(6,robot.n_links);
	for i=1:robot.n_links
		wFm(6,i)=-robot.links(i).mass*g;
	end

Inverse dynamics
================

For the inverse dynamics, the acceleration of the base-link :math:`\dot{\mathbf{u}}_{0}` and of the joints :math:`\dot{\mathbf{u}}_{m}` is specified,  then, the ``ID`` function computed the inverse dynamics, providing the required forces to obtain these accelerations.

.. code-block:: matlab
	
	%Generalized accelerations
	u0dot=zeros(6,1);
	umdot=zeros(robot.n_q,1);

	%Oprational-space accelerations
	[t0dot,tLdot]=Accelerations(t0,tL,P0,pm,Bi0,Bij,u0,um,u0dot,umdot,robot);

	%Inverse Dynamics - Flying base
	[tau0,taum] = ID(wF0,wFm,t0,tL,t0dot,tLdot,P0,pm,I0,Im,Bij,Bi0,robot);


If the base-link is left uncontrolled :math:`\dot{\mathbf{\tau}}_{0}=\mathbf{0}` (floating-base case) and thus its acceleration is unknown the ``Floating_ID`` function is available.

.. code-block:: matlab
	
	%Accelerations
	umdot=zeros(robot.n_q,1);

	%Inverse Dynamics - Floating Base
	[taum_floating,u0dot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tL,P0,pm,I0,Im,Bij,Bi0,u0,um,umdot,robot);

