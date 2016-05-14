SPART Tutorial
==============


This SPART tutorial will cover the basic operation of the library. Before starting make sure you have correctly installed and configured SPART.

In this tutorial we will model the following spacecraft with a 5 degree-of-freedom manipulator with the following dimensions: L0=50, L1=125, L2=144, L3=47, L4=142, L5=80, L6=70 mm.

.. figure:: Figures/Tutorial_Configuration.png
   :scale: 50 %
   :align: center
   :alt: Tutorial model configuration

   Spacecraft-manipulator system.

Before SPART can start computing any kinematic or dynamic magnitude it needs a representation of the geometry and dynamic properties of the spacecraft-manipulator system. These fixed parameters are introduced into a `data` structure.

The data structure has 3 fields:
	* data.n -- Number of links.
	* data.man(i) -- Describes the ith link/joint.
		* data.man(i).type -- Type of joint. 0 for a revolute joint, 1 for a prismatic joint.
		* data.man(i).DH -- Denavit-Hartenberg parameters. Definitions are included below.
		* data.man(i).b -- Vector from the Center-of-Mass of the link to the next joint in the local frame.
		* data.man(i).mass -- Mass of the link.
		* data.man(i).I -- Inertia matrix of the link.
	* data.base -- Describes the base-spacecraft.
		* data.base.T_L0_J1 -- Homogeneous transformation matrix from the first 
		* data.base.mass -- Mass of the base-spacecraft.
		* data.base.I -- Inertia matrix of the base-spacecraft
	* data.EE -- Provides additional information about the end-effector
		* data.EE.theta -- Final rotation about the z axis. Last joint DH parameters are not enough to completely specify the orientation of the x and y end-effector axes.
		* data.EE.d -- Final translation about the z axis. Last joint DH parameters are not enough to completely specify the location of the end-effector frame origin along the z-axis.

The kinematic relationship between a pair of connected joints is expressed by the Denavit-Hartenberg parameters.

.. figure:: Figures/DH.png
   :scale: 50 %
   :align: center
   :alt: DH parameters

   Denavit-Hertenberg parameters.

.. figure:: Figures/DH_TextDef.png
   :scale: 50 %
   :align: center
   :alt: DH text parameters

   Denavit-Hartenberg parameters and their geometric definition.

Additional geometric definitions are included in the following figure.

.. figure:: Figures/GenLinksJoints.png
   :scale: 50 %
   :align: center
   :alt: DH text parameters

   Schematic disposition of links and joints.

We will also assume that the links are homogeneous bodies and so their center-of-mass will be located at their geometric center, which allows to easily determine the data.man(i).b vector. The data.man(i).b vector goes from the center-of-mass of the ith link to the origin of the ith+1 joint in the local ith link reference frame.
Using the DH parameters and assuming homogeneous bodies the DH and b vector are given below.

===========  ======  ==========  ===================  ===============  =================
Joint/Link    DH.d    DH.alpha          DH.a             DH.theta              b
===========  ======  ==========  ===================  ===============  =================
 Joint 1       L1       pi/2             0                   0             [0;L1/2;0] 
 Joint 2       0         0         sqrt(L2^2+L3^2)      atan2(L2,L3)       *see below
 Joint 3       0         0              L4             -atan2(L2,L3)       [L4/2;0;0] 
 Joint 4       0       	pi/2             0                  pi/2          [0;0;-L5/2] 
 Joint 5     L5+L6      -pi/2            0                 -pi/2           [L6/2;0;0] 
===========  ======  ==========  ===================  ===============  =================

data.man(2).b = [cos(-data.man(2).DH.theta),-sin(-data.man(2).DH.theta),0;sin(-data.man(2).DH.theta),cos(-data.man(2).DH.theta),0;0,0,1]*[L3^2/2;L2^2/2 + L3*L2;0]/(L2 + L3);

The resulting links and joints Cartesian Coordinate Systems are shown in the following image.

.. figure:: Figures/Tutorial_DH.png
   :scale: 50 %
   :align: center
   :alt: CCS of the tutorial model configuration

   CCS of the spacecraft-manipulator system.


We can then create our data structure:

.. code:: matlab
	
	%--- Manipulator Definition ----%
	%Number of joints/links
	data.n=5;

	%First joint
	data.man(1).type=0;
	data.man(1).DH.d = L1;
	data.man(1).DH.alpha = pi/2;
	data.man(1).DH.a = 0;
	data.man(1).DH.theta=0;
	data.man(1).b = [0;L1/2;0];

	%Second joint
	data.man(2).type=0;
	data.man(2).DH.d = 0;
	data.man(2).DH.alpha = 0;
	data.man(2).DH.a = sqrt(L2^2+L3^2);
	data.man(2).DH.theta=atan2(L2,L3);
	data.man(2).b = [cos(-data.man(2).DH.theta),-sin(-data.man(2).DH.theta),0;sin(-data.man(2).DH.theta),cos(-data.man(2).DH.theta),0;0,0,1]*[L3^2/2;L2^2/2 + L3*L2;0]/(L2 + L3);

	%Third joint
	data.man(3).type=0;
	data.man(3).DH.d = 0;
	data.man(3).DH.alpha = 0;
	data.man(3).DH.a =L4;
	data.man(3).DH.theta=-atan2(L2,L3);
	data.man(3).b = [L4/2;0;0];


	%Fourth joint
	data.man(4).type=0;
	data.man(4).DH.d = 0;
	data.man(4).DH.alpha = pi/2;
	data.man(4).DH.a = 0;
	data.man(4).DH.theta=pi/2;
	data.man(4).b = [0;0;-L5/2];


	%Fifth joint
	data.man(5).type=0;
	data.man(5).DH.d = L5+L6;
	data.man(5).DH.alpha =-pi/2;
	data.man(5).DH.a = 0;
	data.man(5).DH.theta=-pi/2;
data.man(5).b = [L6/2;0;0];

Once the manipulator system has been defined we can then specify the configuration of the spacecraft manipulator system as follows.

.. code:: matlab

	%Base position
	R0=eye(3);  %Rotation from base-spacecraft to the inertial frame
	r0=[0;0;0]; %Position of the base-spacecraft in the inertial frame

	%Joint variables [rad]
	qm=[0;0;0;0;0];

Then we can start calling some functions. For example the kinematic function:

.. code:: matlab

	%Kinematics
	[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

The output of the function is as follows:
	* RJ -- Joint 3x3 rotation matrices.
	* RL -- Links 3x3 rotation matrices.
	* r -- Links positions.
	* l -- Joints positions.
	* e -- Joints rotations axis.
	* g -- Vector from the origin of the ith joint to the ith link [inertial]
	* TEE -- End-Effector Homogeneous transformation matrix.

Now you can check that the r and l vectors provide the correct answers when qm=[0;0;0;0;0].

If you change the joint variables and re-run the kinematic function you will get the new positions with that particular configuration. The same can be done with the orientation R0 and position r0 of the base-spacecraft.

.. code:: matlab

	%Joint variables [rad]
	qm=[45;10;-45;20;-90]*pi/180;

	%Kinematics
	[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

You can also define the joint variables as symbolic and obtain symbolic expressions.

.. code:: matlab

	%Joint variables [rad]
	qm=sym('qm',[5,1],'real');

	%Base-spacecraft position
	r0=sym('r0',[3,1],'real');

	%Base-spacecraft orientation
	Euler_Ang=sym('Euler_Ang',[3,1],'real');
	R0 = Angles321_DCM(Euler_Ang)';

	%Kinematics
	[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

To compute the differential kinematics (including the Jacobians) can be computed if the base-spacecraft and joint velocities are specified.

.. code:: matlab

	%Velocities
	q0dot=zeros(6,1); %Base-spacecraft velocity
	qmdot=[4;-1;5;2;1]*pi/180; %Joint velocities

	%Differential Kinematics
	[t0,tm,Bij,Bi0,P0,pm]=DiffKinematics_Serial(R0,r0,q0dot,qmdot,r,l,e,g,data);
	%Jacobian of the Link 3
	[J03, Jm3]=Jacob(r(1:3,3),r0,r,P0,pm,3,data.n);
	%End-effector Jacobian
	[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,r,P0,pm,data.n,data.n);

The output of the differential kinematics as follows:
	* t0 -- Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz].
	* tm -- Manipulator twist vector [wx,wy,wz,vx,vy,vz].
	* Bij -- Twist-propagation matrix (for manipulator i>0 and j>0).
	* Bi0 -- Twist-propagation matrix (for i>0 and j=0).
	* P0 -- Base-spacecraft twist-propagation vector.
	* pm -- Manipulator twist-propagation vector.

