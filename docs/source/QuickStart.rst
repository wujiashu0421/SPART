Quickstart
==========


SPART is a collection of MATLAB and Simulink functions/blocks. The Simulink blocks are contained into the SimulinkLibrary folder. The MATLAB functions are contained into different folders:

* KinDyn - contains the general Kinematics and Dynamics.
* Utils - contains conversions between different orientation parametrization.

Most of these functions require a description of the spacecraft-manipulator system. This is provided using a MATLAB structure. Here is an example of how this looks like for a 2-link manipulator system. 

.. code:: matlab
	
	%--- Define manipulator data ---%

	%Number of joints/links
	data.n=2;

	%First joint
	data.man(1).type=0;
	data.man(1).DH.d = 0;
	data.man(1).DH.alpha = 0;
	data.man(1).DH.a = 1;
	data.man(1).b = [data.man(1).DH.a/2;0;0];
	data.man(1).mass=2;
	data.man(1).I=eye(3)/10;

	%Second joint
	data.man(2).type=0;
	data.man(2).DH.d = 0;
	data.man(2).DH.alpha = 0;
	data.man(2).DH.a = 1;
	data.man(2).b = [data.man(2).DH.a/2;0;0];
	data.man(2).mass=2;
	data.man(2).I=eye(3)/10;

	%Firts joint location with respect to base
	data.base.T_L0_J1=[eye(3),[1;0;0];zeros(1,3),1];

	%Base-spacecraft inertia matrix
	data.base.mass=10;
	data.base.I=eye(3);