function [robot,Variables] = Load_SerialRobot()
%Loads a random serial robot model and generates randomly variables that are used for the SPART tests

%Number of joints/links
data.n=2;

%First joint
data.man(1).type=1;
data.man(1).DH.d = 0;
data.man(1).DH.alpha = 0;
data.man(1).DH.a = 1;
data.man(1).DH.theta = 0;
data.man(1).b = [data.man(1).DH.a/2;0;0];
data.man(1).mass=2;
data.man(1).I=eye(3)/10;

%Second joint
data.man(2).type=1;
data.man(2).DH.d = 0;
data.man(2).DH.alpha = 0;
data.man(2).DH.a = 1;
data.man(2).DH.theta = 0;
data.man(2).b = [data.man(2).DH.a/2;0;0];
data.man(2).mass=2;
data.man(2).I=eye(3)/10;

%End-Effector 
data.EE.theta=0; %Rotation around z-axis
data.EE.d=0; %Translation along z-axis

%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[1;0;0];zeros(1,3),1];

%Base-spacecraft inertia matrix
data.base.mass=10;
data.base.I=eye(3);

%--- Compute Robot Model ---%
[robot] = DH_Serial2robot(data);

%--- Random Variables ---%
%Base position and orientation
Angles = -pi + (2*pi).*rand(3,1);
Variables.R0=Angles321_DCM(Angles);
Variables.r0=-1 + (2).*rand(3,1);

%Joint variables
Variables.qm=-pi/2 + (pi).*rand(2,1);

%Velocities
Variables.q0dot=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.qmdot=-pi/2 + (pi).*rand(2,1);

%Accelerations
Variables.q0ddot=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.qmddot=-pi/2 + (pi).*rand(2,1);

%External forces
Variables.wF0=(-1 + (2).*rand(6,1))/10;
Variables.wFm=(-1 + (2).*rand(6,2))/10;

%Joint torques
Variables.tauq0=(-1 + (2).*rand(6,1))/10;
Variables.tauqm=(-1 + (2).*rand(2,1))/10;

end