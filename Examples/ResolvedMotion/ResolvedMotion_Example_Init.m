%Simulator Init

%Clean and clear
clc
clear

%--- Manipulator Data ---%               
%Number of joints/links
data.n=4;

%Manipulator dimensions
man_side = 1;
man_width = 0.05;

%First joint
data.man(1).type=0;
data.man(1).DH.d = 0;
data.man(1).DH.alpha = 0;
data.man(1).DH.a = man_side;
data.man(1).DH.theta = 0;
data.man(1).b = [data.man(1).DH.a/2;0;0];
data.man(1).mass=10;
data.man(1).I=diag([0,0,data.man(1).mass/12*(man_side^2+man_width^2)]);

%Second joint
data.man(2)=data.man(1);

%Third joint
data.man(3)=data.man(1);

%Fourth joint
data.man(4)=data.man(1);

%--- Base Spacecraft ---%
%Base-spacececraft side
base_side = 0.5;

%Base-spacecraft inertia matrix
data.base.mass=100;
data.base.I=diag([0,0,data.base.mass/6*(base_side^2)]);


%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[base_side/2;0;0];zeros(1,3),1];


%--- Initial conditions ---%
q00=[0;0;0;1;zeros(3,1)];
qm0=[pi/4;-pi/4;-pi/4;pi/4];
q0dot0=zeros(6,1);
qmdot0=zeros(4,1);
q0ddot0=zeros(6,1);
qmddot0=zeros(3,1);

%Compute Initial momentum (M0) and initial matrices.
R0=quat_DCM([q00(1:4)]')';

%Kinematics
[RJ,RL,r,l,e,t0,tm,Bij,Bi0,P0,pm,TEE]=Kinematics_Serial(R0,q00(4:6),qm0,q0dot0,qmdot0,data);
%Inertias
[I0,Im]=I_I(R0,RL,data);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB_Serial(I0,Im,Bij,Bi0,data);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM_Serial(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);
M0 = H0*q0dot0+H0m*qmdot0;

