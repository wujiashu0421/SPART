%SPART Tutorial

%--- Clean and clear ---%
clc
close all
clear

%--- Dimensions [mm] ---%
L0=50;
L1=125;
L2=144;
L3=47;
L4=142;
L5=80;
L6=70;

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
data.man(1).mass=2;
data.man(1).I=diag([2,1,3])/10;

%Second joint
data.man(2).type=0;
data.man(2).DH.d = 0;
data.man(2).DH.alpha = 0;
data.man(2).DH.a = sqrt(L2^2+L3^2);
data.man(2).DH.theta=atan2(L2,L3);
data.man(2).b = [cos(-data.man(2).DH.theta),-sin(-data.man(2).DH.theta),0;sin(-data.man(2).DH.theta),cos(-data.man(2).DH.theta),0;0,0,1]*[L3^2/2;L2^2/2 + L3*L2;0]/(L2 + L3);
data.man(2).mass=2;
data.man(2).I=diag([2,1,3])/10;

%Third joint
data.man(3).type=0;
data.man(3).DH.d = 0;
data.man(3).DH.alpha = 0;
data.man(3).DH.a =L4;
data.man(3).DH.theta=-atan2(L2,L3);
data.man(3).b = [L4/2;0;0];
data.man(3).mass=2;
data.man(3).I=diag([2,1,3])/10;

%Fourth joint
data.man(4).type=0;
data.man(4).DH.d = 0;
data.man(4).DH.alpha = pi/2;
data.man(4).DH.a = 0;
data.man(4).DH.theta=pi/2;
data.man(4).b = [0;0;-L5/2];
data.man(4).mass=2;
data.man(4).I=diag([2,1,3])/10;

%Fifth joint
data.man(5).type=0;
data.man(5).DH.d = L5+L6;
data.man(5).DH.alpha =-pi/2;
data.man(5).DH.a = 0;
data.man(5).DH.theta=-pi/2;
data.man(5).b = [L6/2;0;0];
data.man(5).mass=2;
data.man(5).I=diag([2,1,3])/10;

%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[0;0;L0];zeros(1,3),1];

%Base-spacecraft mass and inertia
data.base.mass=20;
data.base.I=diag([2,1,3]);

%End-Effector
data.EE.theta=-pi/2;
data.EE.d=0;

%Base position
R0=eye(3);  %Rotation from Base-spacecraft to inertial
r0=[0;0;0]; %Position of the base-spacecraft

%Joint variables [rad]
qm=[0;0;0;0;0];

%Velocities
q0dot=zeros(6,1); %Base-spacecraft velocity
qmdot=[4;-1;5;2;1]*pi/180; %Joint velocities


%--- Kinematics ---%
[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

%--- Differential Kinematics ---%
%Differential kinematics
[t0,tm,Bij,Bi0,P0,pm]=DiffKinematics_Serial(R0,r0,q0dot,qmdot,r,l,e,g,data);
%Jacobian of the Link 3
[J03, Jm3]=Jacob(r(1:3,3),r0,r,P0,pm,3,data.n);
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,r,P0,pm,data.n,data.n);

%--- Inertia Matrices ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,data);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB_Serial(I0,Im,Bij,Bi0,data);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM_Serial(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = C_Serial(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);

%--- Forward Dynamics ---%

%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=[0;0;0;0;0;-data.base.mass*g];
wFm=[zeros(5,data.n);
    -data.man(1).mass*g,-data.man(2).mass*g,-data.man(3).mass*g,-data.man(4).mass*g,-data.man(5).mass*g];

%Joint torques
tauq0=zeros(6,1);
tauqm=zeros(data.n,1);

%Forward Dynamics
[q0ddot_FD,qmddot_FD] = FD_Serial(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,data);

%--- Inverse Dynamics - Flying ---%

%Accelerations
q0ddot=zeros(6,1);
qmddot=zeros(5,1);

%Accelerations
[t0dot,tmdot]=Accelerations_Serial(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,data);

%Inverse Dynamics - Flying base
[tau0,taum] = ID_Serial(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,data);

%--- Forward Dynamics - Floating ---%

%Accelerations
qmddot=zeros(5,1);

%Inverse Dynamics - Floating Base
[taum_floating,q0ddot_floating] = Floating_ID_Serial(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,data);