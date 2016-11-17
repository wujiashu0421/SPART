%This script tests the Kinematics and Dynamic functions

%--- Clean and clear ---%
clc
clear
close all

%--- Define manipulator data ---%

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

%--- Variables ---%
%Base position
R0=eye(3);
r0=[0;0;0];

%Joint variables
qm=deg2rad([25;25]);

%Velocities
q0dot=[deg2rad(10);deg2rad(5);deg2rad(1);1;2;3];
qmdot=deg2rad([40;55]);

%Accelerations
q0ddot=[deg2rad(0);deg2rad(5);deg2rad(1);1;2;3];
qmddot=deg2rad([40;55]);

%External forces
wF0=zeros(6,1);
wFm=zeros(6,data.n);

%Joint torques
tauq0=zeros(6,1);
tauqm=[0;0];

%--- Compute Kinematics, Dynamics, ID, and FD ---%

%Start profiling
profile on

%Kinematics
[RB,RJ,RL,rB,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot);
%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Inverse Dynamics
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Inverse Dynamics Floating Base
[taum_floating,q0ddot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,robot);
%Forward Dynamics
[q0ddot_FD,qmddot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = C(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%End-effector Jacobian
[J0EE, JmEE]=Jacob(rL(1:3,end),r0,rL,P0,pm,robot.n_q,robot);

%Stop profiling
profile viewer



