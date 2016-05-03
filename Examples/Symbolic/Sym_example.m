%This script tests the Kinematics and Dynamic functions

%--- Clean and clear ---%
clc
clear
close all

%--- Define manipulator data ---%

%Number of joints/links
data.n=2;

%Define symbolic variables
d = sym('d',[1,data.n],'real');
alpha = sym('alpha',[1,data.n],'real');
a = sym('a',[1,data.n],'real');
theta = sym('theta',[1,data.n],'real');
b = sym('b',[3,data.n],'real');
m = sym('m',[1,data.n],'real');
I = sym('I',[3,data.n],'real');
theta_EE = sym('theta_EE','real');
d_EE = sym('d_EE','real');
m_base = sym('m_base','real');
I_base = sym('I_base',[3,1],'real');

%First joint
data.man(1).type=0;
data.man(1).DH.d = d(1);
data.man(1).DH.alpha = alpha(1);
data.man(1).DH.a = a(1);
data.man(1).DH.theta = theta(1);
data.man(1).b = b(1:3,1);
data.man(1).mass=m(1);
data.man(1).I=diag(I(1:3,1));

%Second joint
data.man(2).type=0;
data.man(2).DH.d = d(2);
data.man(2).DH.alpha = alpha(2);
data.man(2).DH.a = a(2);
data.man(2).DH.theta = theta(2);
data.man(2).b = b(1:3,2);
data.man(2).mass=m(2);
data.man(2).I=diag(I(1:3,2));

%End-Effector 
data.EE.theta=theta_EE; %Rotation around z-axis
data.EE.d=d_EE; %Translation along z-axis

%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[0;0;0];zeros(1,3),1];

%Base-spacecraft inertia matrix
data.base.mass=m_base;
data.base.I=diag(I_base(1:3,1));

%Base position
R0=eye(3);
r0=[0;0;0];

%Joint variables
qm=sym('qm',[data.n,1],'real');

%Velocities
q0dot=sym('q0dot',[6,1],'real');
qmdot=sym('qmdot',[data.n,1],'real');

%Accelerations
q0ddot=sym('q0ddot',[6,1],'real');
qmddot=sym('qmddot',[data.n,1],'real');

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
[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(sym(R0),sym(r0),sym(qm),data);
%Differential Kinematics
[t0,tm,Bij,Bi0,P0,pm]=DiffKinematics_Serial(R0,r0,q0dot,qmdot,r,l,e,g,data);
%Accelerations
[t0dot,tmdot]=Accelerations_Serial(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,data);
%Inertias
[I0,Im]=I_I(R0,RL,data);
%Inverse Dynamics
[tau0,taum] = ID_Serial(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,data);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB_Serial(I0,Im,Bij,Bi0,data);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM_Serial(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = C_Serial(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,r,P0,pm,data.n,data.n);

%Stop profiling
profile viewer

return


%--- Steps that take a very long time to compute ---%

%Inverse Dynamics Floating Base (has an inverse)
[taum_floating,q0ddot_floating] = Floating_ID_Serial(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,data);
%Forward Dynamics
[q0ddot_FD,qmddot_FD] = FD_Serial(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,data);

