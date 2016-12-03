%SPART URDF Tutorial

%--- Clean and clear ---%
clc
close all
clear

%--- URDF filename ---%
filename='kuka_lwr/kuka.urdf';
%filename='kuka_iiwa/kuka_iiwa.urdf';

%--- Create robot model ---%
[robot,robot_keys] = urdf2robot(filename);

%--- Parameters ---%
R0=eye(3);
r0=zeros(3,1);
qm=zeros(robot.n_q,1);
q0dot=zeros(6,1);
qmdot=zeros(robot.n_links,1);

%--- Kinematics ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Diferential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Jacobian of the last link
[J0n, Jmn]=Jacob(rL(1:3,end),r0,rL,P0,pm,robot.n_links,robot);

%--- Dynamics ---%
%Inertias in inertial frames
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = C(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Forward Dynamics ---%

%Gravity
g=9.8; %[m s-2]

%External forces (includes gravity and assumes z is the vertical direction)
wF0=zeros(6,1);
wFm=zeros(6,robot.n_links);
for i=1:robot.n_links
    wFm(1:6,6)=-robot.links(i).mass*g;
end

%Joint torques
tauq0=zeros(6,1);
tauqm=zeros(robot.n_links,1);

%Forward Dynamics
[q0ddot_FD,qmddot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,robot);

%--- Inverse Dynamics - Flying ---%

%Accelerations
q0ddot=zeros(6,1);
qmddot=zeros(robot.n_links,1);

%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot);

%Inverse Dynamics - Flying base
[tau0,taum] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);

%--- Forward Dynamics - Floating ---%

%Accelerations
qmddot=zeros(robot.n_links,1);

%Inverse Dynamics - Floating Base
[taum_floating,q0ddot_floating] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,robot);