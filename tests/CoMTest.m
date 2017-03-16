%Test the Center-of-Mass function and also checks that the CoM does not
%move when no momentum is imparted (tau0=0).

%--- Load robot model ---%
filename='Test_URDF/ManiSat_2Arm_2_2.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%--- CoM ---%
R0=eye(3);
r0=zeros(3,1);
qm=zeros(robot.n_q,1);
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
r_com = Center_of_Mass(r0,rL,robot);

%--- Assign variables ---%
%Base position
R0=Variables.R0;
r0=Variables.r0;

%Joint variables
qm=Variables.qm;

%Velocities
qmdot=Variables.qmdot;

%Accelerations
qmddot=Variables.qmdot;

%--- Compute GIM matrix ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
H=[H0, H0m; H0m', Hm];

%--- Center-of-Mass velocity ---%
%Compute base velocity
q0dot=-H0\H0m*qmdot;
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Compute Center-of-Mass velocity
v_com = Center_of_Mass(t0(4:6),tm(4:6,:),robot);

%--- Convective inertia matrix ---%
%Velocities
q0dot=Variables.q0dot;

%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);

%--- Center-of-Mass acceleration ---%
%Base acceleration
q0ddot=-H0\(H0m*qmddot+C0*q0dot+C0m*qmdot);
%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot);
%Compute Center-of-Mass acceleration
a_com = Center_of_Mass(t0dot(4:6),tmdot(4:6,:),robot);


%% === Tests ===%

%% Center-of-Mass check
test=abs(r_com)<1e-6;
assert(all(test(:)));
%% Center-of-Mass velocity check
test=abs(v_com)<1e-6;
assert(all(test(:)));
%% Center-of-Mass acceleration check
test=abs(a_com)<1e-6;
assert(all(test(:)));