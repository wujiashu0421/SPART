function FD_Test()
%This function test the Forward Dynamics functions.

%Display initial messages explaining the test.
disp('Testing the Forward Dynamics (tau --> qddot).');
disp('The test compares the results obtained by H*qddot+C*qdot');
disp('with those ones obtained trhought the dedicated forward dynamics algorithms.');

%--- Load robot model ---%
[robot,Variables] = Load_SerialRobot();

%--- Assign variables ---%
%Base position
R0=Variables.R0;
r0=Variables.r0;

%Joint variables
qm=Variables.qm;

%Velocities
q0dot=Variables.q0dot;
qmdot=Variables.qmdot;

%Accelerations
q0ddot=Variables.q0ddot;
qmddot=Variables.qmdot;

%External forces
wF0=zeros(6,1);%Variables.wF0;
wFm=zeros(6,2);Variables.wFm;

%Joint torques
tauq0=Variables.tauq0;
tauqm=Variables.tauqm;


%--- Compute Kinematic and Dynamic magnitudes ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Inertias
[I0,Im]=I_I(R0,RL,robot);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Generalized Convective Inertia matrix
[C0, C0m, Cm0, Cm] = CIM(t0,tm,I0,Im,M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);
%Jacobians
[J01, Jm1]=Jacob(rL(1:3,1),r0,rL,P0,pm,1,robot);
[J02, Jm2]=Jacob(rL(1:3,2),r0,rL,P0,pm,2,robot);

%--- Test Forward Dynamics ---%
H=[H0, H0m; H0m', Hm];
C=[C0, C0m; Cm0, Cm];
J1=[J01,Jm1];
J2=[J02,Jm2];
qdot=[q0dot;qmdot];
tauq=[tauq0;tauqm];

%Compute qddot using H*qddot+C*qdot=tau+J'w
qddot_HC=H\(tauq+[P0'*wF0;zeros(2,1)]+J1'*wFm(1:6,1)+J2'*wFm(1:6,2)-C*qdot);

%Forward Dynamics
[q0ddot_FD,qmddot_FD] = FD(tauq0,tauqm,wF0,wFm,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,robot);
qddot_FD=[q0ddot_FD;qmddot_FD];

if abs(qddot_HC-qddot_FD)<1e-6
    disp('PASSED: Forward Dynamics');
else
    warning('FAILED: Forward Dynamics');
end

end



