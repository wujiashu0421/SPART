function ID_Test()
%This function test the Inverse Dynamics functions.

%Display initial messages explaining the test.
disp('Testing the Inverse Dynamics (qddot --> tau).');
disp('The test compares the results obtained by H*qddot+C*qdot');
disp('with those ones obtained trhought the dedicated inverse dynamics algorithms.');

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
wF0=Variables.wF0;
wFm=Variables.wFm;

%--- Compute Kinematic and Dynamic magnitudes ---%
%Kinematics
[RJ,RL,rJ,rL,e,g]=Kinematics(R0,r0,qm,robot);
%Differential Kinematics
[Bij,Bi0,P0,pm]=DiffKinematics(R0,r0,rL,e,g,robot);
%Velocities
[t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot);
%Accelerations
[t0dot,tmdot]=Accelerations(t0,tm,P0,pm,Bi0,Bij,q0dot,qmdot,q0ddot,qmddot,robot);
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

%--- Test Inverse Dynamics - Flying base ---%
H=[H0, H0m; H0m', Hm];
C=[C0, C0m; Cm0, Cm];
J1=[J01,Jm1];
J2=[J02,Jm2];
qddot=[q0ddot;qmddot];
qdot=[q0dot;qmdot];

%Generalized forces using H*qddot+C*qdot=tau+J'w
tau_HC = H*qddot+C*qdot-[P0'*wF0;zeros(2,1)]-J1'*wFm(1:6,1)-J2'*wFm(1:6,2);

%Generalized forces using the dedicated Inverse Dynamics
[tau0_ID,taum_ID] = ID(wF0,wFm,t0,tm,t0dot,tmdot,P0,pm,I0,Im,Bij,Bi0,robot);
tau_ID=[tau0_ID;taum_ID];

if abs(tau_HC-tau_ID)<1e-6
    disp('PASSED: Inverse Dynamics with Flying Base');
else
    warning('FAILED: Inverse Dynamics with Flying Base');
end

%--- Test Inverse Dynamics - Floating base ---%

%Compute q0ddot using H*qddot+C*qdot=tau+J'w
q0ddot_floating_HC=H0\(P0'*wF0+J01'*wFm(1:6,1)+J02'*wFm(1:6,2)-H0m*qmddot-C0*q0dot-C0m*qmdot);
%Generalized forces using H*qddot+C*qdot=tau+J'w
tau_floating_HC = H*[q0ddot_floating_HC;qmddot]+C*qdot-[P0'*wF0;zeros(2,1)]-J1'*wFm(1:6,1)-J2'*wFm(1:6,2);
taum_floating_HC=tau_floating_HC(7:8);

%Inverse Dynamics Floating Base
[taum_floating_ID,q0ddot_floating_ID] = Floating_ID(wF0,wFm,Mm_tilde,H0,t0,tm,P0,pm,I0,Im,Bij,Bi0,q0dot,qmdot,qmddot,robot);

if all(abs(taum_floating_HC-taum_floating_ID)<1e-6) && all(abs(q0ddot_floating_HC-q0ddot_floating_ID)<1e-6)
    disp('PASSED: Inverse Dynamics with Flying Base');
else
    warning('FAILED: Inverse Dynamics with Flying Base');
end

end


