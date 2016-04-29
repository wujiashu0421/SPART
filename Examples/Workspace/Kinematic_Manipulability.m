function [elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm,qmdot,q0dot)
%Function that computes the kinematic manipulability


%--- 2DOF Data ---%
[data,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematics ---%
%Kinematics
[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);
%Differential Kinematics
[t0,tm,Bij,Bi0,P0,pm]=DiffKinematics_Serial(R0,r0,q0dot,qmdot,r,l,e,g,data);

%--- End-Effector Jacobian ---%
%End-effector Jacobian
[J0EE, JmEE]=Jacob(TEE(1:3,4),r0,r,P0,pm,data.n,data.n);

%--- Generalized Inertia Matrices ---%

%Inertias
[I0,Im]=I_I(R0,RL,data);
%Mass Composite Body matrix
[M0_tilde,Mm_tilde]=MCB_Serial(I0,Im,Bij,Bi0,data);
%Generalized Inertia matrix
[H0, H0m, Hm] = GIM_Serial(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,data);

%--- Floating Jacobian ---%
JEE_star = JmEE-J0EE*(H0\H0m);

%--- Manipulability ---%

%Fixed base kinematic manipullabillity
[V,D] = eig(JmEE(4:5,:)*JmEE(4:5,:)');
[ex,ey]=ellipse(D(1,1),D(2,2),0,0,100);
elps_fixed=V*[ex;ey];
%Measure
km_fixed=sqrt(det(JmEE(4:5,:)*JmEE(4:5,:)'));

try
%Floating base kinematic manipullabillity
[V,D] = eig(JEE_star(4:5,:)*JEE_star(4:5,:)');
[ex,ey]=ellipse(D(1,1),D(2,2),0,0,100);
elps_floating=V*[ex;ey];
%Measure
km_floating=sqrt(det(JEE_star(4:5,:)*JEE_star(4:5,:)'));
catch
    elps_floating=[0,0];
    km_floating=0;
end
    