function [RJ,RL,r,l,e,t0,tm,Bij,Bi0,P0,pm,TEE]=Kinematics_Serial(R0,r0,qm,q0dot,qmdot,data)
% Computes the kineamtics of a serial manipulator.
%
% Input ->
%   R0 -> Rotation matrix from the base-spacecraft to the inertial frame.
%   r0 -> Position of the base-spacecraft to the inertial frame.
%   qm -> Manipulator joint varibles.
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%       data.base -> Base-spacecraft data
%           data.base.T_L0_J1 -> Homogeneous transformation of the first
%           joint w.r.t. the base-spacecraft.
%       data.man -> Manipulator data.
%           data.man(i).DH -> DH parameters of the ith joint.
%           data.man(i).type -> Type of joint. type==0 for revolute,
%           otherwise prismatic.
%           data.man(i).b -> Vector from the ith link to the following
%           joint i+1.
%       data.EE -> End-effector parameters
%           data.EE.theta -> Rotation about the final z-axis so that any
%           desired End-Effector cartesian coordinate system orientation
%           can be achieved.
%
% Output ->
%   RJ -> Joint 3x3 rotation matrices.
%   RL -> Links 3x3 rotation matrices.
%   r -> Links positions.
%   l -> Joints positions.
%   e -> Joints rotations axis.
%   t0 -> Base-spacecraft twist vector
%   tm -> Manipulator twist vector.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   TEE -> End-Effector Homogeneous transformation matrix.

%=== LICENSE ===%

%=== CODE ===%

%--- Number of links and Joints ---%
n=data.n;

%--- Homogeneous transformation matrices ---%
%Pre-allocate homogeneous transformations matrices
TJ=zeros(4,4,n+1);
TL=zeros(4,4,n);
%Base-spacecraft
T0 = [R0,r0;zeros(1,3),1];
%First Joint
TJ(1:4,1:4,1) =T0*data.base.T_L0_J1;
%Fordward recursive for rest of joints and links 
for i=1:n
    %Compute Rotation matrix and translation vector from DH parameters
    [R,s] = DH_Rs(data.man(i).DH,qm(i),data.man(i).type);
    %Compute joint homogeneous transformation matrix
    TJ(1:4,1:4,i+1)=TJ(1:4,1:4,i)*[R,s;zeros(1,3),1];
    %Compute link homogeneous transformation matrix
    TL(1:4,1:4,i)=TJ(1:4,1:4,i+1)*[eye(3),-data.man(i).b; zeros(1,3), 1];
end
%End-Effector
TEE = TJ(1:4,1:4,n+1);
TEE(1:3,1:3)=TEE(1:3,1:3)*[ cos(data.EE.theta),-sin(data.EE.theta),0;
                            sin(data.EE.theta),cos(data.EE.theta),0;
                            0,0,1];

%--- Rotation matrices, translation, position and other geometry vectors ---%
%Pre-allocate rotation matrices, translation and position vectors
RJ=zeros(3,3,n);
RL=zeros(3,3,n);
r=zeros(3,n);
l=zeros(3,n);
%Pre-allocate axis
e=zeros(3,n);
%Pre-allocate other gemotery vectors
g=zeros(3,n);
%Format Rotation matrices, link positions, joint axis and other geometry
%vectors
for i=1:n
    RJ(1:3,1:3,i)=TJ(1:3,1:3,i);
    RL(1:3,1:3,i)=TL(1:3,1:3,i);
    r(1:3,i)=TL(1:3,4,i);
    e(1:3,i)=RJ(1:3,3,i);
    l(1:3,i)=TJ(1:3,4,i);
    g(1:3,i)=r(1:3,i)-l(1:3,i);
end

%--- Twist-propagtaion matrix ---%
%Pre-allocate Bij
Bij=zeros(6,6,n,n);
%Compute Bij
for j=1:n
    for i=1:n
        Bij(1:6,1:6,i,j)=[eye(3), zeros(3,3); SkewSym(r(1:3,j)-r(1:3,i)), eye(3)];
    end
end
%Pre-allocate Bi0
Bi0=zeros(6,6,n);
%Compute Bi0
for i=1:n
    Bi0(1:6,1:6,i)=[eye(3), zeros(3,3); SkewSym(r0-r(1:3,i)), eye(3)];
end

%--- Twist-Propagation vector ---%
%Pre-allocate
pm=zeros(6,n);
%Base-spacecraft
P0=[R0,zeros(3,3); zeros(3,3), eye(3)];
%Fordward recursion to obtain the Twist-Propagation vector
for i=1:n
    if data.man(i).type==0
        %Revolute joint
        pm(1:6,i)=[e(1:3,i);cross(e(1:3,i),g(1:3,i))];
    else
        %Prismatic joint
        pm(1:6,i)=[zeros(3,1);e(1:3,i)];
    end
end

%--- Generalized twist vector ---%
%Pre-Allocate
tm=zeros(6,n);
%Base-spacecraft
t0=P0*q0dot;
%First link
tm(1:6,1)=Bi0(1:6,1:6,1)*t0+pm(1:6,1)*qmdot(1);
%Fordward recursion to obtain the twist vector
for i=2:n
    tm(1:6,i)=Bij(1:6,1:6,i,i-1)*tm(1:6,i-1)+pm(1:6,i)*qmdot(i); 
end

end







