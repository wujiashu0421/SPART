function [RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data)
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
%   g -> Vector from the origin of the ith joint to the ith link [inertial]
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
%Add data.EE.theta rotation (around the z-axis)
TEE(1:3,1:3)=TEE(1:3,1:3)*[ cos(data.EE.theta),-sin(data.EE.theta),0;
                            sin(data.EE.theta),cos(data.EE.theta),0;
                            0,0,1];
%Add data.EE.d translation (along the z-axis)
TEE(3,4)=TEE(3,4)+data.EE.d;
%Recompute last link homogeneous transformation matrix (as we have just
%modified the End-Effector transformation matrix).
TL(1:4,1:4,n)=TEE*[eye(3),-data.man(n).b; zeros(1,3), 1];

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

end







