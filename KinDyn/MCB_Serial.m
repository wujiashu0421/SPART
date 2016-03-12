function [M0_tilde,Mm_tilde]=MCB_Serial(I0,Im,Bij,Bi0,data)
% Computes the Mass Composite Body matrix of a Serial Manipulator.
%
% Input ->
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%       data.base -> Base-spacecraft data
%           data.base.mass -> Base-spacecraft mass.
%       data.man -> Manipulator data.
%           data.man.n -> Manipulator number of joints and links.
%           data.man(i).mass -> Link mass.
% Output ->
%   M0_tilde -> Base-spacecraft mass matrix of composite body.
%   Mm_tilde -> Manipulator mass matrix of composite body.

%=== LICENSE ===%

%=== CODE ===%
%Number of links and Joints
n=data.n;
%Pre-allocate
Mm_tilde=zeros(6,6,n);
%Initialize M tilde
Mm_tilde(1:6,1:6,n)=[Im(1:3,1:3,n),zeros(3,3);zeros(3,3),data.man(n).mass*eye(3)];
%Backwards recursion 
for i=n-1:-1:1
    Mm_tilde(1:6,1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),data.man(i).mass*eye(3)]+Bij(1:6,1:6,i+1,i)'*Mm_tilde(1:6,1:6,i+1)*Bij(1:6,1:6,i+1,i);
end
%Base-spacecraft M tilde
M0_tilde=[I0,zeros(3,3);zeros(3,3),data.base.mass*eye(3)]+Bi0(1:6,1:6,1)'*Mm_tilde(1:6,1:6,1)*Bi0(1:6,1:6,1);

end