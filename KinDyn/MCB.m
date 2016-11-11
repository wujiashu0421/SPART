function [M0_tilde,Mm_tilde]=MCB(I0,Im,Bij,Bi0,robot) %#codegen
% Computes the Mass Composite Body matrix of a Serial Manipulator.
%
% Input ->
%   I0 -> Base-spacecraft inertia in inertial frame.
%   Im -> Manipulator inertia in inertial frame.
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%
% Output ->
%   M0_tilde -> Base-spacecraft mass matrix of composite body.
%   Mm_tilde -> Manipulator mass matrix of composite body.

%=== LICENSE ===%

%=== CODE ===%
%Number of links and Joints
n=robot.n_links;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate
    Mm_tilde=zeros(6,6,n);
end

%Backwards recursion
for i=n:-1:1
    %Initialize M tilde
    Mm_tilde(1:6,1:6,i)=[Im(1:3,1:3,i),zeros(3,3);zeros(3,3),robot.links(i).mass*eye(3)];%+Bij(1:6,1:6,i+1,i)'*Mm_tilde(1:6,1:6,i+1)*Bij(1:6,1:6,i+1,i);
    %Add children contributions
    for j=find(robot.Con.Child(:,i))'
        Mm_tilde(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)+Bij(1:6,1:6,j,i)'*Mm_tilde(1:6,1:6,j)*Bij(1:6,1:6,j,i);
    end
end

%Base-spacecraft M tilde
M0_tilde=[I0,zeros(3,3);zeros(3,3),robot.base_link.mass*eye(3)];
%Add children contributions
for j=find(robot.Con.Child_base)'
    Mm_tilde(1:6,1:6,i)=Mm_tilde(1:6,1:6,i)+Bi0(1:6,1:6,j)'*Mm_tilde(1:6,1:6,j)*Bi0(1:6,1:6,j);
end


end