function [t0,tm]=Velocities(Bij,Bi0,P0,pm,q0dot,qmdot,robot) %#codegen
% Computes the velocities of the manipulator links.
%
% Input ->
%   Bij -> Twist-propagation matrix (for manipulator i>0 and j>0).
%   Bi0 -> Twist-propagation matrix (for i>0 and j=0).
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   q0dot -> Base-spacecraft velocities [angular velocity in body, linear
%   velocity in inertial].
%   qmdot -> Manipulator joint rates.
%   robot -> Robot model.
%
% Output ->
%   t0 -> Base-spacecraft twist vector [wx,wy,wz,vx,vy,vz].
%   tm -> Manipulator twist vector [wx,wy,wz,vx,vy,vz].

%=== LICENSE ===%

%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU Lesser General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU Lesser General Public License for more details.
% 
%     You should have received a copy of the GNU Lesser General Public License
%     along with this program.  If not, see <http://www.gnu.org/licenses/>.

%=== CODE ===%

%--- Number of links and Joints ---%
n=robot.n_links_joints;
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-Allocate
    tm=zeros(6,n);
end
%Base-spacecraft
t0=P0*q0dot;

%Fordward recursion to obtain the twist vector
for i=1:n
    
    if robot.joints(i).parent_link==0
        %First link
        tm(1:6,i)=Bi0(1:6,1:6,i)*t0;
    else
        %Rest of the links
        tm(1:6,i)=Bij(1:6,1:6,i,i-1)*tm(1:6,i-1);
    end
    
    %Add joint contribution
    if robot.joints(i).type~=0
        tm(1:6,i)=tm(1:6,i)+pm(1:6,i)*qmdot(robot.joints(i).q_id);
    end
    
end

end







