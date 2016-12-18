function [J0dot, Jmdot]=Jacobdot(rxi,txi,r0,t0,rL,tL,P0,pm,i,robot) %#codegen
% Computes the Jacobian time derivative of the xi point.
%
% Input ->
%   rxi -> Inertial position of the point of interest.
%   txi -> Inertial twist of the point of interest.
%   r0 -> Inertial position of the base-spacecraft.
%   t0 -> Inertial twist of the base-spacecraft.
%   rL -> Links inertial positions.
%   tL -> Inertial twist of the links.
%   P0 -> Base-spacecraft twist-propagation vector.
%   pm -> Manipulator twist-propagation vector.
%   i -> Link where the point xi is located.
%   robot -> Robot model.
%
% Output ->
%   J0dot -> Base-spacecraft Jacobian time derivative
%   Jmdot -> Manipulator Jacobian time derivative

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

%--- Omega ---%
%Base-spacecraft Omega
Omega0=[SkewSym(t0(1:3)), zeros(3,3);
    zeros(3,3), SkewSym(t0(1:3))];
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    %Pre-allocate Omega
    Omega=zeros(6,6,n);
end
%Compute Omega
for j=1:i
    Omega(1:6,1:6,j)=[SkewSym(tL(1:3,j)), zeros(3,3);
        zeros(3,3), SkewSym(tL(1:3,j))];
end


%--- Jacobians time derivative ---%

%Base Jacobian
J0dot=[eye(3),zeros(3,3);SkewSym(r0-rxi),eye(3)]*Omega0*P0+[zeros(3,3),zeros(3,3);SkewSym(t0(4:6)-txi(4:6)),zeros(3,3)]*P0;

%Pre-allocate
if not(isempty(coder.target)) %Only use during code generation (allowing symbolic computations)
    Jmdot=zeros(6,robot.n_q);
end
%Manipulator Jacobian
for j=1:i
    %If joint is not fixed
    if robot.joints(j).type~=0
        if robot.con.branch(i,j)==1
            Jmdot(1:6,robot.joints(j).q_id)=[eye(3),zeros(3,3);SkewSym(rL(1:3,j)-rxi),eye(3)]*Omega(1:6,1:6,j)*pm(1:6,j)+[zeros(3,3),zeros(3,3);SkewSym(tL(4:6,j)-txi(4:6)),zeros(3,3)]*pm(1:6,j);
        else
            Jmdot(1:6,robot.joints(j).q_id)=zeros(6,1);
        end
    end
end

%Add zeros if required
if isempty(coder.target) %Only when not pre-allocated
    if i<robot.n_q
        Jmdot(1:6,i+1:robot.n_q)=zeros(6,robot.n_q-i);
    end
end

end