function [omegadot]=RPddot_omegadot(s,sdot,sddot)
% Computes the angular acceleration projected in the body frame from the
% Rodrigues Parameters and their time derivatives.
%
% Formula from ANALYTICAL MECHANICS OF SPACE SYSTEMS, Hanspeter Schaub and John L. Junkins
% 
% Inputs:
%   s -> Rodrigues Parameters
%   sdot -> Rodrigues Parameters time dervative
%   sddot -> Rodrigues Parameters double time dervative
%
% Outputs:
%   omegadot -> Angular acceleration projected in the body axis

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

%Intermediate variables
B = 2/(1+s'*s)*(eye(3)-SkewSym(s));
Bdot = 4*s'*sdot/(1+s'*s)^2-2*SkewSym(sdot)/(1+s'*s);

%Acceleration
omegadot = Bdot*sdot+B*sddot;


end