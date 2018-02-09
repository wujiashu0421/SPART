function [DCM]=RP_DCM(s)
% Computes the DCM matrix from the Rodrigues Parameters
%
% Formula from ANALYTICAL MECHANICS OF SPACE SYSTEMS, Hanspeter Schaub and John L. Junkins
% 
% Inputs:
%   s -> Rodrigues Parameters
%
% Outputs:
%   DCM -> Direction Cosine Matrix

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

DCM = 1/(1+s'*s)*((1-s'*s)*eye(3)+2*s*s'-2*SkewSym(s));


end