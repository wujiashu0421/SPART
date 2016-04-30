function x_com = Center_of_Mass(r0,r,data) %#codegen
%Computes the center of mass (CoM) of the system
%
% Input ->
%   r0 -> Position of the base-spacecraft to the inertial frame.
%   r -> Links positions.
%   data -> Manipulator data.
%       data.n -> Manipulator number of joints and links.
%       data.base -> Base-spacecraft data
%           data.base.mass -> Mass [kg] of the base.
%       data.man -> Manipulator data.
%           data.man(i).mass -> Mass of the ith Link.      
% Output ->
%   x_com -> Location [m] of the center of mass (3x1 vector).

%=== LICENSE ===%

%=== CODE ===%

%Initialize total mass and total mass * distance varables
mass_total=data.base.mass;
mass_r=r0*data.base.mass;

%Add contribution of manipulator links
for i=1:data.n
    mass_total=mass_total+data.man(i).mass;
    mass_r = mass_r+r(1:3,i).*data.man(i).mass; 
end

%Compute center of mass
x_com=mass_r./mass_total;

end