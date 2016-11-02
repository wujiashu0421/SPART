function DCM = Euler_DCM(e,alpha)
%Provides the Direction Cosine Matrix (DCM) from a Euler axis e=[e1,e2,e3]
%and angle alpha.

%Create quaternion
q=[ e(1)*sin(alpha/2);
    e(2)*sin(alpha/2);
    e(3)*sin(alpha/2);
    cos(alpha/2)];

%Convert quaternion to DCM
DCM = quat_DCM(q);

end