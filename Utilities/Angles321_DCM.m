function DCM = Angles321_DCM(Angles)
%Convert the Euler angles (321 sequence), x-phi, y-theta, z-psi to DCM.

phi = Angles(1);
theta = Angles(2);
psi = Angles(3);

DCM=[   cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta);
        sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), sin(phi)*cos(theta);
        cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)-sin(phi)*cos(psi), cos(phi)*cos(theta)];


end