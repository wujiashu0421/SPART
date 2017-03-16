function [robot,Variables] = Load_SerialURDFRobot(filename)
%Loads a serial robot model (using a URDF description) and generates randomly variables that are used for the SPART tests

%--- Robot model ---%
[robot,robot_keys] = urdf2robot(filename);

%--- Random Variables ---%
%Base position and orientation
Angles = -pi + (2*pi).*rand(3,1);
Variables.R0=Angles321_DCM(Angles);
Variables.r0=-1 + (2).*rand(3,1);

%Joint variables
Variables.qm=-pi/2 + (pi).*rand(robot.n_q,1);

%Velocities
Variables.q0dot=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.qmdot=-pi/2 + (pi).*rand(robot.n_q,1);

%Accelerations
Variables.q0ddot=[-pi/2 + (pi).*rand(3,1);-1 + (2).*rand(3,1)];
Variables.qmddot=-pi/2 + (pi).*rand(robot.n_q,1);

%External forces
Variables.wF0=(-1 + (2).*rand(6,1))/10;
Variables.wFm=(-1 + (2).*rand(6,robot.n_links_joints))/10;

%Joint torques
Variables.tauq0=(-1 + (2).*rand(6,1))/10;
Variables.tauqm=(-1 + (2).*rand(robot.n_q,1))/10;

end