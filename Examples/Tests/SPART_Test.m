%This script runs the different SPART tests

clc
clear

disp('SPART - Serial DH Test:')

%--- Load robot model ---%
[robot,Variables] = Load_SerialDHRobot();

%--- Tests ---%
%Inertia matrices test
IM_Test(robot,Variables);
%Inverse Dynamics test
ID_Test(robot,Variables);
%Forward Dynamics test
FD_Test(robot,Variables);

fprintf('\n');
disp('SPART - Serial URDF Test:')

%--- Load robot model ---%
filename='../URDF_Tutorial/kuka_iiwa/kuka_iiwa.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%--- Tests ---%
%Inertia matrices test
IM_Test(robot,Variables);
%Inverse Dynamics test
ID_Test(robot,Variables);
%Forward Dynamics test
FD_Test(robot,Variables);


fprintf('\n');
disp('SPART - Multibranch (from base) URDF Test:')

%--- Load robot model ---%
filename='../URDF_Tutorial/Multi_kuka_iiwa/Two_iiwa_Base.urdf';
[robot,Variables] = Load_SerialURDFRobot(filename);

%--- Tests ---%
%Inertia matrices test
IM_Test(robot,Variables);
%Inverse Dynamics test
ID_Test(robot,Variables);
%Forward Dynamics test
FD_Test(robot,Variables);


