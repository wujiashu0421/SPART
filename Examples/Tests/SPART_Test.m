%This script runs the different SPART tests

clc
clear

disp('Testing the different functions of SPART:')

%--- Load robot model ---%
[robot,Variables] = Load_SerialRobot();

%--- Tests ---%
%Inertia matrices test
IM_Test(robot,Variables);
%Inverse Dynamics test
ID_Test(robot,Variables);
%Forward Dynamics test
FD_Test(robot,Variables);
