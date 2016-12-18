%This script runs the different SPART tests

clc
clear

disp('Testing the different functions of SPART:')

%Inertia matrices test
IM_Test()
%Inverse Dynamics test
ID_Test();
%Forward Dynamics test
FD_Test();
