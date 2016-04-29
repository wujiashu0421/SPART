% 2DOF Kinematic Manipulability Ellipse


%=== CODE ===%

%Clean and clear
clc
close all
clear

%--- Style Plot ---%
h=figure(1);
set(gca,'FontSize',16)
axis equal
hold all
grid on
box on
xlabel('x [m]')
ylabel('y [m]')
leg={};

%--- Joint States ---% 

%Joint variables
qm=deg2rad([45;-90]);

%Velocities
q0dot=[deg2rad(0);deg2rad(0);deg2rad(0);0;0;0];
qmdot=deg2rad([0;0]);

%Base position
R0=eye(3);
r0=[0;0;0];

%--- 2DOF Data 100 kg ---%
m0=100;
mi=10;
[data,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematics ---%
[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

%--- Kinematic Manipulability ---%
[elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm,qmdot,q0dot);
plot(elps_fixed(1,:)+TEE(1,4),elps_fixed(2,:)+TEE(2,4),'k','linewidth',2);
leg(end+1)={'Fixed'};
plot(elps_floating(1,:)+TEE(1,4),elps_floating(2,:)+TEE(2,4),'k:','linewidth',2);
leg(end+1)={sprintf('Floating m_{0}=%d kg',m0)};

%--- 2DOF Data 500 kg ---%
m0=500;
mi=10;
[data,base_contour,man_contour,man_contour_end]=DOF2_Data(m0,mi);

%--- Kinematics ---%
[RJ,RL,r,l,e,g,TEE]=Kinematics_Serial(R0,r0,qm,data);

%--- Kinematic Manipulability ---%
[elps_fixed,km_fixed,elps_floating,km_floating]=Kinematic_Manipulability(R0,r0,m0,mi,qm,qmdot,q0dot);
plot(elps_floating(1,:)+TEE(1,4),elps_floating(2,:)+TEE(2,4),'k--','linewidth',2);
leg(end+1)={sprintf('Floating m_{0}=%d kg',m0)};

%--- Plot Manipulator ---%
Man_Plot(R0,r0,base_contour,man_contour,man_contour_end,RL,r,data);
legend(leg,'Location','best');





