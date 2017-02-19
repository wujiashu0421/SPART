%Adds SPART to the path and save it

%Find path of repository
pathstr = fileparts(mfilename('fullpath'));

%Add all the required folders into the path
addpath(sprintf('%s/KinDyn',pathstr));
addpath(sprintf('%s/SimulinkLibrary',pathstr));
addpath(sprintf('%s/Utilities',pathstr));
addpath(sprintf('%s/Utilities/Transformations',pathstr));
addpath(sprintf('%s/Utilities/RobotModel',pathstr));
addpath(sprintf('%s/Utilities/URDF_Models',pathstr));

%Save path
savepath;

