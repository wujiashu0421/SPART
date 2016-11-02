function [robot] = urdf2robot(filename) %#codegen
%Creates the SPART robot model from an URDF file.

%This function was inspired by:
%https://github.com/jhu-lcsr/matlab_urdf/blob/master/load_ne_id.m


%--- CODE ---%

%Create Robot structure.
robot=struct();

%Read URDF file
urdf=xmlread(filename);

%Only one robot description per URDF file is allowed
robot_urdf = urdf.getElementsByTagName('robot');
if robot_urdf.getLength() ~= 1
    error('URDF contains more than one robot. This function only accepts a single robot descriptioon per URDF');
end
robot_urdf =robot_urdf.item(0);

%Get robot name
robot.name = char(robot_urdf.getAttribute('name'));

%Get links and joints from URDF file
links_urdf = robot_urdf.getElementsByTagName('link');
joints_urdf = robot_urdf.getElementsByTagName('joint');

%Find and remove links elements that are not directly under the robot element
i=0;
while i<links_urdf.getLength
    if ~strcmp(links_urdf.item(i).getParentNode.getNodeName(),'robot')
        links_urdf.item(i).getParentNode.removeChild(links_urdf.item(i));
    else
        i=i+1;
    end
end

%Find and remove joint elements that are not directly under the robot element
i=0;
while i<joints_urdf.getLength
    if ~strcmp(joints_urdf.item(i).getParentNode.getNodeName(),'robot')
        joints_urdf.item(i).getParentNode.removeChild(joints_urdf.item(i));
    else
        i=i+1;
    end
end

%Count links and joints
robot.n_links = links_urdf.getLength;
robot.n_joints = joints_urdf.getLength;

%Create temporary link and joint maps
links = containers.Map();
joints = containers.Map();

%Display data
fprintf('Number of links: %d (including base links)\n', robot.n_links);
fprintf('Number of joints: %d (including fixed joints)\n',robot.n_joints);

%Iterate over links
for k = 0:robot.n_links-1
    %Create basic structure with default values
    link = struct();
    link.xml = links_urdf.item(k);
    link.name = char(link.xml.getAttribute('name'));
    link.s=zeros(3,1);
    link.R=eye(3);
    link.parent_joint = {};
    link.child_joint = {};
    
    %Grab inertial properties
    inertial = link.xml.getElementsByTagName('inertial').item(0);
    
    %Grab origin properties
    origin = inertial.getElementsByTagName('origin').item(0);
    if ~isempty(origin)
        if ~isempty(char(origin.getAttribute('xyz')))
            link.s = eval(['[',char(origin.getAttribute('xyz')),']'])';
        end
        if ~isempty(char(origin.getAttribute('rpy')))
            rpy = eval(['[',char(origin.getAttribute('rpy')),']']);
            link.R=Angles123_DCM(rpy');
        end
    end
    
    %Mass
    mass = inertial.getElementsByTagName('mass').item(0);
    link.mass = eval(char(mass.getAttribute('value')));
    
    %Inertia
    inertia = inertial.getElementsByTagName('inertia').item(0);
    ixx = eval(inertia.getAttribute('ixx'));
    iyy = eval(inertia.getAttribute('iyy'));
    izz = eval(inertia.getAttribute('izz'));
    ixy = eval(inertia.getAttribute('ixy'));
    iyz = eval(inertia.getAttribute('iyz'));
    ixz = eval(inertia.getAttribute('ixz'));
    link.inertia = [ixx, ixy, ixz; ixy, iyy, iyz; ixz, iyz, izz];
    
    %Store this link in the links map
    links(char(link.name))=link;
end

%Iterate over joints
for k = 0:robot.n_joints-1  
    %Create basic structure with default values
    joint = struct();
    joint.xml = joints_urdf.item(k);
    joint.name = char(joint.xml.getAttribute('name'));
    joint.type = char(joint.xml.getAttribute('type'));
    joint.parent_link = '';
    joint.child_link = '';
    joint.s =zeros(3,1);
    joint.R =eye(3);
    joint.axis = [0; 0; 1];
    
    %Get origin properties
    origin = joint.xml.getElementsByTagName('origin').item(0);
    if ~isempty(origin)
        if ~isempty(char(origin.getAttribute('xyz')))
            joint.s = eval(['[',char(origin.getAttribute('xyz')),']'])';
        end
        if ~isempty(char(origin.getAttribute('rpy')))
            
            rpy = eval(['[',char(origin.getAttribute('rpy')),']']);
            joint.R= Angles123_DCM(rpy');
        end
    end
    
    %Get rotation axis
    axis = joint.xml.getElementsByTagName('axis').item(0);
    if ~isempty(axis)
        joint.axis = eval(['[',char(axis.getAttribute('xyz')),']'])';
    end
    
    %Get parent link name
    parent = joint.xml.getElementsByTagName('parent').item(0);
    if ~isempty(parent)
        joint.parent_link = char(parent.getAttribute('link'));
        
        %Store the joint name in the parent link
        parent=links(joint.parent_link);
        parent.child_joint(end+1) = {joint.name};
        links(joint.parent_link) = parent;
    end
    
    %Get child link name
    child = joint.xml.getElementsByTagName('child').item(0);
    if ~isempty(child)
        joint.child_link = char(child.getAttribute('link'));
        
        %Store the joint name in the child link
        child =links(joint.child_link);
        child.parent_joint(end+1) = {joint.name};
        links(joint.child_link) = child;
    end
    
    %Store this joint in the joints map
    joints(char(joint.name))=joint;
end

% Find the base link
for link_name = links.keys
    if isempty(links(char(link_name)).parent_joint)
        base_link = char(link_name);
        fprintf('Base link: %s\n',base_link);
    end
end
%There needs to be a root link
if ~exist('base_link','var')
    error('Robot has no single base link!');
end

%Structure links and joints map into a structure and create a map with
%names and IDs.

%Create ID maps
robot.link_id=containers.Map();
robot.joint_id=containers.Map();

%Remove base link from the number of total links
robot.n_links=robot.n_links-1;

%Create links and joints stucture
robot.links(robot.n_links) = struct();
robot.joints(robot.n_joints) = struct();

%Save base link on its own structure
clink=links(base_link);
robot.base_link.xml=clink.xml;
robot.base_link.name=clink.name;
robot.base_link.child_joint=[];
robot.base_link.R=clink.R;
robot.base_link.s=clink.s;
robot.base_link.mass=clink.mass;
robot.base_link.inertia=clink.inertia;

%Assign base ID
robot.link_id(base_link)=0;

%Add links and joints into the structure with the standard numbering
nl=-1; %Link index
nj=-1; %Joint index
%Recursively scan through the tree structure
for n=1:length(clink.child_joint)
    robot.base_link.child_joint(end+1)=nj+2;
    [robot,nl,nj]=urdf2robot_recursive(robot,links,joints,joints(clink.child_joint{n}),nl+1,nj+1);
end

end

%--- Recursive function ---%
function [robot,nl,nj]=urdf2robot_recursive(robot,links,joints,child_joint,nl,nj)%#codegen

%Copy the elements of child joint
robot.joints(nj+1).id=nj+1;
robot.joints(nj+1).xml=child_joint.xml;
robot.joints(nj+1).name=child_joint.name;
robot.joints(nj+1).type=child_joint.type;
robot.joints(nj+1).parent_link=robot.link_id(child_joint.parent_link);
robot.joints(nj+1).child_link=nl+1;
robot.joints(nj+1).axis=child_joint.axis;
robot.joints(nj+1).R=child_joint.R;
robot.joints(nj+1).s=child_joint.s;

%Copy elements of child link
clink=links(child_joint.child_link);
robot.links(nl+1).id=nl+1;
robot.links(nl+1).xml=clink.xml;
robot.links(nl+1).name=clink.name;
robot.links(nl+1).parent_joint=nj+1;
robot.links(nl+1).child_joint=[];
robot.links(nl+1).R=clink.R;
robot.links(nl+1).s=clink.s;
robot.links(nl+1).mass=clink.mass;
robot.links(nl+1).inertia=clink.inertia;

%Assign ID
robot.joint_id(child_joint.name)=nj+1;
robot.link_id(clink.name)=nl+1;

%Recursively scan through the tree structure
for n=1:length(clink.child_joint)
    robot.links(nl+1).child_joint(end+1)=nj+2;
    [robot,nl,nj]=urdf2robot_recursive(robot,links,joints,joints(clink.child_joint{n}),nl+1,nj+1);
end

end




