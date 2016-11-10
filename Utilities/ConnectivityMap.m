function [C]=ConnectivityMap(robot)%#codegen
%Produced the connectivity map for a robot model

%Pre-allocate connectivity map
C=zeros(robot.n_links,robot.n_links);

%Populate connectivity map
for i=robot.n_links:-1:1
    for j=i:-1:1
        C(i,j)=1;
        if robot.joints(robot.links(j).parent_joint).parent_link==0
            break
        end
    end
end


end
