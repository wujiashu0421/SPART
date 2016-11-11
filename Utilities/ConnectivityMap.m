function [Branch,Child,Child_base]=ConnectivityMap(robot)%#codegen
%Produced the connectivity map for a robot model

%Pre-allocate Branch connectivity map
Branch=zeros(robot.n_links,robot.n_links);

%Populate Branch connectivity map
for i=robot.n_links:-1:1
    for j=i:-1:1
        Branch(i,j)=1;
        if robot.joints(robot.links(j).parent_joint).parent_link==0
            
            break
        end

    end
end

%Populate Child map
Child=zeros(robot.n_links,robot.n_links);
Child_base=zeros(robot.n_links,1);
for i=robot.n_links:-1:1
    parent_link=robot.joints(robot.links(i).parent_joint).parent_link;
    if parent_link~=0
        Child(i,parent_link)=1;
    else
        Child_base(i)=1;
    end
end



end
