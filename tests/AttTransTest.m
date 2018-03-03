%Test the attitude transformations in SPART.


%The tests rely on the aerospace toolbox
if not(license('test', 'aerospace_toolbox'))
    return;
end

%Repeat number of tests
num=10;

for i=1:10
    
    %% Angles123_DCM
    angles = rand(3,1);
    test=abs(Angles123_DCM(angles)-angle2dcm(angles(1),angles(2),angles(3),'xyz'))<1e-6;
    assert(all(test(:)));
    
    %% Angles321_DCM
    angles = rand(3,1);
    test=abs(Angles321_DCM(angles)-angle2dcm(angles(3),angles(2),angles(1),'zyx'))<1e-6;
    assert(all(test(:)));
    
    %% DCM_Angles (321)
    angles = rand(3,1);
    test=abs(DCM_Angles321(angle2dcm(angles(3),angles(2),angles(1),'zyx'))-angles)<1e-6;
    assert(all(test(:)));
    
    %% Euler axis and angle
    alpha = rand(1,1);
    e = rand(3,1);
    e = e/norm(e);
    test=abs(DCM_Euler(Euler_DCM(e,alpha))-[e;alpha])<1e-6;
    assert(all(test(:)));
    
    %% Quat to DCM
    q=rand(4,1);
    q=q/norm(q);
    test=abs(quat_DCM(q)-quat2dcm([q(4);q(1:3)]'))<1e-6;
    assert(all(test(:)));
    
end
