function [bool] = check_collision(obstacle,q2)
% find all joint positions
[jointPositions,T0e] = calculateFK_sol(q2);
joint_positions=jointPositions;
joint6=T0e(1:3,4);
joint6=joint6';
jointpositions=[joint_positions;joint6];
d5=85;
lg=15;
%     length_gripper=d5+lg;
%     radius_gripper=38.1;
%     % radius_link3=6.35;
%     radius_link3=12;
%     r_link3 = 10;
%     r_ee = 30;

length_gripper=d5+lg;
radius_gripper= 5;
% radius_link3=6.35;
radius_link3= 5;
r_link3 = 5;
r_ee = 5;

joint = jointpositions(2:7,:);

bool = 0;
obstacle_number=size(obstacle,1);
for k=1:5
    j1= joint(k,:);
    j2= joint(k+1,:);
    for j=1:obstacle_number
        obs=obstacle(j,:);
        iscollided = detectcollision(j1, j2, obs);
        if(k==3)
            obs=obstacle(j,:);
            obs(1,1:3)=obs(1,1:3)-radius_link3;
            obs(1,4:6)=obs(1,4:6)+radius_link3;
            iscollided = detectcollision(j1, j2, obs);
        else if(k==4)
                obs=obstacle(j,:);
                j2= joint(k+2,:);
                obs(1,1:3)=obs(1,1:3)-radius_gripper;
                obs(1,4:6)=obs(1,4:6)+radius_gripper;
                iscollided = detectcollision(j1, j2, obs);
                while iscollided == 1
                    bool = 1;
                    return;
                end
            end
        end
    end
end