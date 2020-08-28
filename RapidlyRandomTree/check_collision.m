function [bool] = check_collision(obstacle,q_close)

    %q_close =  [1.2, 1.2, 1.2, 1.2];
    
    jointpositions = joint_position(q_close);
    d5=85; 
    lg=15;
    length_gripper=d5+lg;
    radius_gripper=38.1;
    radius_link3=6.35;
    r_link3 = 10;
    r_ee = 30;
    
    joint1 = jointpositions(2, :);
    joint2 = jointpositions(3, :);
    joint3 = jointpositions(4, :);
    joint4 = jointpositions(5, :);
    joint5 = jointpositions(6, :);
    joint6 = jointpositions(7, :);
    joint=[joint1;joint2;joint3;joint4;joint5;joint6];
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