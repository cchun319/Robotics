
%
%  q_start=[-0.2100    0.9700   -1.3900    0.2900 0 0];
% % q_final = [-1.3100   -0.7300   -0.1700   -0.9600 0 0];
% % q_start = [0.0900   -0.6100    1.7300   -1.04000  0  0];
%
% % q_start=[0.1   0.1    -1.5   0.8 0  0];
% % q_final=[-0.5   0.2   0   -0.5 0 0];
% % m2 step=0.2
% % q_start=[0.1   0.9    -1.5   0.8 0  0];
%  q_final=[-1   0.2   0.1   -0.5 0 0];
% % q_start=[0.0900   -0.6100    1.7300   -1.04000  0  0];
% % q_final=[-1.3100   -0.7300   -0.1700   -0.9600 0 0];
% % q_start=[0.3   0.5    -0.5   0.8 0  0];  q_final=[-0.5   0.5   0.1   -0.5 0 0];
% % q_final=[ -0.2500    1.1700   -1.5100   -0.2700 0 0];
% %  q_start=[-0.2100    0.9700   -1.3900    0.2900 0 0];
q_start=[0.1   0.1    -1.5   0.8 0  0];
q_final=[-0.1   0.5   -0.3   0.5 -0.3 0];
%  q_start=[0.0900   -0.6100    1.7300   -1.04000  0  0];
%  q_final=[-1.3100   -0.7300   -0.1700   -0.9600 0 0];
%download the map
map = loadmap('map_1.txt');
boundary = map.boundary;
obstacle = map.obstacles;


%decide the start position and end position in workspace
[jointPositions1,T0e1] = calculateFK_sol(q_start);
p_start=jointPositions1(2:6,:);

[jointPositions2,T0e2] = calculateFK_sol(q_final);
p_final=jointPositions2(2:6,:);

% check whether the final point is appropriate selected

selection1 = check_collision(obstacle,q_start);
if(selection1==1)
    error('error selection for start configuration');
end


selection2 = check_collision(obstacle,q_final);
if(selection2==1)
    error('error selection for final configuration');
end


T_start = double.empty(0, 3);
T_start = p_start(5,:);

q_cool = double.empty(0, 6);
q_cool = q_start;
% parameters

q2=q_start;
q1=q_start;
step1=0.1;
threshold=0.3;
thresforlocal=5*step1;
% previous threshold is 15
[j0,T0e] = calculateFK_sol(q_start);
p1=j0(2:6,:);
bool = check_collision(obstacle,q_start);

lynxStart();
plotmap(map,1);
hold on;
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
scatter3(p_start(5,1), p_start(5,2), p_start(5,3),'red','filled');
hold on;
scatter3(p_final(5,1), p_final(5,2), p_final(5,3),'green','filled');
hold on;





for(j=1:50000)
   
    [whethertoescape,q_random,state]=escapefromlocalminimum(q_cool,step1,thresforlocal,obstacle);
   
    if (bool == 0 &&  whethertoescape ==0  )
        q1=q2;
        [j1,T0e1] = calculateFK_sol(q1);
        T_start = [T_start;j1(6,:)];
        q_cool = [q_cool;q1];
 

        
        if(norm(q_final-q1)< threshold)
            q1=q_final;
            [j2,T0e2] = calculateFK_sol(q1);
            f1=j2(end,:);
            T_start = [T_start;f1];
            q_cool = [q_cool;q1];
            break;
        end
        
        [ftotal,inside_obstacles]=potentialfield(p1,p_final,obstacle);
        deltaq=step_in_cspace(q1,ftotal);
        delta=[step1*deltaq/norm(deltaq),0];
        q2=q1+delta;
        bool = check_collision(obstacle,q2);
       [whethertoescape,q_random,state]=escapefromlocalminimum(q_cool,step1,thresforlocal,obstacle);
    end
    
    if (whethertoescape ==1)
        q2=q_random;
        q1=q2;
        [j2,T0e2] = calculateFK_sol(q1);
        f1=j2(end,:);
        T_start = [T_start;f1];
        q_cool = [q_cool;5*q1];
        bool = check_collision(obstacle,q1);
        [whethertoescape,q_random,state]=escapefromlocalminimum(q_cool,step1,thresforlocal,obstacle);
    end
    
    if (bool == 1)
        error('failure due to collision under current potential field');
    end
    
    plotLynx(q1);
    [jointPositions3,T0e3] = calculateFK_sol(q1);
    scatter3(jointPositions3(6,1), jointPositions3(6,2), jointPositions3(6,3),'yellow','filled');
    hold on;
end

% check whether this point is in the obstacles
plotrobot(map,p_start,p_final,q_cool,T_start);



