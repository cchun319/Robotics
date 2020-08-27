function [time,T_start,q_cool,state]=findpath(q_start,q_final)

tic;
%download the map
map = loadmap('map_3.txt');
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

state = double.empty(0, 1);
state = 0;
% parameters

q2=q_start;
q1=q_start;
step1=0.005;
threshold=0.2;
thresforlocal=0.007;
% previous threshold is 15
[j0,T0e] = calculateFK_sol(q_start);
p1=j0(2:6,:);
bool = check_collision(obstacle,q_start);

% lynxStart();
% plotmap(map,1);
% hold on;
% xlabel('x axis');
% ylabel('y axis');
% zlabel('z axis');
% scatter3(p_start(5,1), p_start(5,2), p_start(5,3),'red','filled');
% hold on;
% scatter3(p_final(5,1), p_final(5,2), p_final(5,3),'green','filled');
% hold on;

whethertoescape = 0;

for(j=1:50000)
    if j > 3
        [whethertoescape,state]=escapefromlocalminimum3(q_cool,thresforlocal);
        [whethertoescape,currentstate]=escapefromlocalminimum3(q_cool,thresforlocal);
        state=[state; currentstate];
    end
    if whethertoescape == 0
        q1=q2;
        [j1,T0e1] = calculateFK_sol(q1);
        T_start = [T_start;j1(6,:)];
        
        q_cool = [q_cool;q1];
        p1=j1(2:6,:);
        [ftotal,inside_obstacles]=potentialfield(p1,p_final,obstacle);
        deltaq=step_in_cspace(q1,ftotal);
        delta=[step1*deltaq/norm(deltaq),0];
        q2=q1+delta;
        bool = check_collision(obstacle,q2);
        %[whethertoescape,q_random,state]=escapefromlocalminimum(q_cool,step1,thresforlocal,obstacle);
        
        while bool==1
            error('failure due to collision under current potential field');
            break
        end
        
        if (norm(q_final-q1)< threshold)|| (norm(q_final(1,1:4)-q_cool(end,1:4))< 0.05)
            q1=q_final;
            [j2,T0e2] = calculateFK_sol(q1);
            f1=j2(end,:);
            T_start = [T_start;f1];
            q_cool = [q_cool;q1];
            
            %             plotLynx(q1);
            %             [jointPositions3,T0e3] = calculateFK_sol(q1);
            %             scatter3(jointPositions3(6,1), jointPositions3(6,2), jointPositions3(6,3),'yellow','filled');
            %             hold on;
            
            break;
        end
    elseif whethertoescape == 1
        q_random = rand_move(q_cool, obstacle);
        q1=q_random;
        [j2,T0e2] = calculateFK_sol(q1);
        f1=j2(end,:);
        T_start = [T_start;f1];
        q_cool = [q_cool;q1];
    end
    
    %     plotLynx(q1);
    %     [jointPositions3,T0e3] = calculateFK_sol(q1);
    %     scatter3(jointPositions3(6,1), jointPositions3(6,2), jointPositions3(6,3),'yellow','filled');
    %     hold on;
    
end
% toc;
time=toc;
% plot the whole path

%   plotrobot(map,p_start,p_final,q_cool,T_start);
end
