%function path = findpath(startpoint, goalpoint)
tic;
clc;
p=0.1;
step=0.3;
% function for finding an obstacle free path through the provided map from
% the start location to the goal location
% 
% map   : a representation of the environment including obstacles used for
%         planning
%
% start : a 1x3 vector containing the coordinates (x,y,z) of the start
%         location
%
% goal  : a 1x3 vector containing the coordinates (x,y,z) of the goal
%         location
%
% path  : an Nx5 vector of joint variable values which constitute the path found
%         using your planner; Note: N is not known beforehand and will vary
%         depending how the path is constructed


% YOUR CODE GOES HERE
map = loadmap('lab3_maps\map_5.txt');
boundary = map.boundary;
obstacle1 = map.obstacles(1,1:end);
obstacle2 = map.obstacles(2,1:end);
obstacle3 = map.obstacles(3,1:end);
obstacle4 = map.obstacles(4,1:end);
obstacle=[obstacle1;obstacle2;obstacle3;obstacle4];    
x_length = double(boundary(4) - boundary(1));
y_length = double(boundary(5) - boundary(2));
z_length = double(boundary(6) - boundary(3));

startpoint = [100 300 50];
goalpoint = [300 10 400];

q_start = get_q(startpoint,obstacle); 
q_goal = get_q(goalpoint,obstacle);


lines = int32.empty(0, 2);
q_rand = double.empty(0, 4);
q_close = double.empty(0, 4);
q_new = double.empty(0, 4);
T_start = double.empty(0, 4);
stack = double.empty(0, 6);
T_start = q_start;

limit = 2000; 

joint1_lower_limit = -140;
joint1_upper_limit = 140;
joint2_lower_limit = -120;
joint2_upper_limit = 140;
joint3_lower_limit = -180;
joint3_upper_limit = 180;
joint4_lower_limit = -190;
joint4_upper_limit = 170;


xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
scatter3(startpoint(1,1), startpoint(1,2), startpoint(1,3),'red','filled');
hold on;
scatter3(goalpoint(1,1), goalpoint(1,2), goalpoint(1,3),'green','filled');
hold on;

for i = 1 : limit
   
   if rand() < p 
       q_rand = q_goal;
   else 
       q_rand = double([(randi([joint1_lower_limit, joint1_upper_limit])) (randi([joint2_lower_limit, joint2_upper_limit])) (randi([joint3_lower_limit, joint3_upper_limit])) (randi([joint4_lower_limit, joint4_upper_limit]))]) / 100;
   end
   
   [q_close, q_close_index] = find_q_close(q_rand, T_start);
   
   q_new = find_new_point(q_close, q_rand, step);
   
   if q_new(1)<double(joint1_lower_limit/100) || q_new(1)>double(joint1_upper_limit/100)|| q_new(2)<double(joint2_lower_limit/100) ||  q_new(2)>double(joint2_upper_limit/100) || q_new(3)<double(joint3_lower_limit/100) || q_new(3)>double(joint3_upper_limit/100) || q_new(4)<double(joint4_lower_limit/100) || q_new(4)>double(joint4_upper_limit/100)
        continue;
   end
   
   bool = check_collision(obstacle,q_close);
   
   if bool==0
       line_bool=check_line_collision(obstacle,q_close, q_new); 
       if(line_bool==0)  
       T_start=[T_start; q_new];
       q_new_index = size(T_start,1);
       jointpositions = joint_position(q_new);
   
       joint1 = jointpositions( 2, :);
       joint2 = jointpositions( 3, :);
       joint3 = jointpositions( 4, :);
       joint4 = jointpositions( 5, :);
       joint5 = jointpositions( 6, :);
       joint6 = jointpositions( 7, :);
       %scatter3(joint6(1,1),joint6(1,2),joint6(1,3),'blue');
       hold on;
       lines = [lines; [int32(q_new_index), int32(q_close_index)]];
       [qgoalontheline, q_ideal] = qgoal_on_the_line(q_close, q_new, q_goal);
           if isequal(q_new, q_goal) || qgoalontheline == 1
              if ~isequal(q_new, q_goal) 
                  T_start = T_start(1 : end, :);
                  T_start= [T_start; q_ideal];
                  end   
           path = path_decide(lines, T_start);
           RRTshowing(map, q_start, q_goal, T_start, lines, path);
           %%path_s = smooth12(path, T_start, obstacle);
           %%RRTshowing(map, q_start, q_goal, T_start, lines, path_s); 
           toc;
           return;
           end
       end
   end
end
    path = double.empty(0, 4);
    %RRTfailureshowing(map, q_start, q_goal, T_start, lines);
    toc;
    error('we cannot find a path using RRT');
% end


