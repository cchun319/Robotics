
% 
% q_start=[-0.2100    0.9700   -1.3900    0.2900 0 0];
% q_final = [-1.3100   -0.7300   -0.1700   -0.9600 0 0];
% q_start = [0.0900   -0.6100    1.7300   -1.04000  0  0];

% q_start=[0.1   0.1    -1.5   0.8 0  0]; 
% q_final=[-0.5   0.2   0   -0.5 0 0];
% m2 step=0.2 
q_start=[0.1   0.9    -1.5   0.8 0  0]; q_final=[-1   0.2   0.1   -0.5 0 0];
% q_start=[0.0900   -0.6100    1.7300   -1.04000  0  0];
% q_final=[-1.3100   -0.7300   -0.1700   -0.9600 0 0]; 
% q_start=[0.3   0.5    -0.5   0.8 0  0];  q_final=[-0.5   0.5   0.1   -0.5 0 0];
% q_final=[ -0.2500    1.1700   -1.5100   -0.2700 0 0];
%  q_start=[-0.2100    0.9700   -1.3900    0.2900 0 0];


%download the map
map = loadmap('map_2.txt');
boundary = map.boundary;
obstacle = map.obstacles;
% 
% p1=[250 150 400];p2=[-100 150 400];
% q_start(1,1:4) = get_q(p1,obstacle);
% q_start= [q_start,0,0];
% q_final(1,1:4)   = get_q(p2,obstacle);
% q_final= [q_final,0,0];


%decide the start position and end position in workspace 
[jointPositions1,T0e1] = calculateFK_sol(q_start);
 p_start=jointPositions1(2:6,:);
 
 [jointPositions2,T0e2] = calculateFK_sol(q_final);
 p_final=jointPositions2(2:6,:);
 
 % check whether the final point is appropriate selected 
 
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
% 
%  T_2 = double.empty(0, 3);
%  T_2 = p_start(1,:);
% 
%  T_3 = double.empty(0, 3);
%  T_3 = p_start(2,:);
%  
%  T_4 = double.empty(0, 3);
%  T_4 = p_start(3,:);
%  
%  T_5 = double.empty(0, 3);
%  T_5 = p_start(4,:);
% 
bool2=0;
q_cool = double.empty(0, 6);
q_cool = q_start;
 % parameters 
 
  q2=q_start;
  q1=q_start;
  step1=0.2;
  threshold=0.8;
  thresforlocal=0.19;
  % previous threshold is 15
  [j0,T0e] = calculateFK_sol(q_start);
  p1=j0(2:6,:);
  bool = check_collision(obstacle,q_start);
  
for(j=1:50000) 
    if (bool == 0)
         [whethertoescape,q_random,state]=escapefromlocalminimum(q_cool,step1,thresforlocal);
         if whethertoescape ==0
         q1=q2; 
         [j1,T0e1] = calculateFK_sol(q1);
         T_start = [T_start;j1(6,:)];
        
%          T_2 = [T_2;j1(2,:)];
%          
%          T_3 = [T_3;j1(3,:)];
%          
%          T_4 = [T_4;j1(4,:)];
%          
%          T_5 = [T_5;j1(5,:)];
%          
         q_cool = [q_cool;q1];
          
         p1=j1(2:6,:);
         
          if(norm(q_final-q1)< threshold)
              q1=q_final;
              [j2,T0e2] = calculateFK_sol(q1);
              f1=j2(end,:);
              bool2 = check_collision(obstacle,q1);
              if bool2 == 0
                   T_start = [T_start;f1];
                   q_cool = [q_cool;q1];
                break;
              else
                  continue
              end
          end
          
         [ftotal,inside_obstacles]=potentialfield(p1,p_final,obstacle);
          deltaq=step_in_cspace(q1,ftotal);
          delta=[step1*deltaq/norm(deltaq),0];
          q2=q1+delta;
%           
        else if whethertoescape ==1
                 q2=q_random;
             end
         end
         bool = check_collision(obstacle,q2);
                
    else if (bool == 1)
            [frandi,inside_obstacles]=randi_potentialfield(p1,p_final,obstacle,boundary);
            randideltaq=step_in_cspace(q1,frandi);
            randidelta=[step1*randideltaq/norm(randideltaq),0];
            q2=q1+randidelta;
            bool = check_collision(obstacle,q2);
        end
    end
     plotLynx(q1);
    [jointPositions3,T0e3] = calculateFK_sol(q1);
    scatter3(jointPositions3(6,1), jointPositions3(6,2), jointPositions3(6,3),'yellow','filled');
    hold on;
    end
        % check whether this point is in the obstacles 
         plotrobot(map,p_start,p_final,q_cool,T_start)  
     
%  plotpath(T_start);
    
