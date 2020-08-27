function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. At first, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% parameters:
%   map: The map structure returned by the load_map function
%   path: This is the path returned by your planner (dijkstra function)
%   desired_state: Contains all the information that is passed to the
%                  controller, as in Phase 1
%
% NOTE: It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent map0 path0
% map0 = map;
% path0 = path;
persistent map0 path0 t_total A t_trajectory points path2
if nargin > 2
    map0=map;
    path2=path;
%  path=delete_point(path);
 path=simplify_path(path);
    path0=path;
    V=2;
    points=length(path0);
   path_distance=sum(sqrt(sum((path0(2:end, :) - path0(1:end-1,:)).^2,2)));
   t_total = path_distance/V;
   path_segemnt_dis = sqrt(sqrt(sum((path0(2:end, :) - path0(1:end-1,:)).^2,2)));
t_trajectory = cumsum(path_segemnt_dis);
t_trajectory = t_trajectory/t_trajectory(end);
t_trajectory = [0; t_trajectory ]';
t_trajectory = t_trajectory*t_total;
A = minimum_snap_trajectory(path0, t_total,t_trajectory);
else 
    map=map0;
    path=path0;
    if t>=t_total
    pos = path(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
    else
    i = find(t_trajectory<=t);
    i = i(end);
    pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*A(8*(i-1)+1:8*i,:);
    vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*A(8*(i-1)+1:8*i,:);
    acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*A(8*(i-1)+1:8*i,:);  
    end
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = 0;
desired_state.yawdot = 0;  
end
function A = minimum_snap_trajectory(path, t_total,t_trajectory)
    path0=path;
[P,n]=size(path0);
%%%%%number of path m%%%%%%
m=P-1;
%%%% N=3 for three dimension%%%%
%%%%%P points which is m+1%%%%
%t_trajectory(1)=t_0 which is time at start
%t_trajectory(k)=t_(k+1);
%t_trajectory(m+1)=t_(m)
%each segment has 8 parameters
A = zeros(8*m,3);
H = zeros(8*m,8*m);
B = zeros(8*m,3);
 H=eye(8*m)*eps;
%counter
c=1;
%%%2*(m-1)
% A = eye(8*m)*eps;
for k = 1:(m-1)
       H(2*k-1, 8*(k-1)+1:8*k) = [t_trajectory(k+1)^7, t_trajectory(k+1)^6, t_trajectory(k+1)^5, t_trajectory(k+1)^4, t_trajectory(k+1)^3, t_trajectory(k+1)^2, t_trajectory(k+1), 1];
       B(2*k-1,:) = path(k+1,:);
       H(2*k, 8*(k)+1:8*(k+1)) = [t_trajectory(k+1)^7, t_trajectory(k+1)^6, t_trajectory(k+1)^5, t_trajectory(k+1)^4, t_trajectory(k+1)^3, t_trajectory(k+1)^2, t_trajectory(k+1), 1];
       B(2*k,:) = path(k+1,:);
       H(2*m-2+k, 8*(k-1)+1:8*k) = [7*t_trajectory(k+1)^6, 6*t_trajectory(k+1)^5, 5*t_trajectory(k+1)^4, 4*t_trajectory(k+1)^3, 3*t_trajectory(k+1)^2, 2*t_trajectory(k+1), 1, 0];
       H(2*m-2+k, 8*(k)+1:8*(k+1))=-[7*t_trajectory(k+1)^6, 6*t_trajectory(k+1)^5, 5*t_trajectory(k+1)^4, 4*t_trajectory(k+1)^3, 3*t_trajectory(k+1)^2, 2*t_trajectory(k+1), 1, 0];      
       H(3*m-3+k, 8*(k-1)+1:8*k) = [42*t_trajectory(k+1)^5, 30*t_trajectory(k+1)^4, 20*t_trajectory(k+1)^3, 12*t_trajectory(k+1)^2, 6*t_trajectory(k+1), 2, 0, 0];
       H(3*m-3+k, 8*(k)+1:8*(k+1)) = -[42*t_trajectory(k+1)^5, 30*t_trajectory(k+1)^4, 20*t_trajectory(k+1)^3, 12*t_trajectory(k+1)^2, 6*t_trajectory(k+1), 2, 0, 0];
       H(4*m-4+k, 8*(k-1)+1:8*k) = [210*t_trajectory(k+1)^4, 120*t_trajectory(k+1)^3, 60*t_trajectory(k+1)^2, 24*t_trajectory(k+1), 6, 0, 0, 0];
       H(4*m-4+k, 8*(k)+1:8*(k+1)) = -[210*t_trajectory(k+1)^4, 120*t_trajectory(k+1)^3, 60*t_trajectory(k+1)^2, 24*t_trajectory(k+1), 6, 0, 0, 0];
       H(5*m-5+k, 8*(k-1)+1:8*k) = [840*t_trajectory(k+1)^3, 360*t_trajectory(k+1)^2, 120*t_trajectory(k+1), 24, 0, 0, 0, 0];
       H(5*m-5+k, 8*(k)+1:8*(k+1)) = -[840*t_trajectory(k+1)^3, 360*t_trajectory(k+1)^2, 120*t_trajectory(k+1), 24, 0, 0, 0, 0];
       H(6*m-6+k, 8*(k-1)+1:8*k) = [2520*t_trajectory(k+1)^2, 720*t_trajectory(k+1), 120, 0, 0, 0, 0, 0];
       H(6*m-6+k, 8*(k)+1:8*(k+1)) = -[2520*t_trajectory(k+1)^2, 720*t_trajectory(k+1), 120, 0, 0, 0, 0, 0];
       H(7*m-7+k, 8*(k-1)+1:8*k) = [5040*t_trajectory(k+1), 720, 0, 0, 0, 0, 0, 0];
       H(7*m-7+k, 8*(k)+1:8*(k+1)) = -[5040*t_trajectory(k+1), 720, 0, 0, 0, 0, 0, 0];
end
 B(2*m-1:8*m,:)=0;
   k = 1;
   H(8*m-7, 8*(k-1)+1:8*k) = [t_trajectory(k)^7, t_trajectory(k)^6, t_trajectory(k)^5, t_trajectory(k)^4, t_trajectory(k)^3, t_trajectory(k)^2, t_trajectory(k), 1];
   B(8*m-7,:) = path(k,:);
   H(8*m-6, 8*(k-1)+1:8*k) = [7*t_trajectory(k)^6, 6*t_trajectory(k)^5, 5*t_trajectory(k)^4, 4*t_trajectory(k)^3, 3*t_trajectory(k)^2, 2*t_trajectory(k), 1, 0];
   H(8*m-5, 8*(k-1)+1:8*k) = [42*t_trajectory(k)^5, 30*t_trajectory(k)^4, 20*t_trajectory(k)^3, 12*t_trajectory(k)^2, 6*t_trajectory(k), 2, 0, 0];
   H(8*m-4, 8*(k-1)+1:8*k) = [210*t_trajectory(k)^4, 120*t_trajectory(k)^3, 60*t_trajectory(k)^2, 24*t_trajectory(k), 6, 0, 0, 0];
   k = m;
   H(8*m-3, 8*(k-1)+1:8*k) = [t_trajectory(k+1)^7, t_trajectory(k+1)^6, t_trajectory(k)^5, t_trajectory(k+1)^4, t_trajectory(k+1)^3, t_trajectory(k+1)^2, t_trajectory(k+1), 1];
   B(8*m-3,:) = path(k+1,:);
   H(8*m-2, 8*(k-1)+1:8*k) = [7*t_trajectory(k+1)^6, 6*t_trajectory(k+1)^5, 5*t_trajectory(k)^4, 4*t_trajectory(k+1)^3, 3*t_trajectory(k+1)^2, 2*t_trajectory(k+1), 1, 0];
   H(8*m-1, 8*(k-1)+1:8*k) = [42*t_trajectory(k+1)^5, 30*t_trajectory(k+1)^4, 20*t_trajectory(k)^3, 12*t_trajectory(k+1)^2, 6*t_trajectory(k+1), 2, 0, 0];
   H(8*m, 8*(k-1)+1:8*k) = [210*t_trajectory(k+1)^4, 120*t_trajectory(k+1)^3, 60*t_trajectory(k)^2, 24*t_trajectory(k+1), 6, 0, 0, 0];
   H=H+eye(8*m)*eps;
   A=H\B;
end
end

function path_new = delete_point(path)  %path nx3 matrix; path_new mx3 matrix
    path_diff = diff(path); 
    path_dis = sqrt(path_diff(:, 1).^2 + path_diff(:, 2).^2 + path_diff(:, 3).^2)';
    path_dis = path_dis';  %(n-1)x3 matrix
    path_orie = path_diff ./ path_dis;
    path_new = path;
    delete = [];
    for i = 1: size(path_orie)-1
        if path_orie(i,:) == path_orie(i+1,:)
            delete = [delete; i+1];
        end
    end
    path_new(delete,:) = [];
end
function path1 = simplify_path(path)
path2=path;
Remove = logical(zeros(size(path,1),1));
for i = 1:size(path,1)-1
    if norm(path(i+1,:) - path(i,:)) < 1e-4
        Remove(i) = 1;
    end
end
path = path(~Remove,:);
path = path(1:2:end, :);
Remove = logical(zeros(size(path,1),1));
for i = 1:size(path,1) - 2
    if sum(sum((abs(bsxfun(@minus, path(i:i+2,:), mean(path(i:i+2,:))))<0.0001))) >= 6
        Remove(i+1) = 1;
    end
end
path = path(~Remove,:);
% path1=path;
% path1=[];
% for i = 1:size(path,1)-1
%     path1 = [path1; path(i,:)];
% %     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.1];
% %     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.2];
% %     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.5];
% %     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.7];
% %     path1 = [path1; path(i,:) + (path(i+1,:)-path(i,:))*0.9];
% end
path1 =[path; path2(end,:)];
end



    
    
    
  
    
    
    
    
    
    
    
    
    
    
    
    
    
    