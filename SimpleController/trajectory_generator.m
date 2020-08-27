function [desired_state] = trajectory_generator(t, qn, varargin)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
% t: A scalar, specifying inquiry time
%
% varargin: variable number of input arguments. In the framework,
% this function will first (and only once!) be called like this:
%
% trajectory_generator([],[], 0, path)
%
% i.e. map = varargin{1} and path = varargin{2}.
%
% path: A N x 3 matrix where each row is (x, y, z) coordinate of a
% point in the path. N is the total number of points in the path
%
% This is when you compute and store the trajectory.
%
% Later it will be called with only t and qn as an argument, at
% which point you generate the desired state for point t.
%
persistent via_pts

if nargin == 4
    via_pts = varargin{2};
end

num_session = size(via_pts ,1);

pos = [0; 0; 0];
vel = [0; 0; 0];
acc = [0; 0; 0];

v_dir = diff(via_pts);
d = sqrt(sum(v_dir.^2,2));
t_each = 3 * d; % 1m/3sec;
time = [0; cumsum(t_each, 1)];
a = d ./ t_each.^2 .* v_dir ./d;

for i = 1 : num_session - 1
    if t >= time(i) && t < time(i+1)
        pos = via_pts(i,:)' + 0.5 * a(i,:)' * (t - time(i))^2
        vel = a(i,:)' * (t - time(i))
        acc = a(i,:)'
        if t >= ( time(i) + 0.5*(time(i+1) - time(i)))
            acc = -a(i,:)'
            vel = a(i,:)' * (time(i+1) - time(i)) - a(i,:)' * (t - time(i))
            pos = via_pts(i,:)'  + a(i,:)'* (time(i+1) - time(i)) * (t - time(i)) - a(i,:)' * 0.5 * (t - time(i))^2 - 0.25 * a(i,:)' * (time(i+1) - time(i))^2
        else
            pos = via_pts(end,:)';
            vel = [0; 0; 0];
        end
        
    end
end

yaw = 0;
yawdot = 0;


desired_state = [];

% use the "persistent" keyword to keep your trajectory around
% inbetween function calls



%
% When called without varargin (isempty(varargin) == true), compute
% and return the desired state here.
%

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
