function [d,boundary_point] = distPointToBox(p, box)
% calculates the distance d from point p = [x,y,z] to the boundary of the
% box represented by [x_min y_min z_min x_max y_max z_max]
%
% (d > 0) --> p is outside of the box
% (d < 0) --> p is inside the box

box_center = (box(1:3)+box(4:6))/2;
box_dimensions = box(4:6)-box(1:3);
dist = abs(p-box_center) - box_dimensions / 2;
direction = (p-box_center)/(norm(p-box_center));
step=max(box_dimensions);
boundary_point=box_center;

i_neg = dist < 0;
% multiple cases
switch(sum(i_neg))
    case 3 % point is inside the box
        d = max(dist);
        boundary_point=[0,0,0];
    case 2 % point is closest to a face of the box
        d = max(dist); % choose the positive distane (the other 2 are negative)
    case 1 % point is closest to an edge of the box
        dist = dist(~i_neg); % pull out positive values
        d = sqrt(sum(dist.^2));
    case 0 % point is closest to a vertex of the box
        d = sqrt(sum(dist.^2));
end

end