 function [path] = path_decide(lines, T_start)
    path = lines(end, 1);
    previous = lines(end, 2);     
    k = 0;
    number = size(lines,1);
    while previous ~= 1
        if k > number 
            error('we cannot find a path using RRT');
        end
        previousIndex = find(lines(:, 1) == previous);
        previous = lines(previousIndex(1), 2);
        path = [path, previous];
        k = k + 1;
    end
   end
