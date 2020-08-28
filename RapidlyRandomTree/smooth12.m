function [path_smooth] = smooth12(path, T_start, obstacle)

path_smooth = path(1); 
currentIndex = 1; 
currentSmoothIndex = numel(path); 

while currentIndex < numel(path)
    
    while currentIndex < currentSmoothIndex
        T_s = T_start(path(currentSmoothIndex), :);
        T_c = T_start(path(currentIndex), :);
        if check_line_collision2(obstacle, T_s, T_c )
            path_smooth = [path_smooth, path(currentSmoothIndex)];
            currentIndex = currentSmoothIndex;
            break;
        else
            currentSmoothIndex = currentSmoothIndex - 1;
        end
        
    end
    
    currentSmoothIndex = numel(path);
    
end


end