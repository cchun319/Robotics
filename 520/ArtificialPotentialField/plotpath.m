function plotpath(T_start)
number=size(T_start,1)
for(j=1:(number-1))
    scatter3(T_start((j+1),1),T_start((j+1),2),T_start((j+1),3),'black','filled');
    hold on;
    plot3([T_start(j,1),T_start((j+1),1)],...
    [T_start(j,2),T_start((j+1),2)],[[T_start(j,3),T_start((j+1),3)]],'blue', 'linewidth', 1);
        hold on;
     
end
    