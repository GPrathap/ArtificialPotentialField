function draw_arena();

global arena_map arena_r qstart qgoal;

[x,y,z] = sphere;
surf(x*arena_r,y*arena_r,z*arena_r,'FaceAlpha',0.2,'EdgeAlpha',0.4);  ...
hold on    
for i = 1:length(arena_map)
    surf(x*arena_map{i}(4)+arena_map{i}(1), ...
         y*arena_map{i}(4)+arena_map{i}(2), ...
         z*arena_map{i}(4)+arena_map{i}(3), ...
         'FaceAlpha',0.6,'EdgeAlpha',0.8);  % sphere centered at (3,-2,0)
end
plot3( qstart(1), qstart(2), qstart(3), '+', 'MarkerFaceColor','g', ...
            'MarkerEdgeColor','g','MarkerSize',8);

plot3( qgoal(1), qgoal(2), qgoal(3), 'x', 'MarkerFaceColor','r', ...
            'MarkerEdgeColor','r','MarkerSize',8);        
        
daspect([1 1 1])    % Axes ratios. Keep them at Original size.

end