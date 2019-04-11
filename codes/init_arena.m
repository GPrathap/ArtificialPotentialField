function init_arena()

global arena_map qstart qgoal k ObsTh GoalTh;
arena_map = [];
k=2;
ObsTh = 1;
GoalTh = 3;
arena_map{1} = [ 0 -4 0 1 ];    % x y z r
arena_map{2} = [ 1 0 1 2 ];    % x y z r
arena_map{3} = [ -6 1 -1 2 ];    % x y z r
qstart = [-9.5 1.5 -1.5];   % x y z
qgoal = [8 0 0];      % x y z

% %%%%%%%%%CONFIG 3 Multiple Obstacles, With Local Minima
% 
% arena_map{1} = [ 0 -1  0 0.5 ];    % x y z r
% arena_map{2} = [ 0  0  1 0.5 ];    % x y z r
% arena_map{3} = [ 0  1  0 0.5 ];    % x y z r
% arena_map{4} = [ 0  0 -1 0.5 ];    % x y z r
% 
% qstart = [-9.5 0.01 0.01];   % x y z
% qgoal = [8 0 0];      % x y z
end

