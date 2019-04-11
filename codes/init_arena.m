function init_arena()

global arena_map qstart qgoal k ObsTh GoalTh;
arena_map = [];
k=2;
ObsTh = 1;
GoalTh = 3;
 arena_map{1} = [32,32,28, 1]
 arena_map{2} = [33,31,28, 1]
 arena_map{2} = [33,31,28, 1]
 arena_map{3} = [34,31,28, 1]
 arena_map{4} = [35,31,28, 1]
 arena_map{5} = [35,32,28, 1]
 arena_map{6} = [36,32,28, 1]
 arena_map{7} = [37,32,28, 1]
 arena_map{8} = [38,32,28, 1]
 arena_map{9} = [39,31,28, 1]
 arena_map{10} = [39,32,28, 1]
 arena_map{11} = [40,33,29, 1]
 arena_map{12} = [41,33,29, 1]
 arena_map{13} = [42,32,28, 1]
 arena_map{14} = [43,33,29, 1]
 arena_map{15} = [44,33,29, 1]
 arena_map{16} = [45,33,29, 1]
 arena_map{17} = [47,33,29, 1]
 arena_map{18} = [48,33,29, 1]
 arena_map{19} = [49,33,29, 1]
 arena_map{20} = [50,33,29, 1]
 arena_map{21} = [54,37,30, 1]
qstart = [150, 150, 150];   % x y z
qgoal = [280, 170, 150];      % x y z

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

