function [value,isterminal,direction] = ar_event(t,rob_pos)

global epsilon_gr;
global epsilon_goal;
global qgoal;

gradient = pot_ar(t,rob_pos);

Dummy = [0 0 0];
Dummy(1) = rob_pos(1)-qgoal(1);
Dummy(2) = rob_pos(2)-qgoal(2);
Dummy(3) = rob_pos(3)-qgoal(3);
m=norm(Dummy);
value = [ (m<=epsilon_goal)*1  (norm(gradient)>epsilon_gr)*1];  
isterminal = [ 1 1 ]; 
direction = [ 0 0 ];

end