function cost = global_trajectory_cost_function(inputs,x,goal,A,B,C,Q,R,obstacles,horizon,number_of_inputs,minimum_distance_to_obstacle)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
for i=1:horizon
    u=inputs((i-1)*number_of_inputs+(1:3));
    x=A*x+B*u;
    y=C*x;
    error=goal-y;
    cost=cost+error'*C*Q*C'*error+u'*R*u;
    min_dist=minimum_distance_to_obstacle;
    for j=obstacles
        min_dist=min(norm(y-j{1}),min_dist);
    end
    cost=cost+10^6*(minimum_distance_to_obstacle-min_dist);
end
end

