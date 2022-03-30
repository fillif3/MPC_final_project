function [c,ceq] = global_pah_nonlocon(inputs,A,B,C,obstacles,horizon,number_of_inputs,minimum_distance_to_obstacle)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
x=zeros(6,1);
c=zeros(horizon,1);
cost=0;
for i=1:horizon
    u=inputs((i-1)*number_of_inputs+(1:3))';
    x=A*x+B*u;
    y=C*x;

    min_dist=minimum_distance_to_obstacle+10;
    for j=obstacles
        min_dist=min(norm(y-j{1}'),min_dist);
    end
    c(i)=minimum_distance_to_obstacle-min_dist;
end
ceq=0;
end