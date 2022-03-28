function [trajectory,inputs] = optimize_trajectory_global(x,goal,A,B,C,Q,R,obstacles,constraints_A,constraints_b,horizon,minimum_distance_to_obstacle)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
size_of_B=size(B);
number_of_inputs=size_of_B(1);
cost_function = @(u) global_trajectory_cost_function(u,x,goal,A,B,C,Q,R,obstacles,horizon,number_of_inputs,minimum_distance_to_obstacle);
inputs=ga(cost_function,number_of_inputs*horizon,constraints_A,constraints_b);
trajectory=zeros(length(x),horizon);
for i =1:horizon
    u=inputs((i-1)*number_of_inputs+(1:3));
    x=A*x+B*u;
    trajectory(:,i)=x;
end