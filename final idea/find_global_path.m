function [trajectory,inputs] = find_global_path(goal,A,B,C,Q,R,obstacles,max_linear_velocity,max_linear_accelration,max_linear_force_diff,horizon,minimum_distance_to_obstacle)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
[~,H] = get_prediction_matrices(A,B,horizon);
constraints_A=H;
for i=1:6*horizon
    if mod(i-1,6)<3
        constraints_A(i,:)=-H(i+3,:);
    end
end
constraints_b=max_linear_velocity*ones(horizon*6,1);
constraints_A_accelration=([eye(3*horizon);-eye(3*horizon)]);
constraints_b_accelration=max_linear_accelration*ones(6*horizon,1);
constraints_A_accelration_diff=([eye(3*horizon)-diag(ones(3*(horizon-1),1),-3);-eye(3*horizon)+diag(ones(3*(horizon-1),1),-3)]);
constraints_b_accelration_diff=max_linear_force_diff*ones(6*horizon,1);
constraints_A=[constraints_A;constraints_A_accelration;constraints_A_accelration_diff];
constraints_b=[constraints_b;constraints_b_accelration;constraints_b_accelration_diff];
constraints_Aeq=H((end-5):end,:);
constraints_beq=[goal;zeros(3,1)];
cost_function = @(u) global_trajectory_cost_function(u,goal,A,B,C,Q,R,obstacles,horizon,3,minimum_distance_to_obstacle);
inputs=ga(cost_function,3*horizon,constraints_A,constraints_b,constraints_Aeq,constraints_beq);
trajectory=zeros(6,horizon);
x=zeros(6,1);
for i =1:horizon
    u=inputs((i-1)*3+(1:3))';
    x=A*x+B*u;
    trajectory(:,i)=x;
end
end