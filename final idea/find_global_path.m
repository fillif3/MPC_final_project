function [trajectory,inputs] = find_global_path(goal,A,B,C,Q,R,obstacles,max_linear_velocity,max_linear_accelration,max_linear_force_diff,horizon,minimum_distance_to_obstacle)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
[F,H] = get_prediction_matrices(A,B,horizon);
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
if isempty(obstacles)
    Q_full=Q;
    for i=1:(horizon-1)
        Q_full=blkdiag(Q_full,Q);
    end
    R_full=R;
    for i=1:(horizon-1)
        R_full=blkdiag(R_full,R);
    end
    Hes=H'*Q_full*H+R_full;
    f=2*[(-goal)',zeros(1,3)]*F'*Q_full*H;
    [inputs,~,exitflag]=quadprog(Hes,f,constraints_A,constraints_b,constraints_Aeq,constraints_beq);
    inputs=inputs';
end
if ((~isempty(obstacles))||(exitflag<1))
    non_locon = @(u) global_pah_nonlocon(u,A,B,C,obstacles,horizon,3,minimum_distance_to_obstacle);
    cost_function = @(u) global_trajectory_cost_function(u,goal,A,B,C,Q,R,horizon,3);
    inputs=ga(cost_function,3*horizon,constraints_A,constraints_b,constraints_Aeq,constraints_beq,[],[],non_locon);
end
trajectory=zeros(6,horizon);
x=zeros(6,1);
for i =1:horizon
    u=inputs((i-1)*3+(1:3))';
    x=A*x+B*u;
    trajectory(:,i)=x;
end
end