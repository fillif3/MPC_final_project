clear; clc
rng=(1);
noise_flag=false;
measruable_state=false;
%% define drone parameters
m=0.74;
l=0.5;
r=0.1;
g=9.8;
Ix=4*10^(-3);
Iy=4*10^(-3);
Iz=8.4*10^(-3);
la=0.21;
b=29*10^(-6);
d=1.1*10^(-6);
Jr=455.*10^(-6);
C=eye(6); %I assume each MPC uses same matrix C
if ~measruable_state
    C=C(1:3,:);
end
size_of_c=size(C);
dimensiotn_of_output=size_of_c(1);
if noise_flag
    var_system=0.001;
    var_measurment=0.001;
else
    var_system=0;
    var_measurment=0;
end

%% Prepare Translation MPC
% MPC translation parameters
Ts_translation=0.05;
horizon_translation=11;
treshold_translation=0.1;
Q_scalar_translation=20;
R_scalar_translation=0.1;
max_distance_from_waypoint=1.5;%max_linear_velocity*Ts_global;
max_linear_velocity=20;
max_linear_force=11.5;
max_linear_force_difference=0.5;

% MPC translation system matrices
[A_translation,B_translation,B_translation_noise]=get_trasnlation_model(g,Ts_translation,m);
dimenstion_of_state_vector_translation=length(A_translation);
size_of_B=size(B_translation);
dimenstion_of_input_vector_translation=size_of_B(2);
if ~check_if_obsrvable_and_controlable(A_translation,B_translation,C)
    error('System not obsevable or not controlable')
end
[F_translation,H_translation] = get_prediction_matrices(A_translation,B_translation,horizon_translation);

% MPC translation cost function

[Q_full_translation,R_full_translation,Q_matrix_translation,R_matrix_translation]=get_full_weight_matrices(A_translation,B_translation,Q_scalar_translation,R_scalar_translation,[0,0,0,1,1,1],horizon_translation);
get_cost_function_translation = @(u,x_0) reference_cost_function(u,x_0,F_translation,H_translation,Q_full_translation,R_full_translation);
% Get quadratic cost function
%Hessian_translation=H_translation'*Q_full_translation*H_translation+R_full_translation;
%Get_linear_component_translation= @(x0) H_translation'*Q_full_translation*F_translation*x0;

[constraints_translation_A,get_constraints_translation_b]=prepare_constraints_translation(F_translation,H_translation,horizon_translation,max_distance_from_waypoint,max_linear_velocity,max_linear_force,max_linear_force_difference);


%% Preaper trajectory
horizon_global=15;
Ts_global= (horizon_translation-1)*Ts_translation;
[A_global,B_global,~]=get_trasnlation_model(g,Ts_global,m);
obstacles=[];%{[5,5,0],[3,7,0],[4,6,0],[6,4,0],[7,3,0]};
treshold=2*max_distance_from_waypoint;
goal=[10;10;0];
[trajectory_global,inputs_global] = find_global_path(goal,A_global,B_global,C,Q,R,obstacles,max_linear_velocity/2,max_linear_force/2,max_linear_force_difference, horizon_global,treshold);

%% Plot results
%plot trajectory
plot3(trajectory_global(1,:),trajectory_global(2,:),trajectory_global(3,:));
hold on
%plot obstacles
[X,Y,Z] = sphere;
for i=obstacles
    o=i{1};
    surf(X*treshold+o(1),Y*treshold+o(2),Z*treshold+o(3))
end

