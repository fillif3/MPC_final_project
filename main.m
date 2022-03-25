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

%% Prepare simulation and objective
% Simulation parameters
numberOfIterations=50;
options = optimoptions('fmincon','Algorithm','interior-point');


% define starting state and trajectory
x=[[10,0;0,15]*ones(2,1);zeros(10,1)];
x_translation=[zeros(7,1);[10,0;0,15]*ones(2,1)];
x_translation_history=zeros(9,numberOfIterations+1);
x_translation_history(:,1)=x_translation;
x_rotation=zeros(9,1);
x_estimated_translation=x_translation;
x_estimated_rotation=x_rotation;

xHistory=zeros(12,numberOfIterations+1);
trajectory_generator= @(t,x,ts,horizon) constant_trajectory(t,x,ts,horizon);
previous_input_translation=[0;0;0];
previous_input_rotation=[0;0;0];


%% Prepare Translation MPC
% MPC translation parameters
Ts_translation=0.05;
horizon_translation=8;
horizon_translation_control=2;
Q_scalar_translation=20;
R_scalar_translation=0.1;
alpha_translation=1.05;
lambda_translation=0.96;
max_linear_velocity=20;
max_linear_accelration=11.5;

% MPC translation system matrices
[A_translation,B_translation,B_translation_noise]=get_trasnlation_model(g,Ts_translation,m);
dimenstion_of_state_vector_translation=length(A_translation);
size_of_B=size(B_translation);
dimenstion_of_input_vector_translation=size_of_B(2);
[A_translation_augmented,B_translation_augmented,B_translation_noise_augmented,C_translation_augmented]=get_augmented_model(A_translation,B_translation,B_translation_noise,C);
dimenstion_of_augmented_state_vector_translation=length(A_translation_augmented);
if ~check_if_obsrvable_and_controlable(A_translation_augmented,B_translation_augmented,C_translation_augmented)
    error('System not obsevable or not controlable')
end
[F_translation,H_translation] = get_prediction_matrices(A_translation_augmented,B_translation_augmented,alpha_translation,horizon_translation,horizon_translation_control);
[F_translation_normal,H_translation_normal] = get_prediction_matrices(A_translation_augmented,B_translation_augmented,1,horizon_translation,horizon_translation_control);

% MPC translation cost function

[Q_full_translation,R_full_translation]=get_full_weight_matrices(A_translation_augmented,B_translation_augmented,Q_scalar_translation,R_scalar_translation,[0,0,0,0,0,0,1,1,1],lambda_translation,alpha_translation,horizon_translation,horizon_translation_control);
get_cost_function_translation = @(u,x_0,trajectory) reference_cost_function(u,x_0,trajectory,F_translation,H_translation,Q_full_translation,R_full_translation,horizon_translation);
%Hessian_translation=H_translation'*Q_full_translation*H_translation+R_full_translation;
%Get_linear_component_translation= @(x0) H_translation'*Q_full_translation*F_translation*x0;

% MPC translation state (velocity) constraints
velocity_constraints_A=zeros(6*horizon_translation,dimenstion_of_input_vector_translation*horizon_translation_control);
velocity_constraints_b_helper=zeros(6*horizon_translation,length(A_translation_augmented));

for i=1:horizon_translation
    delta_x_i=H_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i=H_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    velocity_constraints_A((i-1)*6+(1:3),:)=delta_x_i/Ts_translation+delta_v_i;
    velocity_constraints_A((i-1)*6+(4:6),:)=-delta_x_i/Ts_translation-delta_v_i;
    delta_x_i_x0=F_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i_x0=F_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    velocity_constraints_b_helper((i-1)*6+(1:3),:)=-delta_x_i_x0/Ts_translation-delta_v_i_x0;
    velocity_constraints_b_helper((i-1)*6+(4:6),:)=delta_x_i_x0/Ts_translation+delta_v_i_x0;
end
Get_velocity_constraints_b= @(x0) velocity_constraints_b_helper*x0+max_linear_velocity*ones([6*horizon_translation,1]);

% MPC translation input (accelration) constraints
input_constraints_A=zeros(6*horizon_translation_control,dimenstion_of_input_vector_translation*horizon_translation_control);
input_constraints_b_helper=zeros(6*horizon_translation_control,3);
for i=1:horizon_translation_control
    for j=1:i
        input_constraints_A((i-1)*6+(1:3),3*(j-1)+(1:3))=eye(3);
        input_constraints_A((i-1)*6+(4:6),3*(j-1)+(1:3))=-eye(3);
    end
    input_constraints_b_helper((i-1)*6+(1:3),:)=-eye(3);
    input_constraints_b_helper((i-1)*6+(4:6),:)=eye(3);
end
Get_input_constraints_b= @(u_previous) input_constraints_b_helper*u_previous+max_linear_accelration*ones([6*horizon_translation_control,1]);



%% Prepare Rotation MPC
% MPC Rotation parameters
NumberOfIterationsOfInterloop=10;
Ts_rotation=Ts_translation/NumberOfIterationsOfInterloop;
horizon_rotation=10;
horizon_rotation_control=1;
Q_scalar_rotation=25;
R_scalar_rotation=0.1;
alpha_rotation=1.05;
lambda_rotation=0.96;
max_phi=pi/2;
min_phi=-max_phi;
max_theta=pi/2;
min_theta=-max_theta;
max_psi=pi;
min_psi=0;
max_vel_phi=0.8;
min_vel_phi=-max_vel_phi;
max_vel_theta=0.8;
min_vel_theta=-max_vel_theta;
max_vel_psi=1;
min_vel_psi=-max_vel_psi;
max_abs_accelration_phi=18.5*la;
max_abs_accelration_theta=max_abs_accelration_phi;
max_abs_accelration_psi=1;

% MPC rotation system matrices
[A_rotation,B_rotation,B_rotation_noise]=get_rotation_model(g,Ts_rotation,Ix,Iy,Iz);
dimenstion_of_state_vector_rotation=length(A_rotation);
size_B_rotation=size(B_rotation);
dimenstion_of_input_vector_rotation=size_B_rotation(2);
[A_rotation_augmented,B_rotation_augmented,B_rotation_noise_augmented,C_rotation_augmented]=get_augmented_model(A_rotation,B_rotation,B_rotation_noise,C);
if ~check_if_obsrvable_and_controlable(A_rotation_augmented,B_rotation_augmented,C_rotation_augmented)
    error('System not obsevable or not controlable')
end
[F_rotation,H_rotation] = get_prediction_matrices(A_rotation_augmented,B_rotation_augmented,alpha_rotation,horizon_rotation,horizon_rotation_control);
[F_rotation_normal,H_rotation_normal] = get_prediction_matrices(A_rotation_augmented,B_rotation_augmented,1,horizon_rotation,horizon_rotation_control);

% MPC rotation cost function

[Q_full_rotation,R_full_rotation]=get_full_weight_matrices(A_rotation_augmented,B_rotation_augmented,Q_scalar_rotation,R_scalar_rotation,[0,0,0,0,0,0,1,1,1],lambda_rotation,alpha_rotation,horizon_rotation,horizon_rotation_control);
get_cost_function_rotation = @(u,x0,trajectory) reference_cost_function(u,x0,trajectory,F_rotation,H_rotation,Q_full_rotation,R_full_rotation,horizon_rotation);
%Hessian=H_rotation'*Q_full_rotation*H_rotation+R_full_rotation;
%Linear= @(x0) H_rotation'*Q_full_rotation*F_rotation*x0;

% MPC rotation state constraints
state_constraints_A_rotation=zeros(12*horizon_rotation,dimenstion_of_input_vector_rotation*horizon_rotation_control);
state_constraints_b_helper_rotation=zeros(12*horizon_rotation,dimenstion_of_augmented_state_vector_translation);

for i=1:horizon_rotation
    delta_x_i=H_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i=H_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    state_constraints_A_rotation((i-1)*12+(1:3),:)=H_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(7:9),:);
    state_constraints_A_rotation((i-1)*12+(4:6),:)=delta_x_i/Ts_rotation+delta_v_i;
    state_constraints_A_rotation((i-1)*12+(7:12),:)=-state_constraints_A_rotation((i-1)*12+(1:6),:);
    delta_x_i_x0=F_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i_x0=F_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    state_constraints_b_helper_rotation((i-1)*12+(1:3),:)=-F_rotation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(7:9),:);
    state_constraints_b_helper_rotation((i-1)*12+(4:6),:)=-delta_x_i_x0/Ts_translation-delta_v_i_x0;
    state_constraints_b_helper_rotation((i-1)*12+(7:12),:)=-state_constraints_b_helper_rotation((i-1)*12+(1:6),:);
end
Get_state_constraints_b_rotation= @(x0) state_constraints_b_helper_rotation*x0+repmat([max_phi;max_theta;max_psi;max_vel_phi;max_vel_theta;max_vel_psi;...
    -min_phi;-min_theta;-min_psi;-min_vel_phi;-min_vel_theta;-min_vel_psi],horizon_rotation,1);

% MPC rotation input constraints
input_constraints_A_rotation=zeros(6*horizon_rotation_control,dimenstion_of_input_vector_rotation*horizon_rotation_control);
input_constraints_b_helper_rotation=zeros(6*horizon_rotation_control,3);
for i=1:horizon_rotation_control
    for j=1:i
        input_constraints_A_rotation((i-1)*6+(1:3),3*(j-1)+(1:3))=eye(3);
        input_constraints_A_rotation((i-1)*6+(4:6),3*(j-1)+(1:3))=-eye(3);
    end
    input_constraints_b_helper_rotation((i-1)*6+(1:3),:)=-eye(3);
    input_constraints_b_helper_rotation((i-1)*6+(4:6),:)=eye(3);
end
Get_input_constraints_b_rotation= @(u_previous) input_constraints_b_helper_rotation*u_previous+repmat([max_abs_accelration_phi; max_abs_accelration_theta; max_abs_accelration_psi],horizon_rotation_control*2,1);


%% The simulation loop
for i=1:numberOfIterations
    %% Find the next control input from transletioan MPC
    % Prepare cost function and constraints given previous input and current state
    %f_translation=Get_linear_component_translation(x);
    A_constraint_translation=[velocity_constraints_A;input_constraints_A];
    velocity_constraints_b=Get_velocity_constraints_b(x_translation);
    input_constraints_b=Get_input_constraints_b(previous_input_translation);
    b_constraint_translation=[velocity_constraints_b;input_constraints_b];
    % Solve convex program -> Change it later into quadratic program
    %inputs = quadprog(Hessian_translation,f_translation,A_constraint_translation,b_constraint_translation);
    cost_function_translation=@(u) get_cost_function_translation(u,x_translation,trajectory_generator(i*Ts_translation,x,Ts_translation,horizon_translation));
    inputs = fmincon(cost_function_translation,zeros(horizon_translation_control*dimenstion_of_input_vector_translation,1)...
        ,A_constraint_translation,b_constraint_translation,[],[],[],[],[],options);
    x_translation=A_translation_augmented*x_translation+B_translation_augmented*inputs(1:3);
    x_translation_history(:,i+1)=x_translation;
    continue
    % Analzye input
    u_x = inputs(1:3:end);
    u_y = inputs(2:3:end);
    u_z = inputs(3:3:end);
    U1 = u_x.^2+u_y.^2++u_z.^2+0.001;
    % Create reference for rotation MPC
    ref_phi =-asin(u_y./U1);
    ref_theta=asin(u_x./(U1.*cos(ref_phi)));
    ref_phi_der=(ref_phi-[x_rotation(1);ref_phi(1:(end-1))])/Ts_translation; % Check if dimensions are ok later
    ref_theta_der=(ref_theta-[x_rotation(2);ref_theta(1:(end-1))])/Ts_translation;
    reference=[ref_phi,ref_theta,zeros(size(ref_phi)),ref_phi_der,ref_theta_der,zeros(size(ref_phi))];
    % MPC Rotation
    for j=1:NumberOfIterationsOfInterloop
        % Prepare cost function and constraints given previous input and current state
        A_constraint_rotation=[state_constraints_A_rotation;input_constraints_A_rotation];
        state_constraints_b_rotation =Get_state_constraints_b_rotation(x([4:6,10:12]));
        input_constraints_b_rotation=Get_input_constraints_b_rotation(previous_input_translation);
        b_constraint_rotation=[state_constraints_b_rotation;input_constraints_b_rotation];
        % Solve convex program -> Change it later into quadratic program
        cost_function_rotation=@(u) get_cost_function_rotation(u,x([4:6,10:12]),trajectory_rotation_generator(reference,j,horizon_rotation,NumberOfIterationsOfInterloop));
        inputs_rotation = fmincon(@cost_function_rotation,zeros(horizon_translation_control*dimenstion_of_input_vector_rotation)...
        ,A_constraint_rotation,b_constraint_rotation,options);
        u=[U1(1),inputs_rotation(1:3)];

        % apply control input %(Move later into noise)
        [x,y]= quadrotor_discrete_linerized_model(u,state,C,g,var_system,var_measurment,mCenter+mAcatator,Ix,Iy,Iz);
        %tspan = [0 0.1];
        %[t,state_tra]=ode45(Simulation_model,[0 0.1], x);
        %x=state_tra(end,:);
    
        xHistory(i+1,:)=x;
        %estimate output
        if noise_flag %TODO change x into x_estimated
            [x_estimated,P]=Kalman(u,x_estimated,y,P,Predictive_model);
        else
            x_estimated=x;
        end
    end
   

end

%% Plot simulation reults
figure 
plot([1:(numberOfIterations+1)],x_translation_history(7,:))
hold on
plot([1:(numberOfIterations+1)],x_translation_history(8,:))
plot([1:(numberOfIterations+1)],x_translation_history(9,:))

