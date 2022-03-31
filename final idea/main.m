clear; clc
rng=(1);
noise_flag=false;
measruable_state=false;
warning('off');
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
Ts_translation=0.1;
horizon_translation=10;
treshold_translation=0.1;
Q_scalar_translation=20;
R_scalar_translation=0.1;
max_distance_from_waypoint=1.5;%max_linear_velocity*Ts_global;
max_linear_velocity=20;
max_linear_force=pi/2;
max_linear_force_difference=max_linear_force/10; %Changed -> test

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

[Q_full_translation,R_full_translation,Q_matrix_translation,R_matrix_translation]=get_full_weight_matrices(A_translation,B_translation,Q_scalar_translation,R_scalar_translation,[1,1,1,1,1,1],horizon_translation);
get_cost_function_translation = @(u,x_0,waypoint) reference_cost_function(u,x_0,waypoint,A_translation,B_translation,Q_matrix_translation,R_matrix_translation);
% Get quadratic cost function
%Hessian_translation=H_translation'*Q_full_translation*H_translation+R_full_translation;
%Get_linear_component_translation= @(x0) H_translation'*Q_full_translation*F_translation*x0;

[constraints_translation_A,get_constraints_translation_b]=prepare_constraints_translation(F_translation,H_translation,horizon_translation,max_distance_from_waypoint,max_linear_velocity,max_linear_force,max_linear_force_difference);
constraints_Aeq_translation=H_translation((end-5):end,:);
%constraints_beq_translation=zeros(6,1);

%% Preaper trajectory
horizon_global=15;
Ts_global= 0.5;%(horizon_translation-1)*Ts_translation;
[A_global,B_global,~]=get_trasnlation_model(g,Ts_global,m);
obstacles=[];%{[5,5,0],[3,7,0],[4,6,0],[6,4,0],[7,3,0]};
treshold=2*max_distance_from_waypoint;
goal=[10;10;0];
[~,~,Q_matrix_global,~]=get_full_weight_matrices(A_translation,B_translation,Q_scalar_translation,R_scalar_translation,[1,1,1,0,0,0],horizon_translation);
[trajectory_global,inputs_global] = find_global_path(goal,A_global,B_global,C,Q_matrix_global,R_matrix_translation,obstacles,max_linear_velocity/10,max_linear_force/2,max_linear_force_difference*3.5, horizon_global,treshold);


%% Prepare Rotation MPC
% MPC Rotation parameters
NumberOfIterationsOfInterloop=10;
Ts_rotation=0.01;%Ts_translation/NumberOfIterationsOfInterloop; 
horizon_rotation=10;
Q_scalar_rotation=25;
R_scalar_rotation=0.1;
max_angles=[pi/2,pi/2,pi]';
min_angles=[-pi/2 -pi/2 0]';
max_angles_abs_vel=[0.8,0.8,1]';
max_angles_abs_acc=[18.5*la,18.5*la,1]';

% MPC rotation system matrices
[A_rotation,B_rotation,B_rotation_noise]=get_rotation_model(g,Ts_rotation,Ix,Iy,Iz);
[number_of_states_rotation,number_of_inputs_rotation]=size(B_rotation);
if ~check_if_obsrvable_and_controlable(A_rotation,B_rotation,C)
    error('System not obsevable or not controlable')
end
[F_rotation,H_rotation] = get_prediction_matrices(A_rotation,B_rotation,horizon_rotation);
% MPC rotation cost function

[Q_full_rotation,R_full_rotation,Q_matrix_rotation,R_matrix_rotation]=get_full_weight_matrices(A_rotation,B_rotation,Q_scalar_rotation,R_scalar_rotation,[1,1,1,1,1,1],horizon_rotation);

% MPC rotation state constraints
[constraints_rotation_A,get_constraints_rotation_b]=prepare_constraints_rotation(F_rotation,H_rotation,horizon_rotation,max_angles,min_angles,max_angles_abs_vel,max_angles_abs_acc);
constraints_Aeq_rotation=H_rotation((end-5):end,:);



%% Prepare simulation and objective
% Simulation parameters
numberOfIterations=10000;
options = optimoptions('quadprog','Display','off','Algorithm','active-set');


% define starting state and trajectory
x=zeros(12,1);
x_estimation=x;
% Translation managment
x_translation=zeros(6,1);
x_translation_estimation=x_translation;
x_translation_history=zeros(6,numberOfIterations+1);
u_previous_translation=zeros(3,1);
u_traslation_hisotry=zeros(3,numberOfIterations);
previous_input_translation=[0;0;0];
error_position_history=zeros(3,numberOfIterations);
x_estimated_translation=x_translation;
xHistory=zeros(12,numberOfIterations+1);
% Rotation managment
x_rotation=zeros(6,1);
x_rotation_estimation=x_rotation;
x_rotation_history=zeros(6,numberOfIterations+1);%*number_of_innerloop
u_previous_rotation=zeros(3,1);
u_rotation_hisotry=zeros(3,numberOfIterations);
error_position_history=zeros(3,numberOfIterations);
x_estimated_rotation=x_rotation;



% Make a simulation
%horizon_global=15;
waypoint_order_number=1;
previous_waypoint=[];
current_waypoint = [trajectory_global(:,waypoint_order_number)];
interpolation=false;
last_waypoint=[trajectory_global(1:3,horizon_global);zeros(3,1)];
constraints_beq_translation=current_waypoint;
inputs=zeros(horizon_translation*dimenstion_of_input_vector_translation,1);

for i=1:numberOfIterations
%         If system is close to the next waypoint, 
    while ((norm(x_estimated_translation(1:3)-current_waypoint(1:3))<(max_distance_from_waypoint/2))&&(waypoint_order_number<=horizon_global))
        if waypoint_order_number>=horizon_global
            waypoint_order_number=waypoint_order_number+1;
            previous_waypoint=current_waypoint;
            current_waypoint=last_waypoint;
            break
        end
        if interpolation
            interpolation=false;
        else
            waypoint_order_number=waypoint_order_number+1;
        end
        previous_waypoint=current_waypoint;
        current_waypoint = trajectory_global(:,waypoint_order_number);
        
    end
    if norm(x_estimated_translation(:)-last_waypoint(:))<treshold_translation
            break
    end
    if mod(i,22)==0
        disp(current_waypoint)
    end
    while true
        current_waypoint_without_velicoty=[current_waypoint(1:3);zeros(3,1)];
        
        x_estimatet_translation_shifted=x_estimated_translation-current_waypoint_without_velicoty;
        b_constraints_tranlsation=get_constraints_translation_b(u_previous_translation,x_estimatet_translation_shifted);

        constraints_beq_translation=-F_translation((horizon_translation-1)*length(A_translation)+(1:length(A_translation)),:)*x_estimatet_translation_shifted;
        %cost_function_translation =@(u) get_cost_function_translation(u,x_estimated_translation,current_waypoint);
    %     [inputs,~,exitflag,~] = fmincon(cost_function_translation,zeros(horizon_translation*3,1)...
    %         ,constraints_translation_A,b_constraints_tranlsation,constraints_Aeq_translation,...
    %         constraints_beq_translation,[],[],[],options);
        Hes=H_translation'*Q_full_translation*H_translation+R_full_translation;
        grad=2*x_estimatet_translation_shifted'*F_translation'*Q_full_translation*H_translation;
        
        [inputs,~,exitflag,message]=quadprog(Hes,grad,constraints_translation_A,b_constraints_tranlsation,...
            constraints_Aeq_translation,constraints_beq_translation,[],[],[inputs(4:end);zeros(3,1)],options);
        if isempty(inputs)
            interpolation=true;
            current_waypoint=(current_waypoint+previous_waypoint)/2;
        else
            break
        end
    end
    %disp(exitflag)
    next_input_translation=inputs(1:3);
    previous_input_translation=next_input_translation+previous_input_translation;
    x_estimated_translation=A_translation*x_estimated_translation+B_translation*next_input_translation;
    x_translation_history(:,i+1)=x_estimated_translation;
    u_traslation_hisotry(:,i)=previous_input_translation;


end
toc

% % Plot results
% plot trajectory
plot3(trajectory_global(1,:),trajectory_global(2,:),trajectory_global(3,:),'o');
axis([-1 max(goal) -1 max(goal) -1 max(goal)])
hold on
% plot obstacles
[X,Y,Z] = sphere;
for j=obstacles
    o=j{1};
    surf(X*treshold+o(1),Y*treshold+o(2),Z*treshold+o(3))
end
plot3(x_translation_history(1,1:i),x_translation_history(2,1:i),x_translation_history(3,1:i));

% %Plot planned velocities
% figure
% plot([1:horizon_global],trajectory_global(4,:));
% hold on
% plot([1:horizon_global],trajectory_global(5,:));
% plot([1:horizon_global],trajectory_global(6,:));

%Plot real velocities
figure
plot([1:i],x_translation_history(4,1:i));
hold on
plot([1:i],x_translation_history(5,1:i));
plot([1:i],x_translation_history(6,1:i));

% 
% %Plot inputs
% figure
% plot([1:horizon_global],inputs_global(1:3:end));
% hold on
% plot([1:horizon_global],inputs_global(2:3:end));
% plot([1:horizon_global],inputs_global(3:3:end));
