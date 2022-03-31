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
numberOfIterations=30;
options = optimoptions('quadprog','Display','off');


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





ref_angle=-[1,1,0,0,0,0]'*0.1;
for i=1:numberOfIterations
%         If system is close to the next waypoint, 

    while true
        x_estimatet_rotation_shifted=x_estimated_rotation-ref_angle;
        b_constraints_rotation=get_constraints_rotation_b(x_estimatet_rotation_shifted);
        constraints_beq_rotation=-F_rotation((horizon_rotation-1)*length(A_rotation)+(1:length(A_rotation)),:)*x_estimatet_rotation_shifted;
        %cost_function_translation =@(u) get_cost_function_translation(u,x_estimated_translation,current_waypoint);
    %     [inputs,~,exitflag,~] = fmincon(cost_function_translation,zeros(horizon_translation*3,1)...
    %         ,constraints_translation_A,b_constraints_tranlsation,constraints_Aeq_translation,...
    %         constraints_beq_translation,[],[],[],options);
        Hes_rotation=H_rotation'*Q_full_rotation*H_rotation+R_full_rotation;
        grad_rotation=2*x_estimatet_rotation_shifted'*F_rotation'*Q_full_rotation*H_rotation;
        
        [inputs,~,exitflag]=quadprog(Hes_rotation,grad_rotation,constraints_rotation_A,b_constraints_rotation,...
            constraints_Aeq_rotation,constraints_beq_rotation,[],[],[],options);
        if isempty(inputs)
            interpolation=true;
            current_waypoint=(current_waypoint+previous_waypoint)/2;
        else
            break
        end
    end
    %disp(exitflag)
    next_input_rotation=inputs(1:3);
    x_estimated_rotation=A_rotation*x_estimated_rotation+B_rotation*next_input_rotation;
    x_rotation_history(:,i+1)=x_estimated_rotation;
    u_rotation_hisotry(:,i)=next_input_rotation;


end



% Plot rotation position
figure
plot([1:i],x_rotation_history(1,1:i));
hold on
plot([1:i],x_rotation_history(2,1:i));
plot([1:i],x_rotation_history(3,1:i));
% Plot rotation velocity
figure
plot([1:i],x_rotation_history(4,1:i));
hold on
plot([1:i],x_rotation_history(5,1:i));
plot([1:i],x_rotation_history(6,1:i));
% Plot rotation inputs
% 
%Plot inputs
figure
plot([1:i],u_rotation_hisotry(1:3:end));
hold on
plot([1:i],u_rotation_hisotry(2:3:end));
plot([1:i],u_rotation_hisotry(3:3:end));
