rng=(1);
noise_flag=false;
measruable_state=true;
%% define drone parameters
m=0.74;
l=0.5;
r=0.1;
g=9.8;
Ix=4*10^(-3);
Iy=4*10^(-3);
Iz=8.4*10^(-3);
Ia=21;
b=29*10^(-6);
d=1.1*10^(-6);
Jr=455.*10^(-6);
C=eye(12);
if ~measruable_state
    C=C(1:6,:);
end
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
horizon_translation=8;
horizon_translation_control=1;
Q_scalar_translation=20;
R_scalar_translation=0.1;
alpha_translation=1.05;
lambda_translation=0.96;

% MPC translation system matrices
[A_translation,B_translation,B_translation_noise]=get_trasnlation_model(g,Ts_translation,m);
[A_translation_augmented,B_translation_augmented,B_translation_noise_augmented,C_translation_augmented]=get_augmented_model(A_translation,B_translation,B_translation_noise,C);
if ~check_if_obsrvable_and_controlable(A_translation_augmented,B_translation_augmented,C_translation_augmented)
    error('System not obsevable or not controlable')
end
[F_translation,H_translation] = get_prediction_matrices(A_translation_augmented,B_translation_augmented,alpha_translation,horizon_translation,horizon_translation_control);

% MPC translation cost function

[Q_full_translation,R_full_translation]=get_full_weight_matrices(A_translation_augmented,B_translation_augmented,Q_scalar_translation,R_scalar_translation,lambda_translation,alpha_translation);
Hessian=H_translation'*Q_full_translation*H_translation+R_full_translation;
Linear= @(x0) H_translation'*Q_full_translation*F_translation*x0;

% MPC translation constraints



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

% MPC rotation system matrices
[A_rotation,B_rotation,B_rotation_noise]=get_trasnlation_model(g,Ts_rotation);
[A_rotation_augmented,B_rotation_augmented,B_rotation_noise_augmented,C_rotation_augmented]=get_augmented_model(A_rotation,B_rotation,B_rotation_noise,C);
if ~check_if_obsrvable_and_controlable(A_rotation_augmented,B_rotation_augmented,C_rotation_augmented)
    error('System not obsevable or not controlable')
end
[F_rotation,H_rotation] = get_prediction_matrices(A_rotation_augmented,B_rotation_augmented,alpha_rotation,horizon_rotation,horizon_rotation_control);

% MPC rotation cost function

[Q_full_rotation,R_full_rotation]=get_full_weight_matrices(A_rotation_augmented,B_rotation_augmented,Q_scalar_rotation,R_scalar_rotation,lambda_rotation,alpha_rotation);
Hessian=H_rotation'*Q_full_rotation*H_rotation+R_full_rotation;
Linear= @(x0) H_rotation'*Q_full_rotation*F_rotation*x0;

% MPC rotation constraints


% Simulation parameters
numberOfIterations=5000;


% define starting state and trajectory
x=zeros(12,1);
x_estimated=x;
xHistory=zeros(12,numberOfIterations+1);
trajectory=getTrajectory(numberOfIterations);


%% The simulation loop
for i=1:numberOfIterations
    % MPC Translation 
    Predictive_model_transloation=@(u) quadrotor_discrete_linerized_model_translation(u,x_estimated,Ts,g,Ix,Iy,Iz);
    [U1,angle1Ref,angle2Ref]=MPC(Predictive_model_transloation,trajectory(i,:),horizaon_translation,horizon_translation_control);
    % MPC Rotation
    for j=1:NumberOfIterationsOfInterloop
        Predictive_model_rational=@(u) quadrotor_discrete_linerized_model_rational(u,x_estimated,Ts,g,Ix,Iy,Iz);
        u=MPC_rational(Predictive_model_rational,horizon_rational,horizon_rational_control);
        % apply control input
        [x,y]= quadrotor_discrete_linerized_model(u,state,C,g,var_system,var_measurment,mCenter+mAcatator,Ix,Iy,Iz);
        tspan = [0 0.1];
        [t,state_tra]=ode45(Simulation_model,[0 0.1], x);
        x=state_tra(end,:);
    
        xHistory(i+1,:)=x;
        %estimate output
        if noise_flag
            [x_estimated,P]=Kalman(u,x_estimated,y,P,Predictive_model);
        else
            x_estimated=x;
        end
    end
   

end

%% Plot simulation reults