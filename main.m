rng=(1);
noise_flag=false;
%define constrains
mCenter=2;
mAcatator=0.2;
l=0.5;
r=0.1;
g=9.81;
Ts=0.1;
Horizon=10;
[Ix,Iy,Iz]=getInterias(r,l,mCenter,mAcatator);
numberOfIterations=500;
C=eye(12);
C=C(1:6,:);
P=eye(12)*0.01;
x_estimated=x;

% define starting state and trajectory
x=zeros(12,1);
xHistory=zeros(12,numberOfIterations+1);
trajectory=getTrajectory(numberOfIterations);
% In the loop
for i=1:numberOfIterations
    % compute control input
    Predictive_model=@(u) quadrotor_discrete_linerized_model(u,x_estimated,Ts,g,Ix,Iy,Iz);
    u=MPC(Predictive_model,trajectory(i,:),Horizon);
    % apply control input
    Simulation_model=@(t,state) quadrotor_continous_nonlinear_model(u,state,g,mCenter+mAcatator,Ix,Iy,Iz);
    tspan = [0 0.1];
    [t,state_tra]=ode45(Simulation_model,[0 0.1], x);
    x=state_tra(end,:);
    if noise_flag
        x=add_gauss_noise(x,Q,Ts);
    end
    y=C*x
    if noise_flag
        y=add_gauss_noise(y,R,Ts);
    end
    xHistory(i+1,:)=x;
    %estimate output
    if noise_flag
        [x_estimated,P]=Kalman(u,x_estimated,y,P,Predictive_model);
    else
        x_estimated=x;
    end
   

end

%plot reults