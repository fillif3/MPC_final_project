function trajectory = sinus_trajectory(t,state,ts,horizon)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
trajectory=zeros(9,horizon);
for i=1: horizon
    trajectory(7,i)=t;%cos(t);
    trajectory(8,i)=t;%sin(t);
    trajectory(9,i)=t;
    t=t+ts;
end
end