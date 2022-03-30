function [velocity_constraints_A,Get_velocity_constraints_b]=prepare_constraints_translation_veloctiy(F,H,horizon,max_v)
size_f=size(F);
number_of_states=size_f(2);
velocity_constraints_b_helper=zeros(number_of_states*horizon,number_of_states);
velocity_constraints_A=H;
for i=1:number_of_states*horizon
    if mod(i-1,number_of_states)<number_of_states/2
        velocity_constraints_A(i,:)=-H(i+number_of_states/2,:);
    end
end
for i=1:horizon
    velocity_constraints_b_helper((i-1)*6+(1:3),:)=F((i-1)*number_of_states+(4:6),:);
    velocity_constraints_b_helper((i-1)*6+(4:6),:)=-velocity_constraints_b_helper((i-1)*6+(1:3),:);
end
Get_velocity_constraints_b= @(x0) velocity_constraints_b_helper*x0+max_v*ones([6*horizon,1]);
end