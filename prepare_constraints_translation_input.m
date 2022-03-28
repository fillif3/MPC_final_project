function [input_constraints_A,Get_input_constraints_b]=prepare_constraints_translation_input(horizon_translation_control,dimenstion_of_input_vector_translation,max_linear_accelration);
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
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

end