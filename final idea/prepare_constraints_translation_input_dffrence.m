function [constraints_A_accelration_diff,get_constraints_b_accelration_diff]=prepare_constraints_translation_input_dffrence(horizon,number_of_inputs,max_a_diff)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
constraints_A_accelration_diff=([eye(number_of_inputs*horizon)-diag(ones(number_of_inputs*(horizon-1),1),-number_of_inputs);-eye(number_of_inputs*horizon)+diag(ones(number_of_inputs*(horizon-1),1),-number_of_inputs)]);
constraints_b_accelration_diff=max_a_diff*ones(2*number_of_inputs*(horizon+1),1);
deacclration_a_constraint = [zeros(3,number_of_inputs*(horizon-1)),eye(3);-zeros(3,number_of_inputs*(horizon-1)),-eye(3)];
constraints_A_accelration_diff=[constraints_A_accelration_diff;deacclration_a_constraint];
get_constraints_b_accelration_diff= @(u_previous) constraints_b_accelration_diff+[[eye(number_of_inputs);...
    zeros((horizon-1)*number_of_inputs,number_of_inputs);-eye(number_of_inputs);...
    zeros((horizon-1)*number_of_inputs,number_of_inputs)]*u_previous;zeros(6,1)];

end