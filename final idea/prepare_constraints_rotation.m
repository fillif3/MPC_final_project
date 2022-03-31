function[constraints_A,get_constraints_b]=prepare_constraints_rotation(F,H,horizon,max_angles,min_angles,max_angles_v,max_angles_f)

[velocity_position_A,get_position_constraints_b]=prepare_constraints_rotation_position(F,H,horizon,max_angles,min_angles);


% MPC translation state (velocity) constraints
[velocity_constraints_A,get_velocity_constraints_b]=prepare_constraints_rotation_veloctiy(F,H,horizon,max_angles_v);

% MPC translation input (accelration) constraints
size_h=size(H);
number_of_inputs=size_h(2)/horizon;
[input_constraints_A,input_constraints_b]=prepare_constraints_rotation_input(horizon,number_of_inputs,max_angles_f);

constraints_A=[velocity_position_A;velocity_constraints_A;input_constraints_A];

get_constraints_b= @(x0) [get_position_constraints_b(x0);get_velocity_constraints_b(x0);...
    input_constraints_b];

end