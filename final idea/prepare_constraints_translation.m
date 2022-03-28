function[constraints_A,get_constraints_b]=prepare_constraints_translation(F,H,horizon,max_distance,max_v,max_f,max_df)

[velocity_position_A,get_position_constraints_b]=prepare_constraints_translation_position(F,H,horizon,max_distance);


% MPC translation state (velocity) constraints
[velocity_constraints_A,get_velocity_constraints_b]=prepare_constraints_translation_veloctiy(F,H,horizon,max_v);

% MPC translation input (accelration) constraints
size_h=size(H);
number_of_inputs=size_h(2);
[input_constraints_A,input_constraints_b]=prepare_constraints_translation_input(horizon,number_of_inputs,max_f);



% MPC translation input (accelration) constraints
[input_diff_constraints_A,Get_diff_input_constraints_b]=prepare_constraints_translation_input_dffrence(horizon,number_of_inputs,max_df);
constraints_A=[velocity_position_A;velocity_constraints_A;input_constraints_A;input_diff_constraints_A];


get_constraints_b= @(u_previous,x0) [get_position_constraints_b(x0);get_velocity_constraints_b(x0);...
    input_constraints_b;Get_diff_input_constraints_b(u_previous)];

end