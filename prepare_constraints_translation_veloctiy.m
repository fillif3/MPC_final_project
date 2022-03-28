function [velocity_constraints_A,Get_velocity_constraints_b]=prepare_constraints_translation_veloctiy(F_translation_normal,H_translation_normal,horizon_translation,horizon_translation_control,dimenstion_of_input_vector_translation,max_linear_velocity,Ts_translation)

velocity_constraints_A=zeros(6*horizon_translation,dimenstion_of_input_vector_translation*horizon_translation_control);
dimenstion_of_augmented_state_vector_translation=9;
velocity_constraints_b_helper=zeros(6*horizon_translation,dimenstion_of_augmented_state_vector_translation);

for i=1:horizon_translation
    delta_x_i=H_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i=H_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    velocity_constraints_A((i-1)*6+(1:3),:)=delta_x_i/Ts_translation+delta_v_i;
    velocity_constraints_A((i-1)*6+(4:6),:)=-delta_x_i/Ts_translation-delta_v_i;
    delta_x_i_x0=F_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(1:3),:);
    delta_v_i_x0=F_translation_normal((i-1)*dimenstion_of_augmented_state_vector_translation+(4:6),:);
    velocity_constraints_b_helper((i-1)*6+(1:3),:)=-delta_x_i_x0/Ts_translation-delta_v_i_x0;
    velocity_constraints_b_helper((i-1)*6+(4:6),:)=delta_x_i_x0/Ts_translation+delta_v_i_x0;
end
Get_velocity_constraints_b= @(x0) velocity_constraints_b_helper*x0+max_linear_velocity*ones([6*horizon_translation,1]);
end