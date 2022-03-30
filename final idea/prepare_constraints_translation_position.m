function [constraints_position_A,get_position_constraints_b]=prepare_constraints_translation_position(F,H,horizon,max_distance)
size_f=size(F);
number_of_states=size_f(2);
position_constraints_b_helper=zeros(number_of_states*horizon,number_of_states);
constraints_position_A=H;
for i=1:number_of_states*horizon
    if mod(i-1,number_of_states)>=number_of_states/2
        constraints_position_A(i,:)=-H(i-number_of_states/2,:);
    end
end
for i=1:horizon
    position_constraints_b_helper((i-1)*6+(1:3),:)=-F((i-1)*number_of_states+(1:3),:);
    position_constraints_b_helper((i-1)*6+(4:6),:)=-position_constraints_b_helper((i-1)*6+(1:3),:);
end
get_position_constraints_b= @(x0) position_constraints_b_helper*x0+max_distance*ones([6*horizon,1]);

end