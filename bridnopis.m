x_tra2=[];
x_state=x0';
for i=1:8
    if i==1
        x_state=A_translation_augmented/alpha_translation*x_state+B_translation_augmented/alpha_translation*du';
    else
          x_state=A_translation_augmented/alpha_translation*x_state;
    end
    x_tra2(((i-1)*9+1):((i*9)))=x_state;
end
x_tra2=x_tra2';
