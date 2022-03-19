function trajectory = trajectory_rotation_generator(reference,iteration,horizon,NumberOfIterationsOfInterloop)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
trajectory=zeros(length(reference),horizon);
for i=iteration:(iteration+horizon)
    trajectory(:,i)=reference(:,ceil(i/NumberOfIterationsOfInterloop)); %Check if dimensions are fine
end