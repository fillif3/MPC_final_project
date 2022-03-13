function [Ix,Iy,Iz] = getInterias(r,l,mCenter,mAcuators)
%GETINTERIAS Summary of this function goes here
%   Detailed explanation goes here
spehereInteria= 0.4*r^2*mCenter;
Ix=spehereInteria+2*mAcuators*l^2;
Iy=Iz;
Iz=Ix+2*mAcuators*l^2;
end

