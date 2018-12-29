%% distribution in x-y plane
grname=strcat('../Data/PointCloud/128.mat');
load(grname)
s = 3; % scale
Z = zeros(160*s,55*s);
x_off = -80;
y_off = -35;
for k=1:length(Ground)
    Z(round(Ground(k,1)*s-x_off*s),round(Ground(k,2)*s-y_off*s)) = ...
        Z(round(Ground(k,1)*s-x_off*s),round(Ground(k,2)*s-y_off*s)) + 1;
end
surf(log(1+Z))
axis equal
colormap summer
shading interp