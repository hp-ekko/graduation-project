% load point cload
grname=strcat('../Data/PointCloud/260.mat');
load(grname)
Ground(:,1) = -Ground(:,1);
Ground(:,2) = -Ground(:,2);

% extract x,y coordinate
idx = find( (Ground(:,2)>2) .* (Ground(:,2)<10) );
pts = Ground(idx,[1,2]);

X = pts';
[a1,s1] = LineFitRANSAC(X,10000,1); % 6 is the condition to be inlier    
% dist = abs(a1(1:2)'*Ground(:,1:2)'-1)/norm(a1(1:2));
v1 = sqrt(var(dist));
idx = (Ground(:,1:2)*a1(1:2) > -a1(3));% | (Ground(:,1:2)*a2(1:2) > -a2(3));
pts1 =  Ground(idx,1:3);
idx = (Ground(:,1:2)*a1(1:2) < -a1(3));% & (Ground(:,1:2)*a2(1:2) < -a2(3));
pts2 =  Ground(idx,1:3);
figure(1), hold on
scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'b.')
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),'g.')
fplot(@(x) (-a1(3)-a1(1)*x)/a1(2), 'r');
% axis equal
xlabel('x'), ylabel('y'), zlabel('z')