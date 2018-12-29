%% greedy search
% load point cload
grname=strcat('../Data/PointCloud/128.mat');
load(grname)
Ground(:,1) = -Ground(:,1);
Ground(:,2) = -Ground(:,2);

%% parameter and output
stepSize = 0.05;
theta = 0.00001;
dist_p = 0.2;

a1 = []; % right boundary 
a2 = []; % left boundary 

%% initialize h var mean theta  先初始化一条线，然后根据贪心算法，逐步右移找到边界点（边界线）
a0 = [sin(theta);cos(theta);0]; % center boundary;
v1 = []; h1 = []; p = [0;0;-1];
for k=1:300
a = a0 + p*stepSize;
dist = abs(a(1:2)'*Ground(:,1:2)'+a(3))/norm(a);%norm函数返回的是最大奇异值
idx = find(dist<dist_p);%距离小于阈值，找出这部分点
height = Ground(idx,3);%找到点的z坐标
h1 = [h1,mean(height)];
v1 = [v1,var(height)];
if isBound(h1,v1,stepSize,1)
    a1 = a0;
    break
else
    a0 = a;
end
end
a0 = [sin(theta);cos(theta);0]; % center boundary;
v2 = []; h2 = []; p = [0;0;1];
for k=1:300
a = a0 + p*stepSize;
dist = abs(a(1:2)'*Ground(:,1:2)'+a(3))/norm(a);
idx = find(dist<dist_p);
height = Ground(idx,3);
h2 = [h2,mean(height)];
v2 = [v2,var(height)];
if k==299%isBound(h2,v2,stepSize,1)
    a2 = a0;
    break
else
    a0 = a;
end
end
figure(1)

subplot(3,1,1), hold on, axis equal
idx = (Ground(:,1:2)*a1(1:2) > -a1(3)) | (Ground(:,1:2)*a2(1:2) < -a2(3));
pts1 =  Ground(idx,1:3);
idx = (Ground(:,1:2)*a1(1:2) < -a1(3)) & (Ground(:,1:2)*a2(1:2) > -a2(3));
pts2 =  Ground(idx,1:3);

scatter3(pts1(:,1),pts1(:,2),pts1(:,3),'b.')
scatter3(pts2(:,1),pts2(:,2),pts2(:,3),'g.')
fplot(@(x) (-a1(3)-a1(1)*x)/a1(2), 'r');
fplot(@(x) (-a2(3)-a2(1)*x)/a2(2), 'r');

subplot(3,2,3)
plot(stepSize*(1:length(h1)),h1)
xlabel('mean(h1)')
subplot(3,2,4)
plot(stepSize*(1:length(h1)),v1)
xlabel('var(h1)')
subplot(3,2,5)
plot(stepSize*(1:length(h2)),h2)
xlabel('mean(h2)')
subplot(3,2,6)
plot(stepSize*(1:length(h2)),v2)
xlabel('var(h2)')

function b = isBound(h,v,s,t)
    if length(h)<1/s
        b = false;
        return
    end
    b = h(end)>mean(h)+t*sqrt(var(h));
end
%{
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
%}