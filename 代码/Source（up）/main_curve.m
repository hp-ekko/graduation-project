%% Road Detection
% By XXX
% update: try to detect curve bounds

%% Step 1: Load point cloud
% Here are three se
% 0,1,2: Cross: sparse, noisy. 
% 119 - 128
% 260,261,262,263: Bend: thin
 ID = 261; 
grname=['../Data/PointCloud/' num2str(ID) '.mat'];
load(grname)
Ground(:,1) = -Ground(:,1);
Ground(:,2) = -Ground(:,2);

%% Step 2: Right Bound: Greedy search
% parameter and output
stepSize = 0.05;
theta = 0.00001;
dist_p = 0.2;
a1 = []; a2 = []; % r/left boundary 
a0 = [sin(theta);cos(theta);0]; % center boundary;
v1 = []; h1 = []; p = [0;0;-1];
for k=1:300
a = a0 + p*stepSize;
dist = abs(a(1:2)'*Ground(:,1:2)'+a(3))/norm(a);
idx = find(dist<dist_p);
height = Ground(idx,3);
h1 = [h1,mean(height)];
v1 = [v1,var(height)];
if isBound(h1,v1,stepSize,2)
    a1 = a0;
    break
else
    a0 = a;
end
end

%% Step 3: Left Bound: Robust Line Fitting with RANSAC
% extract x,y coordinate
idx = find( (Ground(:,2)>-10) .* (Ground(:,2)<-6) );
pts = Ground(idx,[1,2]);
[a2,s2] = LineFitRANSAC(pts',10000,5); % 6 is the condition to be inlier    

%% Step 4: Road: Robust Plane Fitting with RANSAC
% find ground points
idx = (Ground(:,1:2)*a1(1:2) < -a1(3)) & (Ground(:,1:2)*a2(1:2) < -a2(3));
X = Ground(idx,1:3)';
[a3,s3] = PlaneFitRANSAC(X,10000,0.1); % 6 is the condition to be inlier    
dist = abs(a3(1:3)'*Ground(:,1:3)'-1)/norm(a3(1:3));
idx_r = dist'<0.1;
pts_r = Ground(idx_r,1:3);

%% Step 5: Refine Right Bound
pts0 =  Ground(~idx_r,1:3);

dist = abs(a1(1:2)'*pts0(:,1:2)'-1)/norm(a1(1:2));
idx1 = dist<4;
[a11,s] = CurveFitRANSAC(pts0(idx1,1:2)',10000,0.5); % 6 is the condition to be inlier    
dist = abs(a2(1:2)'*pts0(:,1:2)'-1)/norm(a2(1:2));
% to find the curve edge set, you should only trust point near the car
% x:y=1:8, means we trust point in a fat ellipse.
idx2 = dist<2;
idx2_d = (pts0(:,1).^2/64+pts0(:,2).^2)'<10^2;
[a21,s] = CurveFitRANSAC(pts0(idx2&idx2_d,1:2)',10000,0.5); % 6 is the condition to be inlier    

%% Step 6: Classify Points 
idx_g = ([Ground(:,1).^2,Ground(:,1:2)]*a11 > 1+0.5*norm(a11)) | ...
    ([Ground(:,1).^2,Ground(:,1:2),]*a21 > 1+0.5*norm(a21));
pts_g =  Ground(idx_g,1:3);
dist = abs([Ground(:,1).^2,Ground(:,1:2)]*a11-1)/norm(a11);
idx1 = dist<0.5;
dist = abs([Ground(:,1).^2,Ground(:,1:2)]*a21-1)/norm(a21);
idx2 = dist<0.5;
idx_b = idx1|idx2;
pts_b = Ground(idx_b,1:3);

pts_r = Ground(idx_r&~idx_g&~idx_b,1:3);

pts0 = Ground(~idx_r&~idx_g&~idx_b,1:3);

figure(1), clf
for k=1:2
    subplot(1,2,k), hold on
    scatter3(pts0(:,1),pts0(:,2),pts0(:,3),'k.') % back ground class
    scatter3(pts_b(:,1),pts_b(:,2),pts_b(:,3),'r.') % boundary class
    scatter3(pts_r(:,1),pts_r(:,2),pts_r(:,3),'b.') % road class
    scatter3(pts_g(:,1),pts_g(:,2),pts_g(:,3),'g.') % grass class
    fplot(@(x) (1-a11(2)*x-a11(1)*x.^2)/a11(3), [-60,60], 'r');
    fplot(@(x) (1-a21(2)*x-a21(1)*x.^2)/a21(3), [-60,60], 'r');
    % save image
    axis equal
    axis([-80,80,-30,30])
    xlabel('x'), ylabel('y'), zlabel('z')
    view(k+1)
end
saveas(gcf, ['curve_output' num2str(ID) '.png'])