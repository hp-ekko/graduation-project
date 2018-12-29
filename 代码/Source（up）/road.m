%Road: Robust Plane Fitting with RANSAC
% find ground points
err = 0.1;
idx = (Ground(:,1:2)*a1(1:2) < -a1(3)) & (Ground(:,1:2)*a2(1:2) < -a2(3));
X = Ground(idx,1:3);
[a3,s3] = PlaneFitRANSAC(X',10000,err); % 6 is the condition to be inlier    
% dist = abs(a3(1:3)'*Ground(:,1:3)'-1)/norm(a3(1:3));
dist = abs(a3(1:3)'*X(:,1:3)'-1)/norm(a3(1:3));
idx_r = dist'<err;


pts_r = X(idx_r,1:3);
figure(1), hold on
pts_n = X(~idx_r,1:3);
scatter3(pts_n(:,1),pts_n(:,2),pts_n(:,3),'k.') % back ground class
scatter3(pts_r(:,1),pts_r(:,2),pts_r(:,3),'b.') % road class
% fplot(@(x) (-a11(3)-a11(1)*x)/a11(2), 'r');
axis equal

