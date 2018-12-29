% Refine Right Bound
pts0 =  Ground(~idx_r,1:3);

dist = abs(a1(1:2)'*pts0(:,1:2)'-1)/norm(a1(1:2));
idx1 = dist<4;
[a11,s] = LineFitRANSAC(pts0(idx1,1:2)',10000,0.5); % 6 is the condition to be inlier    
dist = abs(a2(1:2)'*pts0(:,1:2)'-1)/norm(a2(1:2));
idx2 = dist<4;
[a21,s] = LineFitRANSAC(pts0(idx2,1:2)',10000,0.5); % 6 is the condition to be inlier    

figure(1), hold on
scatter3(pts0(~(idx1|idx2),1),pts0(~(idx1|idx2),2),pts0(~(idx1|idx2),3),'b.') % back ground class
scatter3(pts0(idx1|idx2,1),pts0(idx1|idx2,2),pts0(idx1|idx2,3),'k.') % back ground
fplot(@(x) (-a1(3)-a1(1)*x)/a1(2), 'r');
fplot(@(x) (-a11(3)-a11(1)*x)/a11(2), 'g');
fplot(@(x) (-a2(3)-a2(1)*x)/a2(2), 'r');
fplot(@(x) (-a21(3)-a21(1)*x)/a21(2), 'g');
axis equal

