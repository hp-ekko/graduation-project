function [a_best,max_score] = CurveFitRANSAC(pts,N,dist_p)
%CurveFitRANSAC Fit Curve using ransac
% we define an inlier if its disance is 
% smaller than dist_p of mean distance
M = size(pts,2);
K = 50;
a_best = [0,0]; max_score = 0;
for k = 1:N
    % sample
    idx = randsample(M,K);
    % compute line need X'*a = 1 <=> a1*x1+a2*x2=1
    % in the curve function, we assume a0*x1^2+a1*x1+a2*x2=1
    X = [pts(1,idx).^2;pts(1,idx);pts(2,idx)];   
    a = X'\ones(K,1);
    % a = [a;-1];
    % distance
    dist = abs(a'*[pts(1,:).^2;pts]-1)/norm(a);
    % score is inliers number
    % idx = find(dist<dist_p);
    % subset = pts(:,idx);
    score = sum(dist<dist_p);
    % score = norm(dist(idx))-mean(abs(a'*[0;0]-1)/norm(a));
    % score = sum(subset'*a > 1) - sum(subset'*a < 1);
    % update a_best
    if score > max_score 
        max_score = score;
        a_best = a;
    end
end
