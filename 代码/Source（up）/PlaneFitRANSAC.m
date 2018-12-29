function [a_best,max_score] = PlaneFitRANSAC(pts,N,dist_p)
%PlaneFitRANSAC Fit line using ransac
% we define an inlier if its disance is 
% smaller than dist_p of mean distance
M = size(pts,2);
K = 1000;
%d2_mean = norm(pts - mean(pts,2),'fro')/length(pts);
%d_mean = sqrt(d2_mean);
a_best = [0,0,0]; max_score = 0;
for k = 1:N
    % sample
    idx = randsample(M,K);
    X = pts(:,idx);   
    % compute line
    % a = inv(X*X')*X*ones(N,1);
    a = X'\ones(K,1);
    % a = [a;-1];
    % distance
    dist = abs(a'*pts-1)/norm(a);
    % score is inliers number
    idx = find(dist<dist_p);
    subset = pts(:,idx);
    score = norm(dist(idx));
    % score = sum(subset'*a > 1) - sum(subset'*a < 1);
    % update a_best
    if score > max_score 
        max_score = score;
        a_best = [a;-1];
    end
end
