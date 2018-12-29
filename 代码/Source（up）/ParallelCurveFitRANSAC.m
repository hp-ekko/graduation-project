function [a1_best, a2_best, max_score] = ParallelCurveFitRANSAC(pts1,pts2,N,dist_p)
%ParallelCurveFitRANSAC Fit Two Curve using ransac
% we define an inlier if its disance is 
% smaller than dist_p of mean distance
M1 = size(pts1,2);
M2 = size(pts2,2);
K = 50;
a1_best = [0,0]; 
a2_best = [0,0];
max_score = 0;
for k = 1:N
    % sample
    idx1 = randsample(M1,K);
    idx2 = randsample(M2,K);
    % in the curve function, we assume a0*x1^2+a1*x1+a2*x2=1
    % in the parallel curve function, we assume 
    %   a0*x11^2+a1*x11+a2*(x12+a3)=1
    %   a0*x21^2+a1*x21+a2*(x21-a3)=1
    % => a0*x11^2+a1*x11+a2*x12+a3=1
    %    a0*x21^2+a1*x21+a2*x22-a3=1
    % use Least Square
    % =>a0*x11^2+a1*x11+a2*x12+a3*1 =1
    % =>a0*x21^2+a1*x21+a2*x12-a3*1 =1
    X1 = [pts1(1,idx1).^2;pts1(1,idx1);pts1(2,idx1); ones(1,K)]; 
    X2 = [pts2(1,idx2).^2;pts2(1,idx2);pts2(2,idx2);-ones(1,K)]; 
    X = [X1,X2];
    a = X'\ones(K*2,1);
    % split a into a1 a2
    % from a0*x21^2+a1*x21+a2*x12 = 1+a3 
    %   => [a0,a1,a2]/(1+a3)*X = 1
    a1 = [a(1);a(2);a(3)]/(1-a(4));
    a2 = [a(1);a(2);a(3)]/(1+a(4));
    dist1 = abs(a1'*[pts1(1,:).^2;pts1]-1)/norm(a1);
    dist2 = abs(a2'*[pts2(1,:).^2;pts2]-1)/norm(a2);
    % Finally, I use the inliers number as score
    score = sum(dist1<dist_p)+sum(dist2<dist_p);
    % update a_best
    if score > max_score 
        max_score = score;
        a1_best = a1;
        a2_best = a2;
    end
end
