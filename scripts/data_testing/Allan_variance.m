% compute Allan variance
% input:
%     y --- array of PxQ dimension, Allan variance is counted for each column
%     M --- number of elements in the window
% output:
%     var --- Allan variance, size(var) = 1xQ
function [var] = Allan_variance(y,M)
    P = size(y,1);
    Q = size(y,2);
    K = floor(P/M);        % number of clusters
    averages = zeros(K,Q); % array (matrix) of K averages of clusters
    var = zeros(1,Q);      % Allan variance
    for i = 1:K
        z = y((i-1)*M+1:i*M,:); % non-overlaping
        averages(i,:) = sum(z)/M;
    end

    for i = 1 : (K-1)
        var = var + ( averages(i+1,:) - averages(i,:) ).^2;
    end
    var = (var/(2*K-2)).^(1/2);
end

