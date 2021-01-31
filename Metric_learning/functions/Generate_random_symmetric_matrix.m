function [A] = Generate_random_symmetric_matrix(dimension)

    % https://www.mathworks.com/matlabcentral/answers/123643-how-to-create-a-symmetric-random-matrix

   d = rand(dimension,1); % The diagonal values
   t = triu(bsxfun(@min,d,d.').*rand(dimension),1); % The upper trianglar random values
   A = diag(d)+t+t.'; % Put them together in a symmetric matrix

end