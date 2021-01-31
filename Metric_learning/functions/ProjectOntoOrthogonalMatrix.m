function [A] = ProjectOntoOrthogonalMatrix(A)

    [U, S, V] = svd(A);
    S = eye(size(A));
    A = U * S * transpose(V);

end