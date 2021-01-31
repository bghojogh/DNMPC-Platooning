function [A] = ProjectOntoPositiveSemideinite(A)

    [V, D] = eig(A);
    D(D < 0) = 0;
    A = V * D * transpose(V);

end