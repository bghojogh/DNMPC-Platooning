function [A] = ProjectOntoPositiveSemideinite_epsilon(A, epsilon)

    [V, D] = eig(A);
    D(D < epsilon) = epsilon;
    A = V * D * transpose(V);

end