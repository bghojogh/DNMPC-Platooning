function Theta = Optimize_Theta(Theta, Q, Omega, n_iterations, learning_rate, rho)

    % calculate gradient:
    gradient = -1 * rho * (Q - Theta + Omega);

    % projected gradient descent:
    for iteration = 1:n_iterations
        Theta = Theta - (learning_rate * gradient);
        Theta = ProjectOntoPositiveSemideinite_epsilon(Theta, 0.01);
        % Theta = ProjectOntoPositiveSemideinite(Theta);
        % Theta = ProjectOntoOrthogonalMatrix(Theta);
    end
   
end
