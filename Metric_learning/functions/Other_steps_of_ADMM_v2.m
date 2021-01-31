function [Q, Theta, Omega, R, G, F] = Other_steps_of_ADMM_v2(Q, Theta, Omega, R, G, G_set, G_set_cutin, F, O, N, p, car_index, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho)

    Q = Optimize_Q_v2(Q, p, car_index, X0, Xdes, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho, Theta, Omega);
    Theta = Optimize_Theta(Theta, Q, Omega, n_iterations, learning_rate, rho);
    Omega = Optimize_Omega(Omega, Theta, Q);
    R = Optimize_R_v2(R, X0, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate);
    G = Optimize_G_v2(G, X0, Xnba, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, N, car_index);
    F = Optimize_F_v2(F, car_index, X0, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, G_set, G_set_cutin, O);
   
end
