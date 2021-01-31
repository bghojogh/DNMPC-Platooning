function [Q, Theta, Omega, R, G, F] = Other_steps_of_ADMM_v1(Q, Theta, Omega, R, G, G_set, G_set_cutin, F, O, N, p, car_index, X0, Xdes, Xnfa, Xnffa, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho)

    Q = Optimize_Q_v1(Q, p, car_index, X0, Xdes, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho, Theta, Omega);
    Theta = Optimize_Theta(Theta, Q, Omega, n_iterations, learning_rate, rho);
    Omega = Optimize_Omega(Omega, Theta, Q);
    R = Optimize_R_v1(R, X0, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate);
    G = Optimize_G_v1(G, X0, Xnfa, Xnffa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, N, car_index);
    F = Optimize_F_v1(F, car_index, X0, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, G_set, G_set_cutin, O);
   
end
