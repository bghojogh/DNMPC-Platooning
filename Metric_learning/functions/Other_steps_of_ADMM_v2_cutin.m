function [Q_cutin, Theta_cutin, Omega_cutin, R_cutin, G_cutin, F_cutin] = Other_steps_of_ADMM_v2_cutin(Q_cutin, Theta_cutin, Omega_cutin, R_cutin, G_cutin, G_set, G_set_cutin, F_cutin, O_cutin, N_cutin, p_cutin, cut_in_index, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho)

    Q_cutin = Optimize_Q_v2(Q_cutin, p_cutin, cut_in_index, X0, Xdes, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho, Theta_cutin, Omega_cutin);
    Theta_cutin = Optimize_Theta(Theta_cutin, Q_cutin, Omega_cutin, n_iterations, learning_rate, rho);
    Omega_cutin = Optimize_Omega(Omega_cutin, Theta_cutin, Q_cutin);
    R_cutin = Optimize_R_v2(R_cutin, X0, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate);
    G_cutin = Optimize_G_v2(G_cutin, X0, Xnba, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, N_cutin, cut_in_index);
    F_cutin = Optimize_F_v2_cutin(F_cutin, cut_in_index, X0, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, G_set, G_set_cutin, O_cutin);
   
end
