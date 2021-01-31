function F = Optimize_F_v2(F, car_index, X0, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, G_set, G_set_cutin, O)

    % calculations taken rom "Costfunction2" unction:
    Pp = zeros(Np,1);     % Predictive Position
    Vp = zeros(Np,1);     % Predictive Velocity
    Tp = zeros(Np,1);     % Predictive Torque

    Mass = Vehicle_Type(1);Radius = Vehicle_Type(2); g = Vehicle_Type(3);f = Vehicle_Type(4);
    Eta = Vehicle_Type(5);Ca = Vehicle_Type(6);Tao = Vehicle_Type(7);

    [Pp(1),Vp(1),Tp(1)] = VehicleDynamic(u(1),Tim_step,X0(1),X0(2),X0(3),Mass,Radius,g,f,Eta,Ca,Tao);
    for i = 1:Np-1
        [Pp(i+1),Vp(i+1),Tp(i+1)] = VehicleDynamic(u(i+1),Tim_step,Pp(i),Vp(i),Tp(i),Mass,Radius,g,f,Eta,Ca,Tao);
    end

    Xp = [Pp,Vp];      % Predictive State

    % calculate gradient:
    aa = (X0(1:2)-Xa(1,:))';
    gradient = 2 * aa * aa';
    for i = 1:Np-1      
        aa = (Xp(i,:)-Xa(i+1,:))';
        gradient = gradient + (2 * aa * aa');     
    end

    % projected gradient descent:
    for iteration = 1:n_iterations
        F = F - (learning_rate * gradient);
        F = Project_F_onto_feasible_set(F, O, car_index, G_set, G_set_cutin);
    end
   
end
