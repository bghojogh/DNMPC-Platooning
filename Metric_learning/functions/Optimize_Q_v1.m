function Q = Optimize_Q_v1(Q, p, car_index, X0, Xdes, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho, Theta, Omega)

    if p(car_index) == 0
        Q = zeros(size(Q));
    else
        % calculations taken rom "Costfunction1" unction:
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
        aa = (X0(1:2)-Xdes(1,:))';
        gradient = 2 * aa * aa';
        for i = 1:Np-1      
            aa = (Xp(i,:)-Xdes(i+1,:))';
            gradient = gradient + (2 * aa * aa');     
        end
        gradient = gradient + (rho * (Q - Theta + Omega));
        
        % gradient descent:
        for iteration = 1:n_iterations
            Q = Q - (learning_rate * gradient);
        end
        
    end
   
end
