function G = Optimize_G_v1(G, X0, Xnfa, Xnffa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, N, car_index)

    if isempty(N{car_index})
        G = zeros(size(G));
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
        bb = (X0(1:2)-Xnfa(1,:)) + (X0(1:2)-Xnffa(1,:));
        aa = bb';
        gradient = 2 * aa * aa';
        for i = 1:Np-1      
            bb = (Xp(i,:)-Xnfa(i+1,:)) + (X0(1:2)-Xnffa(1,:));
            aa = bb';
            gradient = gradient + (2 * aa * aa');     
        end

        % projected gradient descent:
        for iteration = 1:n_iterations
            G = G - (learning_rate * gradient);
            G = ProjectOntoPositiveSemideinite_epsilon(G, 0.01);
            % G = ProjectOntoPositiveSemideinite(G);
            % G = ProjectOntoOrthogonalMatrix(G);
        end
    end
   
end
