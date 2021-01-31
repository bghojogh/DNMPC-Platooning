function R = Optimize_R_v2(R, X0, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate)

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

    Udes = Radius/Eta*(Ca*Vp.^2 + Mass*g*f);
    U0 = Radius/Eta*(Ca*X0(2).^2 + Mass*g*f);

    % calculate gradient:
    aa = (u(1)-U0)';
    gradient = 2 * aa * aa';
    for i = 1:Np-1      
        aa = (u(i+1)-Udes(i))';
        gradient = gradient + (2 * aa * aa');     
    end

    % projected gradient descent:
    for iteration = 1:n_iterations
        R = R - (learning_rate * gradient);
        R = ProjectOntoPositiveSemideinite_epsilon(R, 0.01);
        % R = ProjectOntoPositiveSemideinite(R);
        % R = ProjectOntoOrthogonalMatrix(R);
    end
   
end
