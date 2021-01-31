%% Code for the paper
% Title:   Distributed model predictive control for heterogeneous vehicle platoons under unidirectional topologies
% Authors: Zheng, Yang, Shengbo Eben Li, Keqiang Li, Francesco Borrelli, and J. Karl Hedrick. 
% Journal: IEEE Transactions on Control Systems Technology 25, no. 3 (2017): 899-910.

addpath('./../functions/')

%% DMPC for platoons with PF topology
clc;clear;close all;
load PlatoonParameter.mat  % This set of parameters were used in the paper

Time_sim = 12;
Num_step = Time_sim / Tim_step;

%% Initial Virables 
Postion  = zeros(Num_step,Num_veh);     % postion of each vehicle;
Velocity = zeros(Num_step,Num_veh);     % velocity of each vehicle;
Torque   = zeros(Num_step,Num_veh);     % Braking or Tracking Torque of each vehicle;
U        = zeros(Num_step,Num_veh);     % Desired Braking or Tracking Torque of each vehicle;

Cost    = zeros(Num_step,Num_veh);      % Cost function
Exitflg = zeros(Num_step,Num_veh);      % Stop flag - solvers

% Leading vehicle
d  = 10;                                % Desired spacing
a0 = zeros(Num_step,1); 
v0 = zeros(Num_step,1); 
x0 = zeros(Num_step,1);


% Transient process of leader, which is given in advance
v0(1) = 20; a0(1/Tim_step+1:2/Tim_step) = 2; 
for i = 2:Num_step
    v0(i) = v0(i-1)+a0(i)*Tim_step; 
    x0(i) = x0(i-1)+v0(i)*Tim_step;    
end
 
% Zero initial error for the followers
for i = 1:Num_veh
    Postion(1,i) = x0(1)-i*d;
    Velocity(1,i) = 20;             
    Torque(1,i) = (Mass(i)*g*f + Ca(i)*Velocity(1,i)^2)*R(i)/Eta;
end



%% parameters:
n_iterations = 10;
n_iterations_ADMM = 10;
learning_rate = 0.1;
rho = 0.1;
p = [1, 1, 1, 1, 1, 1, 1];
O = {[2], [3], [4], [5], [6], [7], []}; % can send information to
N = {[], [1], [2], [3], [4], [5], [6]}; % can get information from

%% MPC weighted matrix in the cost function
% PFL topology --> Fi > Gi+1
% Q1 : leader weighted matrix for state; 
% R1 --> leader weighted matrix for control input
% Fi --> penalty for deviation of its own assumed trajectory 
% Gi --> penalty for deviation of its neighbors' assumed trajectory
time_elapsed = zeros(Num_step,Num_veh);
Q_list = zeros(Num_step,Num_veh,2,2);
Theta_list = zeros(Num_step,Num_veh,2,2);
Omega_list = zeros(Num_step,Num_veh,2,2);
R_list = zeros(Num_step,Num_veh);
G_list = zeros(Num_step,Num_veh,2,2);
F_list = zeros(Num_step,Num_veh,2,2);
Q_list_initial = zeros(1,Num_veh,2,2);
Theta_list_initial = zeros(1,Num_veh,2,2);
Omega_list_initial = zeros(1,Num_veh,2,2);
R_list_initial = zeros(1,Num_veh);
G_list_initial = zeros(1,Num_veh,2,2);
F_list_initial = zeros(1,Num_veh,2,2);
Q1 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q2 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q3 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q4 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q5 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q6 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Q7 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
if (p(1) == 0); Q1 = Q1 * 0; end
if (p(2) == 0); Q2 = Q2 * 0; end
if (p(3) == 0); Q3 = Q3 * 0; end
if (p(4) == 0); Q4 = Q4 * 0; end
if (p(5) == 0); Q5 = Q5 * 0; end
if (p(6) == 0); Q6 = Q6 * 0; end
if (p(7) == 0); Q7 = Q7 * 0; end
Theta1 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta2 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta3 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta4 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta5 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta6 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Theta7 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
Omega1 = Generate_random_symmetric_matrix(2);
Omega2 = Generate_random_symmetric_matrix(2);
Omega3 = Generate_random_symmetric_matrix(2);
Omega4 = Generate_random_symmetric_matrix(2);
Omega5 = Generate_random_symmetric_matrix(2);
Omega6 = Generate_random_symmetric_matrix(2);
Omega7 = Generate_random_symmetric_matrix(2);
R1 = rand; R2 = rand; R3 = rand; R4 = rand; R5 = rand; R6 = rand; R7 = rand;
G1 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G2 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G3 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G4 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G5 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G6 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G7 = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
G_set = {G1, G2, G3, G4, G5, G6, G7};
F1 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 1, G_set);
F2 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 2, G_set);
F3 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 3, G_set);
F4 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 4, G_set);
F5 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 5, G_set);
F6 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 6, G_set);
F7 = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O, 7, G_set);   
Q_list_initial(:,1,:,:) = Q1;
Q_list_initial(:,2,:,:) = Q2;
Q_list_initial(:,3,:,:) = Q3;
Q_list_initial(:,4,:,:) = Q4;
Q_list_initial(:,5,:,:) = Q5;
Q_list_initial(:,6,:,:) = Q6;
Q_list_initial(:,7,:,:) = Q7;
Theta_list_initial(:,1,:,:) = Theta1;
Theta_list_initial(:,2,:,:) = Theta2;
Theta_list_initial(:,3,:,:) = Theta3;
Theta_list_initial(:,4,:,:) = Theta4;
Theta_list_initial(:,5,:,:) = Theta5;
Theta_list_initial(:,6,:,:) = Theta6;
Theta_list_initial(:,7,:,:) = Theta7;
Omega_list_initial(:,1,:,:) = Omega1;
Omega_list_initial(:,2,:,:) = Omega2;
Omega_list_initial(:,3,:,:) = Omega3;
Omega_list_initial(:,4,:,:) = Omega4;
Omega_list_initial(:,5,:,:) = Omega5;
Omega_list_initial(:,6,:,:) = Omega6;
Omega_list_initial(:,7,:,:) = Omega7;
R_list_initial(:,1) = R1;
R_list_initial(:,2) = R2;
R_list_initial(:,3) = R3;
R_list_initial(:,4) = R4;
R_list_initial(:,5) = R5;
R_list_initial(:,6) = R6;
R_list_initial(:,7) = R7;
G_list_initial(:,1,:,:) = G1;
G_list_initial(:,2,:,:) = G2;
G_list_initial(:,3,:,:) = G3;
G_list_initial(:,4,:,:) = G4;
G_list_initial(:,5,:,:) = G5;
G_list_initial(:,6,:,:) = G6;
G_list_initial(:,7,:,:) = G7;
F_list_initial(:,1,:,:) = F1;
F_list_initial(:,2,:,:) = F2;
F_list_initial(:,3,:,:) = F3;
F_list_initial(:,4,:,:) = F4;
F_list_initial(:,5,:,:) = F5;
F_list_initial(:,6,:,:) = F6;   

% Distributed MPC assumed state
Np = 20;                      % ????
Pa = zeros(Np,Num_veh);       % Assumed postion of each vehicle;
Va = zeros(Np,Num_veh);       % Assumed velocity of each vehicle;
ua = zeros(Np,Num_veh);       % Assumed  Braking or Tracking Torque input of each vehicle;

Pa_next = zeros(Np+1,Num_veh);  % 1(0):????????Assumed postion of each vehicle at the newt time step;
Va_next = zeros(Np+1,Num_veh);  % Assumed velocity of each vehicle at the newt time step;
ua_next = zeros(Np+1,Num_veh);  % Assumed Braking or Tracking Torque of each vehicle at the newt time step;

% Initialzie the assumed state for the first computation: constant speed
for i = 1:Num_veh
    ua(:,i) = Torque(1,i);
    Pa(1,i) = Postion(1,i);             
    Va(1,i) = Velocity(1,i);
    Ta(1,i) = Torque(1,i);
    for j = 1:Np
        [Pa(j+1,i),Va(j+1,i),Ta(j+1,i)] = VehicleDynamic(ua(j,i),Tim_step,Pa(j,i),Va(j,i),Ta(j,i),Mass(i),R(i),g,f,Eta,Ca(i),Tao(i));
    end    
end

tol_opt = 1e-5;
options = optimset('Display','off','TolFun', tol_opt, 'MaxIter', 2000,...
                'LargeScale', 'off', 'RelLineSrchBnd', [], 'RelLineSrchBndDuration', 1);
 
%% cut in parameters:
Num_veh_cutin = 8;
cutin_OnAndOff = [0, 1, 0, 0, 0, 0, 0]; 

cutin_cars_in_platoon_so_far = zeros(Num_veh_cutin,1);
t_cutin = [3, 2, 5, 3.5, 2, 1.8, 6.5];  %--> in range (0.2, 7]
Mass_cutin = [1210.6, 1305.9, 1863.4, 1554.9, 1231.6, 1830.1, 1698.3, 1605.3];
R_cutin = [0.35, 0.4, 0.33, 0.36, 0.3, 0.38, 0.35, 0.31];
Ca_cutin = [1.16, 1, 1.11, 1.08, 1.14, 1.12, 1.16, 1.15];
Tao_cutin = [0.76, 0.63, 0.53, 0.76, 0.6, 0.71, 0.55, 0.61];
cutin_newIndexInPlatoon = [2, 3, 4, 5, 6, 7, 8];

Postion_cutin  = zeros(Num_step,Num_veh_cutin);     % postion of each vehicle;
Velocity_cutin = zeros(Num_step,Num_veh_cutin);     % velocity of each vehicle;
Torque_cutin   = zeros(Num_step,Num_veh_cutin);     % Braking or Tracking Torque of each vehicle;
U_cutin        = zeros(Num_step,Num_veh_cutin);     % Desired Braking or Tracking Torque of each vehicle;

Cost_cutin    = zeros(Num_step,Num_veh_cutin);      % Cost function
Exitflg_cutin = zeros(Num_step,Num_veh_cutin);      % Stop flag - solvers

% Distributed MPC assumed state
Pa_cutin = zeros(Np,Num_veh_cutin);       % Assumed postion of each vehicle;
Va_cutin = zeros(Np,Num_veh_cutin);       % Assumed velocity of each vehicle;
ua_cutin = zeros(Np,Num_veh_cutin);       % Assumed Braking or Tracking Torque input of each vehicle;

Pa_next_cutin = zeros(Np+1,Num_veh_cutin);  % 1(0): Assumed postion of each vehicle at the newt time step;
Va_next_cutin = zeros(Np+1,Num_veh_cutin);  % Assumed velocity of each vehicle at the newt time step;
ua_next_cutin = zeros(Np+1,Num_veh_cutin);  % Assumed Braking or Tracking Torque of each vehicle at the newt time step;
            
%% cut in weights:
p_cutin = [1, 1, 1, 1, 1, 1, 1, 1];
O_cutin = {[], [], [], [], [], [], [], []}; % can send information to
N_cutin = {[], [], [], [], [], [], []}; % can get information from

Q_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
Theta_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
Omega_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
R_weight_list_cutin = zeros(Num_step,Num_veh_cutin);
G_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
F_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
G_set_cutin = {[], [], [], [], [], [], []};  

%% metric params lists:
Xp_list = zeros(Num_step,Num_veh,Np,2);
Xdes_list = zeros(Num_step,Num_veh,Np+1,2);
u_list = zeros(Num_step,Num_veh,Np);
Udes_list = zeros(Num_step,Num_veh,Np);
Xa_list = zeros(Num_step,Num_veh,Np+1,2);
Xnba_list = zeros(Num_step,Num_veh,Np+1,2);

Xp_list_cutin = zeros(Num_step,Num_veh_cutin,Np,2);
Xdes_list_cutin = zeros(Num_step,Num_veh_cutin,Np+1,2);
u_list_cutin = zeros(Num_step,Num_veh_cutin,Np);
Udes_list_cutin = zeros(Num_step,Num_veh_cutin,Np);
Xa_list_cutin = zeros(Num_step,Num_veh_cutin,Np+1,2);
Xnba_list_cutin = zeros(Num_step,Num_veh_cutin,Np+1,2);

%% cut-out parameters:
cutout_OnAndOff = [0, 0, 0, 1, 0, 0, 0]; 
t_cutout = [0, 0, 0, 4, 0, 0, 0];  %--> in range (0.2, 7]
cutout_cars_in_platoon_so_far = zeros(Num_veh,1);
            
 %%  For debugging
 % Terminal state
 Xend = zeros(Num_step,Num_veh); Vend = zeros(Num_step,Num_veh);
 
%% Iterative Simulation  
for i = 2:Num_step - Np
    
    fprintf('\n Steps i= %d\n',i)
    
    % Solve optimization problem
    tic
    %% Vehicle one
    Vehicle_Type = [Mass(1),R(1),g,f,Eta,Ca(1),Tao(1)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,1),Velocity(i-1,1),Torque(i-1,1)];                % the vehicle variable in the last time
    Pd = x0(i-1:i+Np-1) - d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,1),Va(:,1)];                                             % ???????,???????
    Xnba = zeros(Np+1,2);                                               % 1:????????
   
    cut_in_index_behind = 2;
    if cutin_cars_in_platoon_so_far(cut_in_index_behind) == 1  %--> cut in has occured
        O{original_car_index} = cut_in_index_behind - 0.5; % can send information to
    end
    
    u0 = ua(:,1);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(1,1)*ones(Np,1); ub = Torquebound(1,2)*ones(Np,1);      % ??????              
    Pnp = Pd(end,1); Vnp = Vd(end,1);   % ????
    Xend(i,1) = Pnp; Vend(i,1) = Vnp; Tnp = (Ca(1)*Vnp.^2 + Mass(1)*g*f)/Eta*R(1);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,1), Exitflg(i,1), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q1,Xdes,R1,F1,Xa,G1,Xnba), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q1, Theta1, Omega1, R1, G1, F1] = Other_steps_of_ADMM_v2(Q1, Theta1, Omega1, R1, G1, G_set, G_set_cutin, F1, O, N, p, 1, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{1} = G1;
    end
    Q_list(i,1,:,:) = Q1;
    Theta_list(i,1,:,:) = Theta1;
    Omega_list(i,1,:,:) = Omega1;
    R_list(i,1) = R1;
    G_list(i,1,:,:) = G1;
    F_list(i,1,:,:) = F1;
    
    % ???????
    U(i,1) = u(1);
    [Postion(i,1),Velocity(i,1),Torque(i,1)] = VehicleDynamic(U(i,1),Tim_step,Postion(i-1,1),Velocity(i-1,1),Torque(i-1,1),Mass(1),R(1),g,f,Eta,Ca(1),Tao(1));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,1),Velocity(i,1),Torque(i,1)];   
    ua(1:Np-1,1) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,1),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(1),R(1),g,f,Eta,Ca(1),Tao(1));
    end    
    ua(Np,1) = (Ca(1)*Temp(Np,2).^2 + Mass(1)*g*f)/Eta*R(1);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,1),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(1),R(1),g,f,Eta,Ca(1),Tao(1));
    Pa_next(:,1) = Temp(:,1);
    Va_next(:,1) = Temp(:,2);
    toc
    
    veh_index = 1;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnba);
    
    %% cut-in 2:
    cut_in_index = 2;
    original_index_ahead = cut_in_index - 1;
    original_index_behind = cut_in_index;
    
    if (cutin_OnAndOff(cut_in_index) == 1) && (i >= (t_cutin(cut_in_index) * (1 / Tim_step)))
        if cutin_cars_in_platoon_so_far(cut_in_index) == 0  %--> if has just cut in (had not cut in so far)
            % Zero initial error for the cut-in followers
            Postion_cutin(i,cut_in_index) = Postion(i, original_index_ahead) - d/2;
            Postion_cutin(i-1,cut_in_index) = Postion_cutin(i,cut_in_index);
            Velocity_cutin(i,cut_in_index) = 20;             
            Velocity_cutin(i-1,cut_in_index) = Velocity_cutin(i,cut_in_index); 
            Torque_cutin(i,cut_in_index)   = (Mass_cutin(cut_in_index)*g*f + Ca_cutin(cut_in_index)*Velocity_cutin(1,cut_in_index)^2)*R_cutin(cut_in_index)/Eta;
            Torque_cutin(i-1,cut_in_index) = Torque_cutin(i,cut_in_index);
            % Initialzie the assumed state for the first computation: constant speed
            ua_cutin(:,cut_in_index) = Torque_cutin(i,cut_in_index);
            Pa_cutin(1,cut_in_index) = Postion_cutin(i,cut_in_index);                % The first point should be interpreted as k = 0 (current state)
            Va_cutin(1,cut_in_index) = Velocity_cutin(i,cut_in_index);
            Ta_cutin(1,cut_in_index) = Torque_cutin(i,cut_in_index);
            for j = 1:Np
                [Pa_cutin(j+1,cut_in_index),Va_cutin(j+1,cut_in_index),Ta_cutin(j+1,cut_in_index)] = VehicleDynamic(ua_cutin(j,cut_in_index),Tim_step,Pa_cutin(j,cut_in_index), ...
                          Va_cutin(j,cut_in_index),Ta_cutin(j,cut_in_index),Mass_cutin(cut_in_index),R_cutin(cut_in_index),g,f,Eta,Ca_cutin(cut_in_index),Tao_cutin(cut_in_index));
            end  
            % update O_cutin and N_cutin:
            O_cutin{cut_in_index} = original_index_behind; % can send information to
            N_cutin{cut_in_index} = original_index_ahead; % can get information from
            % Initialize wieghts:
            Q_list_cutin(i,cut_in_index,:,:) = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
            if (p_cutin(cut_in_index) == 0); Q_list_cutin(i,cut_in_index,:,:) = Q_list_cutin(i,cut_in_index,:,:) * 0; end
            Theta_list_cutin(i,cut_in_index,:,:) = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
            Omega_list_cutin(i,cut_in_index,:,:) = Generate_random_symmetric_matrix(2);
            R_weight_list_cutin(i,cut_in_index) = rand; 
            G_list_cutin(i,cut_in_index,:,:) = ProjectOntoPositiveSemideinite(Generate_random_symmetric_matrix(2));
            G_set_cutin{cut_in_index} = G_list_cutin(i,cut_in_index,:,:);
            F_list_cutin(i,cut_in_index,:,:) = Project_F_onto_feasible_set(Generate_random_symmetric_matrix(2), O_cutin, 1, G_set_cutin);
        end
        cutin_cars_in_platoon_so_far(cut_in_index) = 1;
        Vehicle_Type = [Mass_cutin(cut_in_index),R_cutin(cut_in_index),g,f,Eta,Ca_cutin(cut_in_index),Tao_cutin(cut_in_index)];                 % the vehicle parameters ? Mass,R,g,f,Eta,Ca(i),Tao, 
        X0 = [Postion_cutin(i-1, cut_in_index),Velocity_cutin(i-1, cut_in_index),Torque_cutin(i-1, cut_in_index)];                % the vehicle variable in the last time
        if p_cutin(cut_in_index) == 1
            Pd = x0(i-1:i+Np-1) - (2*d);  Vd = v0(i-1:i+Np-1);
        else
            Pd = zeros(Np+1,1);  Vd = zeros(Np+1,1);                      
        end     
        Xdes = [Pd,Vd];  
        Xa = [Pa_cutin(:,cut_in_index),Va_cutin(:,cut_in_index)];                                         
        Xnfa = [Pa(:,original_index_ahead) - d, Va(:,original_index_ahead)];                              

        u0 = ua_cutin(:,cut_in_index);   
        A = [];b = []; Aeq = []; beq = [];                                   
        lb = Torquebound(2,1)*ones(Np,1); ub = Torquebound(2,2)*ones(Np,1);         
        Pnp = Xnfa(end,1); Vnp = Xnfa(end,2);  
        Xend_cutin(i,cut_in_index) = Pnp; Vend_cutin(i,cut_in_index) = Vnp; Tnp = (Ca_cutin(cut_in_index)*Vnp.^2 + Mass_cutin(cut_in_index)*g*f)/Eta*R_cutin(cut_in_index);
        % MPC - subporblem for vehicle 2
        Q_cutin = reshape(Q_list_cutin(i,cut_in_index,:,:), 2, 2);
        Theta_cutin = reshape(Theta_list_cutin(i,cut_in_index,:,:), 2, 2);
        Omega_cutin = reshape(Omega_list_cutin(i,cut_in_index,:,:), 2, 2);
        R_weight_cutin = R_weight_list_cutin(i,cut_in_index);
        G_cutin = reshape(G_list_cutin(i,cut_in_index,:,:), 2, 2);
        F_cutin = reshape(F_list_cutin(i,cut_in_index,:,:), 2, 2);
        for iteration_ADMM = 1:n_iterations_ADMM
            [u, Cost_cutin(i,2), Exitflg_cutin(i,2), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q_cutin,Xdes,R_weight_cutin,F_cutin,Xa,G_cutin,Xnfa), ...
                u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
            [Q_cutin, Theta_cutin, Omega_cutin, R_weight_cutin, G_cutin, F_cutin] = Other_steps_of_ADMM_v2_cutin(Q_cutin, Theta_cutin, Omega_cutin, R_weight_cutin, G_cutin, G_set, G_set_cutin, F_cutin, O_cutin, N_cutin, p_cutin, cut_in_index, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
            G_set_cutin{cut_in_index} = G_cutin;
        end
        Q_list_cutin(i,cut_in_index,:,:) = Q_cutin;
        Theta_list_cutin(i,cut_in_index,:,:) = Theta_cutin;
        Omega_list_cutin(i,cut_in_index,:,:) = Omega_cutin;
        R_weight_list_cutin(i,cut_in_index) = R_weight_cutin;
        G_list_cutin(i,cut_in_index,:,:) = G_cutin;
        F_list_cutin(i,cut_in_index,:,:) = F_cutin;
        G_set_cutin{cut_in_index} = G_list_cutin(i,cut_in_index,:,:);

        % state involves one step
        U_cutin(i,cut_in_index) = u(1);
        [Postion_cutin(i, cut_in_index),Velocity_cutin(i, cut_in_index),Torque_cutin(i, cut_in_index)] = VehicleDynamic(U_cutin(i,cut_in_index),Tim_step,Postion_cutin(i-1, cut_in_index),Velocity_cutin(i-1, cut_in_index),Torque_cutin(i-1, cut_in_index),Mass_cutin(cut_in_index),R_cutin(cut_in_index),g,f,Eta,Ca_cutin(cut_in_index),Tao_cutin(cut_in_index));

        % Update assumed state
        Temp = zeros(Np+1,3);
        Temp(1,:) = [Postion_cutin(i, cut_in_index),Velocity_cutin(i, cut_in_index),Torque_cutin(i, cut_in_index)]; 
        ua_cutin(1:Np-1,2) = u(2:Np);
        for j = 1:Np-1
            [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua_cutin(j,cut_in_index),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass_cutin(cut_in_index),R_cutin(cut_in_index),g,f,Eta,Ca_cutin(cut_in_index),Tao_cutin(cut_in_index));
        end    

        ua_cutin(Np,cut_in_index) = (Ca_cutin(cut_in_index)*Temp(Np,2).^2 + Mass_cutin(cut_in_index)*g*f)/Eta*R_cutin(cut_in_index);
        [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua_cutin(Np,cut_in_index),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass_cutin(cut_in_index),R_cutin(cut_in_index),g,f,Eta,Ca_cutin(cut_in_index),Tao_cutin(cut_in_index));
        Pa_next_cutin(:,cut_in_index) = Temp(:,1);
        Va_next_cutin(:,cut_in_index) = Temp(:,2);
        
        [Xp_list_cutin(i, cut_in_index, :, :), Xdes_list_cutin(i, cut_in_index, :, :), u_list_cutin(i, cut_in_index, :), Udes_list_cutin(i, cut_in_index, :), Xa_list_cutin(i, cut_in_index, :, :), Xnba_list_cutin(i, cut_in_index, :, :)] = ...
        Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    end
    
    %% Vehicle two
    tic
    Vehicle_Type = [Mass(2),R(2),g,f,Eta,Ca(2),Tao(2)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,2),Velocity(i-1,2),Torque(i-1,2)];                % the vehicle variable in the last time
                          % ?Np+1??,?????,i-1 ?????????, i?????????????
    cut_in_index_ahead = 2;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        Pd = x0(i-1:i+Np-1) - 3*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
    else  
        Pd = x0(i-1:i+Np-1) - 2*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
    end
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,2),Va(:,2)];                                             % ???????,???????
                                                   % 1:????????
    
    cut_in_index_ahead = 2;
    original_car_index = cut_in_index_ahead;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        Xnfa = [Pa_cutin(:,cut_in_index_ahead) - d, Va_cutin(:,cut_in_index_ahead)]; 
        p(original_car_index) = 1;
        N{original_car_index} = cut_in_index_ahead - 0.5; % can get information from
    else  
        Xnfa = [Pa(:,1) - d, Va(:,1)];
    end
%     cut_in_index_behind = cut_in_index_ahead + 1;
%     if cutin_cars_in_platoon_so_far(cut_in_index_behind) == 1  %--> cut in has occured
%         O{original_car_index} = cut_in_index_behind - 0.5; % can send information to
%     end
   
    u0 = ua(:,2);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(2,1)*ones(Np,1); ub = Torquebound(2,2)*ones(Np,1);      % ??????              
    Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
    Xend(i,2) = Pnp; Vend(i,2) = Vnp; Tnp = (Ca(2)*Vnp.^2 + Mass(2)*g*f)/Eta*R(2);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,2), Exitflg(i,2), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q2,Xdes,R2,F2,Xa,G2,Xnfa), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q2, Theta2, Omega2, R2, G2, F2] = Other_steps_of_ADMM_v2(Q2, Theta2, Omega2, R2, G2, G_set, G_set_cutin, F2, O, N, p, 2, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{2} = G2;
    end
    Q_list(i,2,:,:) = Q2;
    Theta_list(i,2,:,:) = Theta2;
    Omega_list(i,2,:,:) = Omega2;
    R_list(i,2) = R2;
    G_list(i,2,:,:) = G2;
    F_list(i,2,:,:) = F2;
    
    % ???????
    U(i,2) = u(1);
    [Postion(i,2),Velocity(i,2),Torque(i,2)] = VehicleDynamic(U(i,2),Tim_step,Postion(i-1,2),Velocity(i-1,2),Torque(i-1,2),Mass(2),R(2),g,f,Eta,Ca(2),Tao(2));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,2),Velocity(i,2),Torque(i,2)]; 
    ua(1:Np-1,2) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,2),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(2),R(2),g,f,Eta,Ca(2),Tao(2));
    end    
    
    ua(Np,2) = (Ca(2)*Temp(Np,2).^2 + Mass(2)*g*f)/Eta*R(2);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,2),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(2),R(2),g,f,Eta,Ca(2),Tao(2));
    Pa_next(:,2) = Temp(:,1);
    Va_next(:,2) = Temp(:,2);
    toc
    
    veh_index = 2;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    
    %% vehicle three
    tic
    Vehicle_Type = [Mass(3),R(3),g,f,Eta,Ca(3),Tao(3)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,3),Velocity(i-1,3),Torque(i-1,3)];                % the vehicle variable in the last time
                          % ?Np+1??,?????,i-1 ?????????, i?????????????
    cut_in_index_ahead = 2;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        Pd = x0(i-1:i+Np-1) - 4*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
    else  
        Pd = x0(i-1:i+Np-1) - 3*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
    end
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,3),Va(:,3)];                                             % ???????,???????
    Xnfa = [Pa(:,2) - d, Va(:,2)];                                               % 1:????????
   
    original_car_index = 3;
    veh_index_behind = original_car_index + 1;
    if cutout_cars_in_platoon_so_far(veh_index_behind) == 1  %--> cut in has occured
        O{original_car_index} = veh_index_behind + 1; % can send information to
    end
    
    u0 = ua(:,3);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(3,1)*ones(Np,1); ub = Torquebound(3,2)*ones(Np,1);      % ??????              
    Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
    Xend(i,3) = Pnp; Vend(i,3) = Vnp; Tnp = (Ca(3)*Vnp.^2 + Mass(3)*g*f)/Eta*R(3);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,3), Exitflg(i,3), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q3,Xdes,R3,F3,Xa,G3,Xnfa), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q3, Theta3, Omega3, R3, G3, F3] = Other_steps_of_ADMM_v2(Q3, Theta3, Omega3, R3, G3, G_set, G_set_cutin, F3, O, N, p, 3, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{3} = G3;
    end
    Q_list(i,3,:,:) = Q3;
    Theta_list(i,3,:,:) = Theta3;
    Omega_list(i,3,:,:) = Omega3;
    R_list(i,3) = R3;
    G_list(i,3,:,:) = G3;
    F_list(i,3,:,:) = F3;
    
    % ???????
    U(i,3) = u(1);
    [Postion(i,3),Velocity(i,3),Torque(i,3)] = VehicleDynamic(U(i,3),Tim_step,Postion(i-1,3),Velocity(i-1,3),Torque(i-1,3),Mass(3),R(3),g,f,Eta,Ca(3),Tao(3));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,3),Velocity(i,3),Torque(i,3)];    
    ua(1:Np-1,3) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,3),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(3),R(3),g,f,Eta,Ca(3),Tao(3));
    end   
    
    ua(Np,3) = (Ca(3)*Temp(Np,2).^2 + Mass(3)*g*f)/Eta*R(3);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,3),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(3),R(3),g,f,Eta,Ca(3),Tao(3));
    Pa_next(:,3) = Temp(:,1);
    Va_next(:,3) = Temp(:,2);
    toc
    
    veh_index = 3;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    
    %% vehicle four
    veh_index = 4;
    if ~((cutout_OnAndOff(veh_index) == 1) && (i >= (t_cutout(veh_index) * (1 / Tim_step)))) %%--> if cut-out has not occured
        tic
        Vehicle_Type = [Mass(4),R(4),g,f,Eta,Ca(4),Tao(4)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
        X0 = [Postion(i-1,4),Velocity(i-1,4),Torque(i-1,4)];                % the vehicle variable in the last time
                               % ?Np+1??,?????,i-1 ?????????, i?????????????
        cut_in_index_ahead = 2;
        if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
            Pd = x0(i-1:i+Np-1) - 5*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        else  
            Pd = x0(i-1:i+Np-1) - 4*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
        Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
        Xa = [Pa(:,4),Va(:,4)];                                             % ???????,???????
        Xnfa = [Pa(:,3) - d, Va(:,3)];                                               % 1:????????

        u0 = ua(:,4);   % ?????    
        A = [];b = []; Aeq = []; beq = [];                                       % ??????
        lb = Torquebound(4,1)*ones(Np,1); ub = Torquebound(4,2)*ones(Np,1);      % ??????              
        Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
        Xend(i,4) = Pnp; Vend(i,4) = Vnp; Tnp = (Ca(4)*Vnp.^2 + Mass(4)*g*f)/Eta*R(4);
        % MPC ????
        for iteration_ADMM = 1:n_iterations_ADMM
        [u, Cost(i,4), Exitflg(i,4), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q3,Xdes,R3,F3,Xa,G3,Xnfa), ...
            u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
        [Q4, Theta4, Omega4, R4, G4, F4] = Other_steps_of_ADMM_v2(Q4, Theta4, Omega4, R4, G4, G_set, G_set_cutin, F4, O, N, p, 4, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
        G_set{4} = G4;
        end
        Q_list(i,4,:,:) = Q4;
        Theta_list(i,4,:,:) = Theta4;
        Omega_list(i,4,:,:) = Omega4;
        R_list(i,4) = R4;
        G_list(i,4,:,:) = G4;
        F_list(i,4,:,:) = F4;

        % ???????
        U(i,4) = u(1);
        [Postion(i,4),Velocity(i,4),Torque(i,4)] = VehicleDynamic(U(i,4),Tim_step,Postion(i-1,4),Velocity(i-1,4),Torque(i-1,4),Mass(4),R(4),g,f,Eta,Ca(4),Tao(4));

        % ????????,?????assumed state, ?t+1????Np?????
        Temp = zeros(Np+1,3);
        Temp(1,:) = [Postion(i,4),Velocity(i,4),Torque(i,4)];  
        ua(1:Np-1,4) = u(2:Np);
        for j = 1:Np-1
            [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,4),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(4),R(4),g,f,Eta,Ca(4),Tao(4));
        end

        ua(Np,4) = (Ca(4)*Temp(Np,2).^2 + Mass(4)*g*f)/Eta*R(4);
        [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,4),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(4),R(4),g,f,Eta,Ca(4),Tao(4));
        Pa_next(:,4) = Temp(:,1);
        Va_next(:,4) = Temp(:,2);
        toc
        
        veh_index = 4;
        [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
        Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    else  %%--> cut-out has occured
        cutout_cars_in_platoon_so_far(veh_index) = 1;
    end
    
    
      %% vehicle five
    tic
    Vehicle_Type = [Mass(5),R(5),g,f,Eta,Ca(5),Tao(5)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,5),Velocity(i-1,5),Torque(i-1,5)];                % the vehicle variable in the last time
                          % ?Np+1??,?????,i-1 ?????????, i?????????????
    cut_in_index_ahead = 2;
    cutout_veh_index = 4;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 5*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 6*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    else  
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 4*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 5*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    end
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,5),Va(:,5)];                                             % ???????,???????
                                                   % 1:????????
    if cutout_cars_in_platoon_so_far(veh_index)
        Xnfa = [Pa(:,3) - d, Va(:,3)]; 
    else
        Xnfa = [Pa(:,4) - d, Va(:,4)];      
    end
    
    original_car_index = 5;
    veh_index_ahead = original_car_index - 1;
    if cutout_cars_in_platoon_so_far(veh_index_ahead) == 1  %--> cut in has occured
        N{original_car_index} = veh_index_ahead - 1; % can get information from
    end
   
    u0 = ua(:,5);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(5,1)*ones(Np,1); ub = Torquebound(5,2)*ones(Np,1);      % ??????              
    Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
    Xend(i,5) = Pnp; Vend(i,5) = Vnp; Tnp = (Ca(5)*Vnp.^2 + Mass(5)*g*f)/Eta*R(5);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,5), Exitflg(i,5), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q3,Xdes,R3,F3,Xa,G3,Xnfa), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q5, Theta5, Omega5, R5, G5, F5] = Other_steps_of_ADMM_v2(Q5, Theta5, Omega5, R5, G5, G_set, G_set_cutin, F5, O, N, p, 5, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{5} = G5;
    end
    Q_list(i,5,:,:) = Q5;
    Theta_list(i,5,:,:) = Theta5;
    Omega_list(i,5,:,:) = Omega5;
    R_list(i,5) = R5;
    G_list(i,5,:,:) = G5;
    F_list(i,5,:,:) = F5;
    
    % ???????
    U(i,5) = u(1);
    [Postion(i,5),Velocity(i,5),Torque(i,5)] = VehicleDynamic(U(i,5),Tim_step,Postion(i-1,5),Velocity(i-1,5),Torque(i-1,5),Mass(5),R(5),g,f,Eta,Ca(5),Tao(5));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,5),Velocity(i,5),Torque(i,5)];   
    ua(1:Np-1,5) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,5),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(5),R(5),g,f,Eta,Ca(5),Tao(5));
    end
    
    ua(Np,5) = (Ca(5)*Temp(Np,2).^2 + Mass(5)*g*f)/Eta*R(5);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,5),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(5),R(5),g,f,Eta,Ca(5),Tao(5));
    Pa_next(:,5) = Temp(:,1);
    Va_next(:,5) = Temp(:,2);

    toc
    
    veh_index = 5;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    
    %% vehicle six
    tic
    Vehicle_Type = [Mass(6),R(6),g,f,Eta,Ca(6),Tao(6)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,6),Velocity(i-1,6),Torque(i-1,6)];                % the vehicle variable in the last time
                           % ?Np+1??,?????,i-1 ?????????, i?????????????
    cut_in_index_ahead = 2;
    cutout_veh_index = 4;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 6*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 7*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    else  
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 5*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 6*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    end
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,6),Va(:,6)];                                             % ???????,???????
    Xnfa = [Pa(:,5) - d, Va(:,5)];                                               % 1:????????
   
    u0 = ua(:,6);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(6,1)*ones(Np,1); ub = Torquebound(6,2)*ones(Np,1);      % ??????              
    Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
    Xend(i,6) = Pnp; Vend(i,6) = Vnp; Tnp = (Ca(6)*Vnp.^2 + Mass(6)*g*f)/Eta*R(6);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,6), Exitflg(i,6), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q3,Xdes,R3,F3,Xa,G3,Xnfa), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q6, Theta6, Omega6, R6, G6, F6] = Other_steps_of_ADMM_v2(Q6, Theta6, Omega6, R6, G6, G_set, G_set_cutin, F6, O, N, p, 6, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{6} = G6;
    end
    Q_list(i,6,:,:) = Q6;
    Theta_list(i,6,:,:) = Theta6;
    Omega_list(i,6,:,:) = Omega6;
    R_list(i,6) = R6;
    G_list(i,6,:,:) = G6;
    F_list(i,6,:,:) = F6;
    
    % ???????
    U(i,6) = u(1);
    [Postion(i,6),Velocity(i,6),Torque(i,6)] = VehicleDynamic(U(i,6),Tim_step,Postion(i-1,6),Velocity(i-1,6),Torque(i-1,6),Mass(6),R(6),g,f,Eta,Ca(6),Tao(6));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,6),Velocity(i,6),Torque(i,6)]; 
    ua(1:Np-1,6) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,6),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(6),R(6),g,f,Eta,Ca(6),Tao(6));
    end
     
    ua(Np,6) = (Ca(6)*Temp(Np,2).^2 + Mass(6)*g*f)/Eta*R(6);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,6),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(6),R(6),g,f,Eta,Ca(6),Tao(6));
    Pa_next(:,6) = Temp(:,1);
    Va_next(:,6) = Temp(:,2);

    toc
    
    veh_index = 6;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    
     %% vehicle seven
    tic
    Vehicle_Type = [Mass(7),R(7),g,f,Eta,Ca(7),Tao(7)];                 % the vehicle parameters : Mass,R,g,f,Eta,Ca(i),Tao, 
    X0 = [Postion(i-1,7),Velocity(i-1,7),Torque(i-1,7)];                % the vehicle variable in the last time
                          % ?Np+1??,?????,i-1 ?????????, i?????????????
    cut_in_index_ahead = 2;
    cutout_veh_index = 4;
    if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 7*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 8*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    else  
        if cutout_cars_in_platoon_so_far(cutout_veh_index)
            Pd = x0(i-1:i+Np-1) - 6*d;  Vd = v0(i-1:i+Np-1);
        else
            Pd = x0(i-1:i+Np-1) - 7*d;  Vd = v0(i-1:i+Np-1);                      % ?Np+1??,?????,i-1 ?????????, i?????????????
        end
    end
    Xdes = [Pd,Vd];  % Udes = Td;                                       % ?????????
    Xa = [Pa(:,7),Va(:,7)];                                             % ???????,???????
    Xnfa = [Pa(:,6) - d, Va(:,6)];                                               % 1:????????
   
    u0 = ua(:,7);   % ?????    
    A = [];b = []; Aeq = []; beq = [];                                       % ??????
    lb = Torquebound(7,1)*ones(Np,1); ub = Torquebound(7,2)*ones(Np,1);      % ??????              
    Pnp = (Xnfa(end,1)+Pd(end))/2; Vnp = (Xnfa(end,2)+Vd(end))/2;   % ????
    Xend(i,7) = Pnp; Vend(i,7) = Vnp; Tnp = (Ca(7)*Vnp.^2 + Mass(7)*g*f)/Eta*R(7);
    % MPC ????
    for iteration_ADMM = 1:n_iterations_ADMM
    [u, Cost(i,7), Exitflg(i,7), output] = fmincon(@(u) Costfunction2( Np, Tim_step, X0 ,u, Vehicle_Type,Q3,Xdes,R3,F3,Xa,G3,Xnfa), ...
        u0, A, b, Aeq, beq, lb, ub, @(u) Nonlinearconstraints(Np, Tim_step, X0, u, Vehicle_Type,Pnp,Vnp,Tnp),options); 
    [Q7, Theta7, Omega7, R7, G7, F7] = Other_steps_of_ADMM_v2(Q7, Theta7, Omega7, R7, G7, G_set, G_set_cutin, F7, O, N, p, 7, X0, Xdes, Xnba, Xa, Np, Vehicle_Type, u, Tim_step, n_iterations, learning_rate, rho);
    G_set{7} = G7;
    end
    Q_list(i,7,:,:) = Q7;
    Theta_list(i,7,:,:) = Theta7;
    Omega_list(i,7,:,:) = Omega7;
    R_list(i,7) = R7;
    G_list(i,7,:,:) = G7;
    F_list(i,7,:,:) = F7;
    
    % ???????
    U(i,7) = u(1);
    [Postion(i,7),Velocity(i,7),Torque(i,7)] = VehicleDynamic(U(i,7),Tim_step,Postion(i-1,7),Velocity(i-1,7),Torque(i-1,7),Mass(7),R(7),g,f,Eta,Ca(7),Tao(7));
    
    % ????????,?????assumed state, ?t+1????Np?????
    Temp = zeros(Np+1,3);
    Temp(1,:) = [Postion(i,7),Velocity(i,7),Torque(i,7)];
    ua(1:Np-1,7) = u(2:Np);
    for j = 1:Np-1
        [Temp(j+1,1),Temp(j+1,2),Temp(j+1,3)] = VehicleDynamic(ua(j,7),Tim_step,Temp(j,1),Temp(j,2),Temp(j,3),Mass(7),R(7),g,f,Eta,Ca(7),Tao(7));
    end

    ua(Np,7) = (Ca(7)*Temp(Np,2).^2 + Mass(7)*g*f)/Eta*R(7);
    [Temp(Np+1,1),Temp(Np+1,2),Temp(Np+1,3)] = VehicleDynamic(ua(Np,7),Tim_step,Temp(Np,1),Temp(Np,2),Temp(Np,3),Mass(7),R(7),g,f,Eta,Ca(7),Tao(7));
    Pa_next(:,7) = Temp(:,1);
    Va_next(:,7) = Temp(:,2);

    toc
    
    veh_index = 7;
    [Xp_list(i, veh_index, :, :), Xdes_list(i, veh_index, :, :), u_list(i, veh_index, :), Udes_list(i, veh_index, :), Xa_list(i, veh_index, :, :), Xnba_list(i, veh_index, :, :)] = ...
    Costfunction2_returnParams(Np, Tim_step, X0, u, Vehicle_Type, Xdes, Xa, Xnfa);
    
    %% ????????
    Pa = Pa_next;
    Va = Va_next;
    
    Pa_cutin = Pa_next_cutin;
    Va_cutin = Va_next_cutin;
       
end

save('workspace.mat')

FigurePlot