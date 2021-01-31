
close all
clc

show_figure = 'off';  %--> on, off

%%
Q_list = zeros(Num_step,Num_veh,2,2);
R_list = zeros(Num_step,Num_veh);
G_list = zeros(Num_step,Num_veh,2,2);
F_list = zeros(Num_step,Num_veh,2,2);

Q_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
R_weight_list_cutin = zeros(Num_step,Num_veh_cutin);
G_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);
F_list_cutin = zeros(Num_step,Num_veh_cutin,2,2);

for i = 1:Num_step
    Q_list(i,1,:,:) = Q1;
    Q_list(i,2,:,:) = Q2;
    Q_list(i,3,:,:) = Q3;
    Q_list(i,4,:,:) = Q4;
    Q_list(i,5,:,:) = Q5;
    Q_list(i,6,:,:) = Q6;
    Q_list(i,7,:,:) = Q7;
    
    R_list(i,1,:,:) = R1;
    R_list(i,2,:,:) = R2;
    R_list(i,3,:,:) = R3;
    R_list(i,4,:,:) = R4;
    R_list(i,5,:,:) = R5;
    R_list(i,6,:,:) = R6;
    R_list(i,7,:,:) = R7;
    
    G_list(i,1,:,:) = G1;
    G_list(i,2,:,:) = G2;
    G_list(i,3,:,:) = G3;
    G_list(i,4,:,:) = G4;
    G_list(i,5,:,:) = G5;
    G_list(i,6,:,:) = G6;
    G_list(i,7,:,:) = G7;
    
    F_list(i,1,:,:) = F1;
    F_list(i,2,:,:) = F2;
    F_list(i,3,:,:) = F3;
    F_list(i,4,:,:) = F4;
    F_list(i,5,:,:) = F5;
    F_list(i,6,:,:) = F6;
    F_list(i,7,:,:) = F7;
    
    Q_list_cutin(i,1,:,:) = Q1;
    Q_list_cutin(i,2,:,:) = Q2;
    Q_list_cutin(i,3,:,:) = Q3;
    Q_list_cutin(i,4,:,:) = Q4;
    Q_list_cutin(i,5,:,:) = Q5;
    Q_list_cutin(i,6,:,:) = Q6;
    Q_list_cutin(i,7,:,:) = Q7;
    
    R_weight_list_cutin(i,1,:,:) = R1;
    R_weight_list_cutin(i,2,:,:) = R2;
    R_weight_list_cutin(i,3,:,:) = R3;
    R_weight_list_cutin(i,4,:,:) = R4;
    R_weight_list_cutin(i,5,:,:) = R5;
    R_weight_list_cutin(i,6,:,:) = R6;
    R_weight_list_cutin(i,7,:,:) = R7;
    
    G_list_cutin(i,1,:,:) = G1;
    G_list_cutin(i,2,:,:) = G2;
    G_list_cutin(i,3,:,:) = G3;
    G_list_cutin(i,4,:,:) = G4;
    G_list_cutin(i,5,:,:) = G5;
    G_list_cutin(i,6,:,:) = G6;
    G_list_cutin(i,7,:,:) = G7;
    
    F_list_cutin(i,1,:,:) = F1;
    F_list_cutin(i,2,:,:) = F2;
    F_list_cutin(i,3,:,:) = F3;
    F_list_cutin(i,4,:,:) = F4;
    F_list_cutin(i,5,:,:) = F5;
    F_list_cutin(i,6,:,:) = F6;
    F_list_cutin(i,7,:,:) = F7;
end



%%
position_or_velocity = 2;  %--> 1: position, 2: velocity
times_to_plot = [1, t_cutin(2), t_cutout(4), 7];
do_project = 1;  %--> 0 or 1
if position_or_velocity == 1
    path_save = sprintf('./results/position/do_project=%d/', do_project);
else
    path_save = sprintf('./results/velocity/do_project=%d/', do_project);
end

time_ = times_to_plot(1);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  


time_ = times_to_plot(2);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(3);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))   

time_ = times_to_plot(4);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list(t,original_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xa = Xa_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xa = project_onto_subspace(F_list_cutin(t,cutin_car_index,:,:), reshape(Xa, 2, 1));
            end
            position_assumed = Xa(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  


%% Q:
time_ = times_to_plot(1);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  


time_ = times_to_plot(2);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(3);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(4);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list(t,original_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xdes = Xdes_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xdes = project_onto_subspace(Q_list_cutin(t,cutin_car_index,:,:), reshape(Xdes, 2, 1));
            end
            position_assumed = Xdes(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

%% G:
time_ = times_to_plot(1);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(2);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(3);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(4);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list(t, original_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list(t,original_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        for horizon_time_index = 1:Np
            Xp = Xp_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Xp = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xp, 2, 1));
            end
            position_predicted = Xp(position_or_velocity);
            Xnba = Xnba_list_cutin(t, cutin_car_index, horizon_time_index+1, :);
            if do_project
                Xnba = project_onto_subspace(G_list_cutin(t,cutin_car_index,:,:), reshape(Xnba, 2, 1));
            end
            position_assumed = Xnba(position_or_velocity);
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

%% R:
time_ = times_to_plot(1);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list(t, original_car_index, horizon_time_index);
            if do_project
                u = project_onto_subspace(R_list(t,original_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list(t, original_car_index, horizon_time_index);
            if do_project
                Udes = project_onto_subspace(R_list(t,original_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(2);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list(t, original_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_list(t,original_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_list(t,original_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(3);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list(t, original_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_list(t,original_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_list(t,original_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  

time_ = times_to_plot(4);
fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
t = ceil(time_ / Tim_step);
for car_index = 1:5
    switch car_index
        case 1
            is_original_car = 1;
            original_car_index = 1;
            color_ = 'r';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list(t, original_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_list(t,original_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list(t, original_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_list(t,original_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        for horizon_time_index = 1:Np
            u = u_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                u = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), u);
            end
            position_predicted = u;
            Udes = Udes_list_cutin(t, cutin_car_index, horizon_time_index, :);
            if do_project
                Udes = project_onto_subspace(R_weight_list_cutin(t,cutin_car_index), Udes);
            end
            position_assumed = Udes;
            plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s--','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
savefig(fig_, sprintf('%s%s.fig', path_save, title_))
saveas(gcf, sprintf('%s%s.png', path_save, title_))  


