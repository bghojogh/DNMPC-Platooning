
close all
clc

position_or_velocity = 2;  %--> 1: position, 2: velocity
show_figure = 'on';  %--> on, off
save_figure = 0;  %--> 0, 1

times_to_plot = [1, t_cutin(2), t_cutout(4), 7];
do_project = 1;  %--> 0 or 1
if position_or_velocity == 1
    path_save = sprintf('./results/position/do_project=%d/', do_project);
else
    path_save = sprintf('./results/velocity/do_project=%d/', do_project);
end

plot_index = 0;
plotHandles = zeros(1,4);
plotLabels = cell(1,4);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2, 'DisplayName', 'a');hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
% set(gca,'XTickLabel',{'','1','','Cut-in','','2','5','7'})
% xlabel('Vehicle index')
% set(gca,'xtick',[])
set(gca,'XTickLabel',{'y^p_i','y^a_i','','','y^p_i','y^a_i','y^p_i','y^a_i'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'F', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'F', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'F', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i','y^p_i','y^a_i'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'F', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'F', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end


%% Q:
plot_index = 0;
plotHandles = zeros(1,4);
plotLabels = cell(1,4);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y_{des,i}','','','y^p_i','y_{des,i}','y^p_i','y_{des,i}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'Q', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'Q', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'Q', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}','y^p_i','y_{des,i}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'Q', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'Q', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

%% G:
plot_index = 0;
plotHandles = zeros(1,4);
plotLabels = cell(1,4);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_j - d_{i,j}','','','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'G', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'G', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'G', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(Xp_list(t, original_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(Xp_list_cutin(t, cutin_car_index, :, position_or_velocity));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}','y^p_i','y^a_j - d_{i,j}'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'G', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'G', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

%% R:
plot_index = 0;
plotHandles = zeros(1,4);
plotLabels = cell(1,4);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'u^p_i','h_i(v_i)','','','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'R', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'R', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'R', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

plot_index = 0;
plotHandles = zeros(1,5);
plotLabels = cell(1,5);
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
            legend_ = '1';
        case 2
            is_original_car = 0;
            cutin_car_index = 2;
            color_ = 'b';
            legend_ = 'cut-in';
        case 3
            is_original_car = 1;
            original_car_index = 2;
            color_ = 'b';
            legend_ = '2';
        case 4
            is_original_car = 1;
            original_car_index = 5;
            color_ = 'm';
            legend_ = '5';
        case 5
            is_original_car = 1;
            original_car_index = 7;
            color_ = 'y';
            legend_ = '7';
    end
    if is_original_car
        mean_ = mean(u_list(t, original_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s-','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    else
        mean_ = mean(u_list_cutin(t, cutin_car_index, :));
        plot_index = plot_index + 1;
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
            plotHandles(plot_index) = plot([(car_index-1)*2 + 1, (car_index-1)*2 + 2],[position_predicted, position_assumed],'s:','MarkerFaceColor',color_,'Color',color_,'linewidth',2);hold on;
            plotLabels{plot_index} = [legend_];
        end
    end
end
% set(gca,'XTickLabel',{'','Vehicle 1','','Cut-in Vehicle 2','','Vehicle 2','','Vehicle 5','','Vehicle 7'})
% set(gca,'XTickLabel',{'','Vehicle 1','Cut-in Vehicle 2','Vehicle 2','Vehicle 5','Vehicle 7'})
set(gca,'XTickLabel',{'u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)','u^p_i','h_i(v_i)'})
ylabel('Projected value onto metric subspace')
h = legend(plotHandles, plotLabels, 'Location', 'best');  %--> https://www.mathworks.com/matlabcentral/answers/269075-how-do-i-add-plots-to-a-legend-in-a-loop
if save_figure
    title_ = sprintf('Metric= %s, time= %0.2f seconds', 'R', time_);
else
    title_ = sprintf('Weight Matrix: %s, time: %0.0f seconds', 'R', time_);
end
title(title_)
title_(title_=='.') = ['_'];
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end


