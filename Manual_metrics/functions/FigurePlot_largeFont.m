%% »æÍ¼

close all

show_figure = 'on';  %--> on, off
save_figure = 0;  %--> 0, 1
path_save = './results_convergence/';
time_plot = 12;
font_size = 15;

fig_ = figure('Visible',show_figure);
T = 30;
t = (1:Time_sim/Tim_step)*Tim_step;
 plot(t,v0,'m:','linewidth',2);
hold on; plot(t, Velocity(:,1),'r','linewidth',2);
time_start_plot = t_cutin(2)/Tim_step;
plot(t(time_start_plot:end), Velocity_cutin(time_start_plot:end,2),'b--','linewidth',2);
plot(t, Velocity(:,2),'b','linewidth',2);
plot(t, Velocity(:,3),'k','linewidth',2);
time_end_plot = t_cutout(4)/Tim_step;
plot(t(1:time_end_plot-1), Velocity(1:time_end_plot-1,4),'g','linewidth',2);
plot(t, Velocity(:,5),'m','linewidth',2);
plot(t, Velocity(:,6),'c','linewidth',2);
plot(t, Velocity(:,7),'y','linewidth',2);
% h = legend('0','1','1.5','2','3','4','5','6','7','8','Location','SouthEast');
% set(h,'box','off', 'location', 'best'); 
set(gca,'FontSize',font_size)
set(gca,'XTick',[0,2,4,6,8,10,12]);
grid on;
box on;
xlabel('Time (s)');ylabel('Speed (m/s)');
% axis([0 T 19 floor(max(max(Velocity))+1)])
set(gcf,'Position',[250 150 300 350]);
xlim([0, time_plot])
title_ = 'speed';
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

fig_ = figure('Visible',show_figure);
plot(t, Torque(:,1),'r','linewidth',2);hold on;
time_start_plot = t_cutin(2)/Tim_step;
plot(t(time_start_plot:end), Torque_cutin(time_start_plot:end,2),'b--','linewidth',2);
plot(t, Torque(:,2),'b','linewidth',2);hold on;
plot(t, Torque(:,3),'k','linewidth',2);hold on;
time_end_plot = t_cutout(4)/Tim_step;
plot(t(1:time_end_plot-1), Torque(1:time_end_plot-1,4),'g','linewidth',2);
plot(t, Torque(:,5),'m','linewidth',2);hold on;
plot(t, Torque(:,6),'c','linewidth',2);hold on;
plot(t, Torque(:,7),'y','linewidth',2);hold on;
h = legend('1','1.5','2','3','4','5','6','7');
set(h,'box','off', 'location', 'best'); box on;
xlabel('Time (s)');ylabel('Torque (N)');
xlim([0 T])
set(gcf,'Position',[250 150 300 350]);
xlim([0, time_plot])
title_ = 'Torque';
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

fig_ = figure('Visible',show_figure);
plot(t, (Eta*Torque(:,1)/R(1) - Ca(1)*Velocity(:,1).^2 - Mass(1)*g*f)/Mass(1),'r','linewidth',2);hold on;
time_start_plot = t_cutin(2)/Tim_step;
plot(t(time_start_plot:end), (Eta*Torque_cutin(time_start_plot:end,2)/R_cutin(2) - Ca_cutin(2)*Velocity_cutin(time_start_plot:end,2).^2 - Mass_cutin(2)*g*f)/Mass_cutin(2),'b--','linewidth',2);
plot(t, (Eta*Torque(:,2)/R(2) - Ca(2)*Velocity(:,2).^2 - Mass(2)*g*f)/Mass(2),'b','linewidth',2);hold on;
plot(t, (Eta*Torque(:,3)/R(3) - Ca(3)*Velocity(:,3).^2 - Mass(3)*g*f)/Mass(3),'k','linewidth',2);hold on;
time_end_plot = t_cutout(4)/Tim_step;
plot(t(1:time_end_plot-1), (Eta*Torque(1:time_end_plot-1,4)/R(4) - Ca(4)*Velocity(1:time_end_plot-1,4).^2 - Mass(4)*g*f)/Mass(4),'g','linewidth',2);
plot(t, (Eta*Torque(:,5)/R(5) - Ca(5)*Velocity(:,5).^2 - Mass(5)*g*f)/Mass(5),'m','linewidth',2);hold on;
plot(t, (Eta*Torque(:,6)/R(6) - Ca(6)*Velocity(:,6).^2 - Mass(6)*g*f)/Mass(6),'c','linewidth',2);hold on;
plot(t, (Eta*Torque(:,7)/R(7) - Ca(7)*Velocity(:,7).^2 - Mass(7)*g*f)/Mass(7),'y','linewidth',2);hold on;
h = legend('1','1.5','2','3','4','5','6','7');
set(h,'box','off', 'location', 'best'); box on;
xlabel('Time (s)');ylabel('Acceleration (m/s^2)');
xlim([0 T])
set(gcf,'Position',[250 150 300 350]);
xlim([0, time_plot])
title_ = 'Acceleration';
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
cut_in_index_ahead = 1;
original_car_index = 1;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), x0 - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'r--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), x0(1:time_start_plot-1) - d - Postion(1:time_start_plot-1,original_car_index),'r','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'r','linewidth',2);hold on;
else  
    plot(t, x0 - d - Postion(:,original_car_index),'r','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 2;
original_car_index = 2;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'b--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'b','linewidth',2);hold on;
    plot([t(time_start_plot-1), t(time_start_plot)], ...
         [Postion(time_start_plot-1,original_car_index-1) - d - Postion(time_start_plot-1,original_car_index), Postion_cutin(time_start_plot,cut_in_index_ahead) - d - Postion(time_start_plot,original_car_index)], ...
         'b','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'b','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'b','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 3;
original_car_index = 3;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'k--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'k','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'k','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'k','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 4;
original_car_index = 4;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'g--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'g','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'g','linewidth',2);hold on;
else  
    if cutout_cars_in_platoon_so_far(original_car_index) == 0
        plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'g','linewidth',2);hold on;
    else
        time_start_plot = t_cutout(original_car_index)/Tim_step;
        plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'g','linewidth',2);hold on;
%         plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-2) - d - Postion(time_start_plot:end,original_car_index),'g','linewidth',2);hold on;
    end
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 5;
original_car_index = 5;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'m--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'m','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'m','linewidth',2);hold on;
else  
    if cutout_cars_in_platoon_so_far(original_car_index-1) == 0
        plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'m','linewidth',2);hold on;
    else
        time_start_plot = t_cutout(original_car_index-1)/Tim_step;
        plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'m','linewidth',2);hold on;
        plot([t(time_start_plot-1), t(time_start_plot)], ...
             [Postion(time_start_plot-1,original_car_index-1) - d - Postion(time_start_plot-1,original_car_index), Postion(time_start_plot,original_car_index-2) - d - Postion(time_start_plot,original_car_index)], ...
             'm','linewidth',2);hold on;
        plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-2) - d - Postion(time_start_plot:end,original_car_index),'m','linewidth',2);hold on;
    end
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 6;
original_car_index = 6;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'c--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'c','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'c','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'c','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 7;
original_car_index = 7;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion(time_start_plot:end,original_car_index-1) - d - Postion_cutin(time_start_plot:end,cut_in_index_ahead),'y--','linewidth',2);hold on;
    %%%%% original car:
    plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index-1) - d - Postion(1:time_start_plot-1,original_car_index),'y','linewidth',2);hold on;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead) - d - Postion(time_start_plot:end,original_car_index),'y','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index-1) - d - Postion(:,original_car_index),'y','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
% h = legend('1','1.5','2','3','4','5','6','7');
% set(h,'box','off', 'location', 'best'); 
set(gca,'FontSize',font_size)
set(gca,'XTick',[0,2,4,6,8,10,12]);
grid on;
box on;
xlabel('Time (s)');ylabel('Spacing error (m)');
xlim([0 T])
set(gcf,'Position',[250 150 300 350]);
xlim([0, time_plot])
ylim([-8, 11])
title_ = 'position_error';
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end

% figure;
% plot(t, Postion(:,1)- (x0- d),'r','linewidth',2);hold on;
% plot(t, Postion(:,2)-(x0- 2*d) ,'b','linewidth',2);hold on;
% plot(t, Postion(:,3)-(x0- 3*d),'k','linewidth',2);hold on;
% plot(t, Postion(:,4)-(x0- 4*d),'g','linewidth',2);hold on;
% plot(t, Postion(:,5)-(x0- 5*d),'m','linewidth',2);hold on;
% plot(t, Postion(:,6)-(x0- 6*d),'r--','linewidth',2);hold on;
% plot(t, Postion(:,7)-(x0- 7*d),'b--','linewidth',2);hold on;
% h = legend('1','2','3','4','5','6','7');
% set(h,'box','off'); box on;
% xlabel('Time (s)');ylabel('Spacing error (m)');
% xlim([0 T])
% set(gcf,'Position',[250 150 300 350]);


fig_ = figure('Visible',show_figure);
%%%%%%%%%%%%%%
plot(t, x0,'m:','linewidth',2);hold on;
%%%%%%%%%%%%%%
cut_in_index_ahead = 1;
original_car_index = 1;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'r--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'r','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'r','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 2;
original_car_index = 2;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'b--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'b','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'b','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 3;
original_car_index = 3;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'k--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'k','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'k','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 4;
original_car_index = 4;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'g--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'g','linewidth',2);hold on;
else  
    if cutout_cars_in_platoon_so_far(original_car_index) == 0
        plot(t, Postion(:,original_car_index),'g','linewidth',2);hold on;
    else
        time_start_plot = t_cutout(original_car_index)/Tim_step;
        plot(t(1:time_start_plot-1), Postion(1:time_start_plot-1,original_car_index),'g','linewidth',2);hold on;
    end
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 5;
original_car_index = 5;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'m--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'m','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'m','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 6;
original_car_index = 6;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'c--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'c','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'c','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
cut_in_index_ahead = 7;
original_car_index = 7;
if cutin_cars_in_platoon_so_far(cut_in_index_ahead) == 1  %--> cut in has occured
    %%%%% cut-in car:
    time_start_plot = t_cutin(cut_in_index_ahead)/Tim_step;
    plot(t(time_start_plot:end), Postion_cutin(time_start_plot:end,cut_in_index_ahead),'y--','linewidth',2);hold on;
    %%%%% original car:
    plot(t, Postion(:,original_car_index),'y','linewidth',2);hold on;
else  
    plot(t, Postion(:,original_car_index),'y','linewidth',2);hold on;
end
%%%%%%%%%%%%%%
% h = legend('0','1','1.5','2','3','4','5','6','7');
h = legend('0','1','cut-in','2','3','4','5','6','7');
set(h,'box','on', 'location', 'best'); 
set(gca,'FontSize',font_size)
set(gca,'XTick',[0,2,4,6,8,10,12]);
set(gca,'YTick',[-80,0,100,200,270]);
grid on;
box on;
xlabel('Time (s)');ylabel('Absolute Position (m)');
xlim([0 T])
set(gcf,'Position',[250 150 300 350]);
xlim([0, time_plot])
xlim([0 12])
ylim([-80 270])
title_ = 'position_absolute';
if ~exist(path_save, 'dir')
    mkdir(path_save)
end
if save_figure
    savefig(fig_, sprintf('%s%s.fig', path_save, title_))
    saveas(gcf, sprintf('%s%s.png', path_save, title_))  
end 