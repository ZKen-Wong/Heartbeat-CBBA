%% 仿真主程序
clear();
close();
seed = 22;

%% 初始化 生成环境
dt = 0.1;
count = 0;
num_tasks = 8;
num_obs = 200;
num_auvs = 1;
entry_exit_split = 0;
[task_list, auv_list, obstacle_list] = generate_env(num_tasks, num_obs, num_auvs, seed, entry_exit_split);
move = false;
going(1:num_auvs) = true;
anime_switch = true;
temp_task_start_time = zeros(1,num_auvs);   % 记录停留原地执行任务的开始时间
executing(1:num_auvs) = false;              % 停留原地执行任务的标志
init_time_sheet = true;                     % cbba第一次运行时计算所有路径的耗时，节省计算资源

[cbba_auv] = init_cbba(auv_list, num_tasks);  % cbba模块初始化
updated = true;     % cbba是否运行的标志


%% PID initial
yaw_kp = 2;
yaw_ki =0;
yaw_kd =0;
pid_controller('yaw_main', 0, 0, dt, yaw_kp, yaw_ki, yaw_kd, true);
pitch_kp = 2;
pitch_ki = 0.01;
pitch_kd = 2;
pid_controller('pitch_main', 0, 0, dt, pitch_kp, pitch_ki, pitch_kd, true);
pid_vector = [yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, dt];

%% 初始化绘图
r = 15;
[X, Y, Z] = sphere(20);
if anime_switch
    figure;
    set(gcf,'unit','normalize','position',[0.55 0.3 0.35 0.35]) % 设置窗口生成位置和大小
    plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);  % 黑色auv朝向线
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('AUV APF Navigation');

    % 绘制障碍物
    for i = 1:num_obs
        surf(obstacle_list(i).radius*X+obstacle_list(i).pos(1),...
            obstacle_list(i).radius*Y+obstacle_list(i).pos(2),...
            obstacle_list(i).radius*Z+obstacle_list(i).pos(3),...
            'FaceColor','r','EdgeColor','none','FaceAlpha',0.5);
    end
    % 绘制目标点
    for i = 1:num_tasks
        entry = task_list(i).entry;
        exit = task_list(i).exit;
        entryPlot = surf(r*X+entry(1), r*Y+entry(2), r*Z+entry(3), 'FaceColor','y','EdgeColor','none');
        exitPlot = surf(r*X+exit(1), r*Y+exit(2), r*Z+exit(3), 'FaceColor','m','EdgeColor','none');
        hold on;
    end
    % 初始化AUV与轨迹绘制
    auvPlot = gobjects(num_auvs,1);
    headingLine = gobjects(num_auvs,1); 
    trajPlot = gobjects(num_auvs,1);
    for i = 1:num_auvs
        x = cbba_auv(i).state;
        auvPlot(i) = surf(r*X+x(7), r*Y+x(8), r*Z+x(9), 'FaceColor','b', 'EdgeColor','none');
        headingLine(i) = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);  % 黑色auv朝向线
        trajPlot(i) = plot3(nan, nan, nan, 'b-', 'LineWidth', 2);  % 轨迹
    end
end

while any(going)
    count = count + 1;
    time = count * dt;
%% 无方案时不移动 得到APF指向 模型仿真移动
    for i = 1:num_auvs
        if ~going(i)
            continue;
        end
        x = cbba_auv(i).state;
        pos = x(7:9)';
        pitch = x(11);
        yaw = x(12);
        if ~updated   % 没有在更新任务
            if cbba_auv(i).temp_task == 0  %初始化变量当前任务
                cbba_auv(i).temp_task = cbba_auv(i).temp_task + 1;
            end

            if cbba_auv(i).temp_task > length(cbba_auv(i).path)    %完成全部任务则停车
                ui = zeros(1,6);
                going(i) = false;  % 这台auv结束行动
            else
                temp_task = cbba_auv(i).path(cbba_auv(i).temp_task);
                goal = task_list(find(vertcat(task_list.id) == temp_task, 1)).entry; %找到当前任务的位置
                goal_dist = norm(pos - goal);
                if goal_dist < 3            %检测到达位置
                    if ~executing(i)
                        cbba_auv(i).temp_task = cbba_auv(i).temp_task + 1;% 切换下一个任务
                        temp_task_start_time(i) = time;                   % 记录执行开始时间
                        executing(i) = true;
                    end
                else
                    ui = APF(x, goal, obstacle_list, pid_vector);
                    if time - temp_task_start_time(i) >= task_list(find(vertcat(task_list.id) == temp_task, 1)).duration(i)  % 任务执行时间已经完成
                        executing(i) = false;
                    end
                end
            end
        else
            ui = zeros(1,6);
        end
        %% 状态更新
        if executing(i)
            xdot = zeros(12,1);
        else
            xdot = auv(x, ui);
        end
        x = x + dt * xdot;
        traj(count,i,:) = x(7:9)';
        cbba_auv(i).state = x;
    end
    %% （是否）启动task allocation
    if updated
        [cbba_auv,updated,init_time_sheet]= cbba_orig(cbba_auv,task_list, obstacle_list, time, pid_vector, updated,init_time_sheet);
    end
    %% 更新仿真 绘制动画
    for i = 1:num_auvs
        if ~going(i)
            continue;
        end
        x = cbba_auv(i).state;
        if mod(count,100) == 0 && anime_switch
            % 朝向单位向量（以 body x 方向为基础）
            dir_len = 30;  % 线段长度
            dx = dir_len * cos(pitch) * cos(yaw);
            dy = dir_len * cos(pitch) * sin(yaw);
            dz = -dir_len * sin(pitch);  % z 轴向下为正
            
            % 起点和终点
            p_start = pos;
            p_end = pos + [dx, dy, dz];
            
            % 更新朝向线段
            set(headingLine(i), 'XData', [p_start(1), p_end(1)], ...
                             'YData', [p_start(2), p_end(2)], ...
                             'ZData', [p_start(3), p_end(3)]);
            % 更新auv，轨迹
            title(['AUV APF Navigation ',num2str(time),'s']);
            set(auvPlot(i),'XData',r*X+x(7),'YData',r*Y+x(8),'ZData',r*Z+x(9));
            set(trajPlot(i), 'XData', traj(1:count,i,1), ...
                 'YData', traj(1:count,i,2), ...
                 'ZData', traj(1:count,i,3));
            hold on;
            xlim([-50 3050]);
            ylim([-50 3050]);
            zlim([-230 50]);
            drawnow;
        end
    end
    %% 记录数据
end