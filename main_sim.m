
% seed_list=[23,11,33,579,57,786,78,85,99,114514];
% for aaa = 1:10
%% 仿真主程序 main simulation programme
clear all;
close();
clear cbba_orig traj hb_record gantt_record
seed = 34;
S.env = RandStream('Threefry','Seed',seed + 11);
S.fail = RandStream('Threefry','Seed',seed + 11); 

%% 初始化 生成环境 Initialize generate env
dt = 0.1;           % 仿真步长 sim step time
count = 0;          % 仿真轮次  num of sim round
num_tasks = 6;      % 任务总数 number of tasks
num_obs = 200;      % 障碍物总数number of obstacle
num_auvs = 3;       % auv总数   number of AUV
fail_num = 1;      % 故障auv数量 number of failure AUV

entry_exit_split = 0;   % task出入口是否分离
% 环境初始化  initialize environment
[task_list, auv_list, obstacle_list] = generate_env(num_tasks, num_obs, num_auvs, S, fail_num, entry_exit_split);
going(1:num_auvs) = true;   % auv是否移动  mark of moving
anime_switch = 1;        % 是否渲染动画   anime switch
temp_task_start_time = zeros(1,num_auvs);   % 记录停留原地执行任务的开始时间  mark the mission start time
executing(1:num_auvs) = false;              % 停留原地执行任务的标志 mark of executing task
init_time_sheet = true;                     % cbba第一次运行时计算所有路径的耗时，节省计算资源 
                                            % mark of first time cbba, calculate all path

[cbba_auv] = init_cbba(auv_list, num_tasks);  % cbba模块初始化 initalize cbba structure
updated = true;     % cbba是否运行的标志 mark of running cbba
last_updated = 0;   % cbba上一次的更新时间 last update time of cbba


%% PID initial
yaw_kp = 2;
yaw_ki =0;
yaw_kd =0;

pitch_kp = 0.1;  % 深度控制 0.1  pitch控制2
pitch_ki = 0;
pitch_kd = 0.5;  % 深度控制 0.5 pitch控制2
% 为每个 AUV 生成唯一的控制器 ID，并“注册/重置”一次
% Generate a unique controller ID for each AUV and "register/reset" it once
yaw_ids   = arrayfun(@(i) sprintf('yaw_%d',   i), 1:num_auvs, 'uni', false);
pitch_ids = arrayfun(@(i) sprintf('pitch_%d', i), 1:num_auvs, 'uni', false);
for i = 1:num_auvs
    pid_controller(yaw_ids{i},   0, 0, dt, yaw_kp,   yaw_ki,   yaw_kd,   true);  % reset
    pid_controller(pitch_ids{i}, 0, 0, dt, pitch_kp, pitch_ki, pitch_kd, true);  % reset
end
pid_vector = [yaw_kp, yaw_ki, yaw_kd, pitch_kp, pitch_ki, pitch_kd, dt];

%% 失控/心跳检测初始化 Failure/heartbeat detection initialization
hb = struct();
pos = zeros(num_auvs,3);
dis = zeros(num_auvs,3);
% 常量 constant
hb.R_comm      = 1000;    % m     commu range
hb.c_sound     = 1500;    % m/s   speed of sound
hb.T_HB        = 5;       % 心跳周期 (s)  heartbeat period
hb.tau_soft    = hb.R_comm/hb.c_sound + 2*hb.T_HB + 1.0;  % 小失效容忍 ≈ 0.667s + 1s
hb.tau_big_pad = 30.0;    % 大失效工程容差 (s)
hb.dt_cap_min  = 30.0;    % 预测失败兜底 (s)

% 延迟桶（仅心跳/上下线消息；不放CBBA信息）
% Delay bucket (only heartbeat/online and offline messages; no CBBA information)
hb.bucket = struct('type',{},'src',{},'dst',{},'t_emit',{},'t_arrive',{},'heard_row',{});

% （i观察j）：0=CONNECTED, 1=SOFT_LOSS(大失效倒计时), 2=DEAD
hb.link_state   = zeros(num_auvs);
hb.last_rx      = zeros(num_auvs);   % i<-j last time receive heartbeat
hb.t_soft_start = -inf(num_auvs);
hb.T_deadline   =  inf(num_auvs);

%% 数据记录的初始化 Initialization of data records
% 失效对象；小失效时间；大失效时间
% Failure AUV; soft loss time; dead time
hb_record = zeros(1,3); 
% CBBA allocation
% 执行者，task id，开始时间，结束时间
% Executor, task id, start time, end time
gantt_record = struct('auv',{},'task_id',{},'t_start',{},'t_end',{},'assign_round',{});
alloc_round = 0;

%% 初始化绘图 Initial plot
r = 15;
[X, Y, Z] = sphere(20);
if anime_switch
    figure;
    set(gcf,'unit','normalize','position',[0 0.3 0.35 0.35]) % 设置窗口生成位置和大小
    plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);  % 黑色auv朝向线
    hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('AUV APF Navigation');

    % 绘制障碍物 plot obstacle
    for i = 1:num_obs
        surf(obstacle_list(i).radius*X+obstacle_list(i).pos(1),...
            obstacle_list(i).radius*Y+obstacle_list(i).pos(2),...
            obstacle_list(i).radius*Z+obstacle_list(i).pos(3),...
            'FaceColor','r','EdgeColor','none','FaceAlpha',0.5);
    end
    % 绘制目标点 plot tasks
    for i = 1:num_tasks
        entry = task_list(i).entry;
        exit = task_list(i).exit;
        entryPlot = surf(r*X+entry(1), r*Y+entry(2), r*Z+entry(3), 'FaceColor','y','EdgeColor','none');
        exitPlot = surf(r*X+exit(1), r*Y+exit(2), r*Z+exit(3), 'FaceColor','m','EdgeColor','none');
        text(entry(1), entry(2), entry(3)+5*r, sprintf('T%d', i), 'Color','k', 'FontSize',10, 'FontWeight','bold');
        hold on;
    end
    % 初始化AUV与轨迹绘制 Initialize AUV and their trajectory
    auvPlot = gobjects(num_auvs,1);
    headingLine = gobjects(num_auvs,1); 
    trajPlot = gobjects(num_auvs,1);
    auvLabel = gobjects(num_auvs,1);
    for i = 1:num_auvs
        x = cbba_auv(i).state;
        auvPlot(i) = surf(r*X+x(7), r*Y+x(8), r*Z+x(9), 'FaceColor','b', 'EdgeColor','none');
        headingLine(i) = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 2);  % 黑色auv朝向线
        trajPlot(i) = plot3(nan, nan, nan, 'b-', 'LineWidth', 2);  % 轨迹
        auvLabel(i) = text(x(7), x(8), x(9)+3*r, sprintf('%d', i), ...
                       'Color','k', 'FontSize',10, 'FontWeight','bold', ...
                       'HorizontalAlignment','center');
    end
end

while any(going)
    count = count + 1;
    time = count * dt;
%% 移动迭代 move iteration
% 得到APF指向 模型仿真移动 无方案and执行任务时停止移动
% Get APF pointing model simulation movement No solution and stop moving when executing the task
    for i = 1:num_auvs
        if ~going(i)
            continue;
        end
        x = cbba_auv(i).state;
        pos = x(7:9)';
        pitch = x(11);
        yaw = x(12);
        if ~updated   % 没有在更新任务 No CBBA update
            if cbba_auv(i).temp_task == 0  %初始化当前任务变量 Initialize current task
                cbba_auv(i).temp_task = cbba_auv(i).temp_task + 1;
            end

            if cbba_auv(i).temp_task > length(cbba_auv(i).path)   %完成全部任务则停车Stop after all tasks done
                ui = zeros(1,6);
                going(i) = false;  % 这台auv结束行动 this auv stops
                tid = cbba_auv(i).path(max(1,cbba_auv(i).temp_task - 1));   % 当前执行的任务 id current task id
                task_list(tid).done = true;
                task_list(tid).done_reward = 0.9^(time/500) *  task_list(tid).value(i);
            else
                temp_task = cbba_auv(i).path(cbba_auv(i).temp_task);
                goal = task_list(find(vertcat(task_list.id) == temp_task, 1)).entry; %找到当前任务的位置 Find the location of the current task
                goal_dist = norm(pos - goal);
                if goal_dist < 3    % 检测到达任务位置 Arrival at task location
                    % if ~executing(i)
                    cbba_auv(i).temp_task = cbba_auv(i).temp_task + 1;% 切换下一个任务 switch to next task
                    temp_task_start_time(i) = time; % 记录执行开始时间 Record task start time
                    executing(i) = true;
                    % end
                else
                    ui = APF(x, goal, obstacle_list, yaw_ids{i}, pitch_ids{i} , pid_vector);
                    tid = cbba_auv(i).path(max(1,cbba_auv(i).temp_task - 1));       % 当前执行的任务 id current task id
                    if time - temp_task_start_time(i) >= task_list(tid).duration(i) % 任务执行时间已经完成 task executed
                        % 登记已经完成的任务 Register completed tasks
                        if ~task_list(tid).done && executing(i)
                            task_list(tid).done = true;
                            task_list(tid).done_reward = 0.9^(time/500) *  task_list(tid).value(i);
                        end
                        executing(i) = false;
                    end
                end
            end
        else
            ui = zeros(1,6);
        end
        % 状态更新 state update
        if executing(i)
            xdot = zeros(12,1);
        else
            xdot = auv(x, ui);
        end
        x = x + dt * xdot;
        traj(count,i,:) = x(7:9)';
        cbba_auv(i).state = x;
    end

    %% 心跳检测：发送 Heartbeat inspection: send
    for ii=1:num_auvs, pos(ii,:) = cbba_auv(ii).state(7:9)'; end
    for i = 1:num_auvs
        for j = 1:num_auvs
            d = norm(pos(i,:) - pos(j,:));
            dis(i,j) = d;
        end
    end
    
    if mod(time, hb.T_HB) < dt/2
        for i = 1:num_auvs
            if ~cbba_auv(i).alive, continue; end
            for k = 1:num_auvs
                if i==k, continue; end
                d = dis(i,k);
                if d <= hb.R_comm
                    tau = d / hb.c_sound;
                    hb.bucket(end+1) = struct('type','HEARTBEAT','src',i,'dst',k, ...
                        't_emit',time,'t_arrive',time+tau,'heard_row',hb.last_rx(i,:)); 
                end
            end
        end
    end
    %% 心跳检测：递送 Heartbeat inspection: receive
   if ~isempty(hb.bucket)
        keep = true(1,numel(hb.bucket));
        for m = 1:numel(hb.bucket)
            if hb.bucket(m).t_arrive <= time
                src = hb.bucket(m).src;
                dst = hb.bucket(m).dst;
                heard_row = hb.bucket(m).heard_row;
                if strcmp(hb.bucket(m).type,'HEARTBEAT')
                    hb.last_rx(dst,src) = time;                 % i<-j
                    cbba_auv(dst).commu_time(src) = time;       % 同步到已有字段 Synchronize to existing fields
                    % 将收到的最后通信表里的收发信人置零 
                    % Set the sender and receiver in the last commu table to zero
                    heard_row(dst) = 0; heard_row(src) = 0;     
                    hb.last_rx(dst,:) = max(hb.last_rx(dst,:), heard_row);  % 更新为最新的收信时间 update the lasest commu time
                end
                keep(m) = false;
            end
        end
        hb.bucket = hb.bucket(keep);
   end
   %% 小/大失效状态机（仅 DEAD 触发重分配) soft/firm dead state machine
    for i = 1:num_auvs      % i是收信人,本地auv i is the receiver, local auv
        if ~cbba_auv(i).alive, continue; end % dead  不再更新
        for j = 1:num_auvs  % j是发信人 j is the sender
            if i==j, continue; end
    
            dt_no_rx = time - hb.last_rx(i,j);
    
            % 连通 communicable
            if dt_no_rx <= hb.tau_soft
                hb.link_state(i,j) = 0;  % CONNECTED
                hb.t_soft_start(i,j) = inf;
                hb.T_deadline(i,j) = inf;
                continue; 
            end
    
            % 刚进入小失效：启动大失效倒计时 soft loss start: dead count down
            if hb.link_state(i,j) == 0
                hb.link_state(i,j)   = 1;             % SOFT_LOSS
                hb.t_soft_start(i,j) = time;
    
                % 预测下次进入通信半径时间 estimate next commu time
                if dis(i,j)<hb.R_comm
                    t_meet = 0;
                else
                [TIME_SHEET, LEN_SHEET] = fetch_time_sheets();
                t_meet = predict_next_contact(i, j, time, hb.R_comm, ...
                          cbba_auv, task_list, TIME_SHEET, LEN_SHEET, num_auvs);
                end
                % if ~isfinite(t_meet), t_meet = time + hb.dt_cap_min; end
                %  永远无法重新通讯那就无法通，不要为了兜底加入一个固定值
    
                % 大失效截止：预计相遇剩余 + 30s Dead Deadline: Estimated Next Commu + 30s
                hb.T_deadline(i,j) = time + max(0, t_meet - time) + hb.tau_big_pad;
                % 记录小失效触发 record soft loss triggered
                hb_record(end+1,:) = [j,time,0];
            end
    
            % 大失效触发 Dead triggered
            if hb.link_state(i,j) == 1 && time >= hb.T_deadline(i,j)
                hb.link_state(i,j) = 2;   % DEAD
                [cbba_auv, did_release] = handle_dead(cbba_auv, j, num_tasks);
                if did_release
                    updated = true;       % 触发下一轮 CBBA trigger next CBBA
                    init_time_sheet = true;
                    % 记录大失效触发 record Dead
                    hb_record(end+1,:) = [j,0,time];
                end
            end
        end
    end
    %% （if）启动task allocation
    if updated
        [cbba_auv,updated,init_time_sheet]= cbba_orig(cbba_auv,task_list, obstacle_list, ...
            time, pid_vector, updated, last_updated, init_time_sheet);
        if ~updated
            % 完成分配后记录方案 Record the allocation after CBBA finish
            alloc_round = alloc_round + 1;
            for n = 1:num_auvs
                if isempty(cbba_auv(n).path), continue; end 
                t_arrive = cbba_auv(n).time_list;
                for k = 1:numel(cbba_auv(n).path)
                    tid = cbba_auv(n).path(k);
                    t_start_abs = t_arrive(k);
                    t_end_abs   = t_start_abs + task_list(tid).duration(cbba_auv(n).id);
                    gantt_record(end+1) = struct( ...
                        'auv',     cbba_auv(n).id, ...
                        'task_id',      tid, ...
                        't_start', t_start_abs, ...
                        't_end',   t_end_abs, ...
                        'assign_round',alloc_round);  
                end 
            end

        end
    end
    
    %% 随机auv失效 Random AUV failure
    for idx = 1:num_auvs
        if cbba_auv(idx).alive && time >= cbba_auv(idx).fail_time
            cbba_auv(idx).alive = false;   % 停止发心跳 Stop Heartbeat
            going(idx) = false;            % 也可改成漂移，这里直接停
            fprintf('AUV %d failed at t=%.1f s\n', idx, time);
        end
    end

    %% 更新仿真 绘制动画 Update Simulation Draw Animation
    for i = 1:num_auvs
        if ~going(i)
            continue;
        end
        x = cbba_auv(i).state;
        if mod(count,200) == 0 && anime_switch  % speed up the anima
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
            set(auvLabel(i), 'Position', [x(7), x(8), x(9)+3*r]);
            hold on;
            xlim([-50 3050]);
            ylim([-50 3050]);
            zlim([-230 50]);
            drawnow;
        end
    end

end
%% 事后统计数据 statistics after simulation
% 分析心跳机制 Analyze heartbeat mechanism
[summary_hb, perAUV_hb] = analyze_hb(hb_record, cbba_auv,seed);
% 画gunchat
plot_cbba_timeline(gantt_record, cbba_auv, seed);
% 分析reward recover 情况
% reward is the total reward of the whole allocation分配方案
reward = 0;
for task_id = 1:num_tasks
    reward = reward + task_list(task_id).done_reward;
end
done = sum(vertcat(task_list.done));
reward_record = struct('seed',seed,'auv_num', num_auvs, ...
    'fail_num',fail_num, 'task_num', num_tasks,'assign_round',alloc_round, ...
    'task_done', done, 'reward',reward);
append_reward_table(reward_record, seed);
% end
    %% 附加函数 function
    function [TIME_SHEET, LEN_SHEET] = fetch_time_sheets()
        if evalin('base','exist(''TIME_SHEET'',''var'')')
            TIME_SHEET = evalin('base','TIME_SHEET');
            LEN_SHEET  = evalin('base','LEN_SHEET');
        else
            TIME_SHEET = []; LEN_SHEET = [];
        end
    end
    
    function [cbba_auv, did_release] = handle_dead(cbba_auv, dead_id, num_tasks)
        did_release = false;
        for i=1:numel(cbba_auv)
            mask = (cbba_auv(i).auv_win == dead_id);
            if any(mask)
                cbba_auv(i).auv_win(mask) = 0;
                cbba_auv(i).big_bid(mask) = 0;
                did_release = true;
            end
            if ~isempty(cbba_auv(i).bundle)
                cbba_auv(i).bundle(ismember(cbba_auv(i).bundle, find(mask))) = [];
            end
            if ~isempty(cbba_auv(i).path)
                cbba_auv(i).path(ismember(cbba_auv(i).path, find(mask))) = [];
            end
        end
    
        % —— 若“新 AUV 加入要全局重拍”，在加入时用下列替代：
        % for i=1:numel(cbba_auv)
        %     cbba_auv(i).auv_win(:)=0; cbba_auv(i).big_bid(:)=0;
        %     cbba_auv(i).bundle=[]; cbba_auv(i).path=[];
        % end
        % did_release = true;
    end
    
    function t_meet = predict_next_contact(i, j, t_now, R, cbba_auv, task_list, TIME_SHEET, LEN_SHEET, num_auvs)
        if isempty(TIME_SHEET), t_meet = inf; return; end
        [T_i, P_i] = build_timeline_for_agent(cbba_auv(i), task_list, TIME_SHEET, LEN_SHEET, num_auvs);
        [T_j, P_j] = build_timeline_for_agent(cbba_auv(j), task_list, TIME_SHEET, LEN_SHEET, num_auvs);
        if isempty(T_i), T_i = t_now; P_i = cbba_auv(i).state(7:9)'; end  %兼容空路径
        if isempty(T_j), T_j = t_now; P_j = cbba_auv(j).state(7:9)'; end
        T_i = T_i + t_now; T_j = T_j + t_now;    % 将相对时间变为绝对时间
    
        t_meet = inf; k_i=1; k_j=1; ti=t_now;               % 初始化区间扫描
        Pi_prev = cbba_auv(i).state(7:9)'; ti_prev=t_now;   % i在上一路径点的位置
        Pj_prev = cbba_auv(j).state(7:9)'; tj_prev=t_now;
    
        while true
            ti_next = (k_i<=numel(T_i))*T_i(k_i) + (k_i>numel(T_i))*inf;    % 访问下一个路径点的时间
            tj_next = (k_j<=numel(T_j))*T_j(k_j) + (k_j>numel(T_j))*inf;
            t_end = min(ti_next, tj_next);                                  % 扫描区间终点
            if ~isfinite(t_end) || t_end<=ti, break; end                    % 没有更多区间扫描则结束循环
    
            if isfinite(ti_next)
                vi = (P_i(k_i,:)-Pi_prev)/max(1e-6, T_i(k_i)-ti_prev);
            else, vi=[0 0 0]; end
            % 上下都是：计算匀速速度
            if isfinite(tj_next)
                vj = (P_j(k_j,:)-Pj_prev)/max(1e-6, T_j(k_j)-tj_prev);
            else, vj=[0 0 0]; end
            
            % r0：区间起点时刻两者的相对位置向量（i − j）
            r0 = (Pi_prev + vi*max(0,ti-ti_prev)) - (Pj_prev + vj*max(0,ti-tj_prev));
            v  = vi - vj;  % 相对速度
            if norm(v)<1e-9
                t_star=0; d_min=norm(r0);
            else        % 距离最小点d_min在导数为0处
                t_star = max(0, min(t_end-ti, -dot(r0,v)/(dot(v,v)+1e-12)));
                d_min = norm(r0 + v*t_star);
            end
            if d_min <= R, t_meet = ti + t_star; return; end
    
            ti = t_end;  % 推进至下一个区间
            if ti>=ti_next && isfinite(ti_next)
                Pi_prev=P_i(k_i,:);
                ti_prev=T_i(k_i); k_i=k_i+1;
            end
            if ti>=tj_next && isfinite(tj_next)
                Pj_prev=P_j(k_j,:); tj_prev=T_j(k_j);
                k_j=k_j+1; 
            end
        end
    end
    
    function [T, P] = build_timeline_for_agent(auv, task_list, TIME_SHEET, LEN_SHEET, num_auvs)

        % LEN_SHEET 预留
        % T 到达各个路径点的累计时间   P记录各个路径点位置
        T = []; P = [];
        cur_row = auv.id;
        t_acc = 0;
        for idx = 1:numel(auv.path)
            j = auv.path(idx);
            dt = TIME_SHEET(cur_row, j);
            if ~isfinite(dt) || dt<=0, continue; end
            t_acc = t_acc + dt;
            T(end+1) = t_acc;
            P(end+1,:) = task_list(j).entry(:)';
            exec_dt = task_list(j).duration(auv.id);
            if isfinite(exec_dt) && exec_dt > 0
                t_acc = t_acc + exec_dt;
                T(end+1)    = t_acc;
                P(end+1, :) = task_list(j).entry(:)';
            end
            cur_row = num_auvs + j;  % 下一段从该任务出口出发
        end
    end