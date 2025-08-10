%% This is the original CBBA from IEEE TRANSACTIONS ON ROBOTICS, VOL. 25, NO. 4, AUGUST 2009
function [cbba_auv,updated,init_time_sheet] = cbba(cbba_auv,task_list, obstacle_list,time, pid_vector,updated,init_time_sheet)

[~,num_auvs] = size(cbba_auv);
[~,num_tasks] = size(task_list);
updated = false;
% 使用持久变量字典
persistent time_sheet 
persistent len_sheet
% timesheet记录auv当前位置和任务各个路径点之间的通行耗时,没有auv到达点，行首先是auv，然后是任务，均为从任务出口前往任务入口，行为出发，列为到达;
% len sheet同理
if isempty(time_sheet)
    time_sheet = zeros(num_tasks+num_auvs,num_tasks);
    time_sheet(:) = inf;
    len_sheet = zeros(num_tasks+num_auvs,num_tasks);
    len_sheet(:) = inf;
end
%% 计算各个路径点之间的耗时，建表
if init_time_sheet
    for i = 1:num_auvs+num_tasks
        % fprintf('time sheet %d\n',i);
        for j = 1:num_tasks
            fprintf('row: %d col: %d\n',i,j);
            if i <= num_auvs  % 从auv当前位置出发
                start_pos = cbba_auv(i).state(7:9)';
                arrive = task_list(j).entry;
                [time_cost, leng, ~] = APF_path(start_pos, arrive, obstacle_list, pid_vector, 0);
                time_sheet(i,j) = time_cost;
                len_sheet(i,j) = leng;
            else                % 从任务出口出发
                start_pos = task_list(i-num_auvs).exit;
                arrive = task_list(j).entry;
                [time_cost, leng, ~] = APF_path(start_pos, arrive, obstacle_list, pid_vector, 0);
                time_sheet(i,j) = time_cost;
                len_sheet(i,j) = leng;
            end
        end
    end
    disp('time sheet and len sheet done')
    init_time_sheet =false;
end
%% build bundle 构建任务包

[cbba_auv,updated] = cbba_bidding(cbba_auv, task_list, time_sheet, len_sheet,num_auvs,num_tasks,updated);

fprintf('bundle construc at time:%d\n',time);

%% 通信模拟


%% Conflic Resolution 冲突解决

[cbba_auv,updated] = cbba_consensus(cbba_auv, num_auvs,updated);

fprintf('conflic resolve at time:%d\n',time);

%% CBBA 拍卖部分
% CBBA Bundle Construction
function [cbba_auv,updated] = cbba_bidding(cbba_auv, task_list, time_sheet, len_sheet,num_auvs,num_tasks,updated)
    for i = 1:num_auvs
        best_bid = -inf;
        best_task = -1;
        best_insert_pos = -1;
    
        % 枚举所有任务
        for j = 1:num_tasks
            if ismember(j, cbba_auv(i).bundle)
                continue;  % 已在bundle中，跳过
            end
            if cbba_auv(i).auv_win(j) ~= 0 && cbba_auv(i).auv_win(j) ~= i
                continue;  % 任务被别人赢得
            end
    
            % 在当前路径中尝试插入任务j，找出最高的bid位置
            cur_path = cbba_auv(i).path;
            [cur_reward, duration, mile] = path_reward(cbba_auv(i), task_list, time_sheet, len_sheet, cur_path, num_auvs);
            best_bid_j = -inf;
            best_pos_j = 0;
            
            % 遍历插入位置，标记任务J的最高出价和插入path的位置
            for pos = 0:length(cur_path)
                new_path = insert_at(cur_path, j, pos+1); % 插入后新的path
                [reward, duration, mile] = path_reward(cbba_auv(i), task_list, time_sheet, len_sheet, new_path, num_auvs);
                cbba_auv(i).mile = mile;
                % 对任务j的评估出价
                % bid = reward / (duration + 1e-5);  % 防止除0
                bid = reward - cur_reward;
                
                % 超过航程就不更新最优bid和插入位置
                if mile > cbba_auv(i).mileage
                    % fprintf('auv %d try task %d at %d over mile \n',i,j,pos);
                    continue;
                else % 非最优那也不更新
                    if bid > best_bid_j && bid > 0
                        best_bid_j = bid;
                        best_pos_j = pos + 1;
                        % fprintf('auv %d try task %d at %d for %d m \n',i,j,best_pos_j,round(mile));
                    end
                end
            end
    
            % 遍历所有可选任务，判断是否是当前所有任务中最好的一个
            if best_bid_j > best_bid
                best_bid = best_bid_j;
                best_task = j;
                best_insert_pos = best_pos_j;
            end
        end
    
        % 如果中标任务J，加入bundle和path 更新auv_win 更新big_bid
        if best_task ~= -1 && best_bid > cbba_auv(i).big_bid(best_task)
            cbba_auv(i).bundle(end+1) = best_task;
            cbba_auv(i).path = insert_at(cur_path, best_task, best_insert_pos);
            cbba_auv(i).big_bid(best_task) = best_bid;
            cbba_auv(i).auv_win(best_task) = i;
            updated = true;
        end
    end
end

%% CBBA 共识部分
function [cbba_auv,updated] = cbba_consensus(cbba_auv, num_auvs,updated)
    for i = 1:num_auvs
        for k = 1:num_auvs
            if i == k || cbba_auv(i).commu(k) == 0  % 跳过自我通信和无信号的通信
                continue;
            end
    
            for task_id = 1:num_tasks
                z_ij = cbba_auv(i).auv_win(task_id);    % agent i 自己认为谁赢了任务 j
                z_kj = cbba_auv(k).auv_win(task_id);    % agent j（k）认为谁赢了任务 j
    
                y_ij = cbba_auv(i).big_bid(task_id);    % agent i 的全局最高出价表
                y_kj = cbba_auv(k).big_bid(task_id);    % agent j 的全局最高出价表
    
                s_i = cbba_auv(i).commu_time;           % agent i 的最后通讯时间戳
                s_k = cbba_auv(k).commu_time;           % agent i 的最后通讯时间戳
    
                % 定义接收者的行为（默认 leave，不变）
                action = 'leave';
    
                if z_kj == k                       % k 认为自己赢
                    if z_ij == i && y_kj > y_ij
                        action = 'update';
                    elseif z_ij == k
                        action = 'update';
                    elseif z_ij ~= i && z_ij ~= k
                        if y_kj > y_ij || s_k(z_ij) > s_i(z_ij)
                            action = 'update';
                        end
                    elseif z_ij == 0
                        action = 'update';
                    end
                elseif z_kj == i                    % k 认为 i 赢
                    if z_ij == i
                        action = 'leave';
                    elseif z_ij == k
                        action = 'reset';
                    elseif z_ij ~= i && z_ij ~= k
                        if s_k(z_ij) > s_i(z_ij)
                            action = 'reset';
                        end
                    elseif z_ij == 0
                        action = 'leave';
                    end
                elseif z_kj ~= i && z_kj ~= k && z_kj ~= 0  % k 认为 m 赢（m ≠ i,j）
                    m = z_kj;
                    s_km = s_k(m);
                    s_im = s_i(m);
    
                    if z_ij == i && s_km > s_im && y_kj > y_ij
                        action = 'update';
                    elseif z_ij == k
                        if s_km > s_im
                            action = 'update';
                        else
                            action = 'reset';
                        end
                    elseif z_ij == m
                        if s_km > s_im
                            action = 'update';
                        end
                    elseif z_ij ~= i && z_ij ~= k && z_ij ~= m && z_ij ~= 0
                        n = z_ij;
                        s_kn = s_k(n);
                        s_in = s_i(n);
                        if s_km > s_im && s_kn > s_in
                            action = 'update';
                        elseif s_km > s_im && y_kj > y_ij
                            action = 'update';
                        elseif s_kn > s_in && s_im > s_km
                            action = 'reset';
                        end
                    elseif z_ij == 0
                        if s_km > s_im
                            action = 'update';
                        end
                    end
                elseif z_kj == 0                % k 认为 该任务无人认领
                    if z_ij == i
                        action = 'leave';
                    elseif z_ij == k
                        action = 'update';
                    elseif z_ij ~= i && z_ij ~= k && z_ij ~= 0
                        if s_k(z_ij) > s_i(z_ij)
                            action = 'update';
                        end
                    elseif z_ij == 0
                        action = 'leave';
                    end
                end
    
                % 执行对应行为
                if strcmp(action, 'update')
                    if cbba_auv(i).auv_win(task_id) ~= z_kj || cbba_auv(i).big_bid(task_id) ~= y_kj
                        cbba_auv(i).auv_win(task_id) = z_kj;
                        cbba_auv(i).big_bid(task_id) = y_kj;
                        % 查找 task_id 在 bundle 中的位置
                        b_idx = find(cbba_auv(i).bundle == task_id);
                        if ~isempty(b_idx)
                            % 从该位置起后续所有任务全部释放
                            for n = b_idx:length(cbba_auv(i).bundle)
                                t_n = cbba_auv(i).bundle(n);
                                if t_n == 0
                                    break;
                                end
                                % 清除bundle和path
                                cbba_auv(i).path(cbba_auv(i).path == t_n) = [];
                                cbba_auv(i).bundle(n) = [];
                            end
                        end
                        updated = true;
                    end
                elseif strcmp(action, 'reset')
                    % 清除本地与任务 task_id 相关的 bundle 和 path
                    cbba_auv(i).auv_win = zeros(1,num_tasks);
                    cbba_auv(i).big_bid = zeros(1,num_tasks);
                    cbba_auv(i).bundle = [];
                    cbba_auv(i).path = [];
                    updated = true;
                elseif strcmp(action, 'leave')
                    % 什么也不做
                end
            end
            % 更新时间戳
            cbba_auv(i).commu_time(k) = time;
            cbba_auv(k).commu_time(i) = time;
        end
    end
end

%% CBBA附加函数
function new_path = insert_at(path, task_id, best_insert_pos)
    % 在path中第pos位置插入task_id
    % disp(best_insert_pos);
    new_path = [path(1:best_insert_pos-1), task_id, path(best_insert_pos:end)];
end

function [reward, total_time, total_mile] = path_reward(cbba_auv, task_list, time_sheet, len_sheet, path, num_auvs)
    start = cbba_auv.id;
    reward = 0;
    total_time = 0;
    total_mile=0;
    time_factor = 0.9; % reward时间衰减因子
    
    if ~isempty(path)
        for idx = 1:length(path)
            tid = path(idx);
            entry = task_list(tid).id;
            travel_time = time_sheet(start,entry);
            len = len_sheet(start,entry);
            start = entry + num_auvs;
            exec_time = task_list(tid).duration(cbba_auv.id);
            total_time = total_time + travel_time + exec_time;
            reward = reward + time_factor^(total_time/500) *  task_list(tid).value(cbba_auv.id);
            total_mile = total_mile + len;
        end
    else
        reward = 0;
        total_time = 0;
        total_mile = 0;
    end
end
end