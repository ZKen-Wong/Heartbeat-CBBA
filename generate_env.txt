function [task_list, auv_list, obstacle_list] = generate_env(num_tasks, num_obs, num_auvs, seed, fail_num, entry_exit_split)
% 生成 AUV 环境，包括任务和障碍物信息
% 输入：
%   num_tasks         - 任务数量
%   num_obs           - 障碍物数量
%   num_auvs          - AUV数量（用于适应性配置）
%   seed              - 随机种子（用于复现）
%   entry_exit_split  - 出口入口是否分离（true/false）
% 输出：
%   task_list         - 任务结构体数组
%   auv_list          - auv结构体数组
%   obstacle_list     - 障碍物结构体数组

rng(seed);

%% 区域定义
xrange = [0, 3000];
yrange = [0, 3000];
zrange = [-200, 0];

safe_radius = 80; % 出入口保护范围
entry_exit_dist_range = [20, 50]; % 出入口间距

%% AUV生成
% 含有auv对每个任务的专长度，总里程限制，当前位置
auv_list = struct([]);
fail_auv = randi([1 num_auvs],1,fail_num);  % 随机故障auv id
fail_time = 2000 * rand(1,fail_auv) + 500; % 随机故障时间  500-2500s之间
for i = 1:num_auvs
    % AUV对每个任务的专长度
    expertises = rand(1, num_tasks);      % 随机专长度0-1
    mileage = 15000;
    x = zeros(12,1);
    x(1) = 0.1;      % 0.1 initial speed

    auv_list(i).id = i;
    auv_list(i).expertise = expertises;
    auv_list(i).mileage = mileage;
    auv_list(i).state = x;          % 1xN AUVs
    for j = 1:numel(fail_auv)
        if i == fail_auv(j)
            auv_list(i).fail_time = fail_time(j);
        else
            auv_list(i).fail_time = inf;
        end
    end
end

%% 任务生成
task_list = struct([]);
for i = 1:num_tasks
    % 出入口点
    entry = [rand_range(xrange), rand_range(yrange), rand_range(zrange)];
    if entry_exit_split
        angle = rand()*2*pi;
        dist = rand_range(entry_exit_dist_range);
        offset = dist * [cos(angle), sin(angle), rand_range([-0.5, 0.5])];
        exit = entry + offset;
    else
        exit = entry;
    end

    % 对每个AUV的任务价值和耗时
    expertise = vertcat(auv_list.expertise);
    values = 10 * expertise(:,i) ;      % 任务价值 [0-10]
    durations = -500 * expertise(:,i) + 600;  % 耗时 100~600s

    deadline = randi([1000, 3000]);

    task_list(i).entry = entry;
    task_list(i).exit = exit;
    task_list(i).value = values';          % 1xN AUVs
    task_list(i).duration = durations';    % 1xN AUVs
    task_list(i).deadline = deadline;
    task_list(i).id = i;
end

%% 障碍物生成
obstacle_list = struct([]);
count = 0;
while count < num_obs
    pos = [rand_range(xrange), rand_range(yrange), rand_range(zrange)];
    safe = true;
    for j = 1:num_tasks
        d1 = norm(pos - task_list(j).entry);
        d2 = norm(pos - task_list(j).exit);
        if d1 < safe_radius || d2 < safe_radius
            safe = false;
            break;
        end
    end
    if safe
        count = count + 1;
        obstacle_list(count).pos = pos;
        obstacle_list(count).radius = 10; % 固定障碍物半径
    end
end
end

%% 辅助函数
function val = rand_range(range)
    val = range(1) + rand() * (range(2) - range(1));
end
