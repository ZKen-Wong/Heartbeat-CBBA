%% 初始化每个AUV的数据结构 同时计算auv在当前位置下的，从各点到各点的耗时
function [cbba_auv] = init_cbba(auv_list, num_tasks)
cbba_auv = auv_list;
[~,num_auvs] = size(auv_list);
for i = 1:num_auvs
    cbba_auv(i).big_bid = zeros(1,num_tasks);  % biggest bid in the whole auv cluster for now
    cbba_auv(i).auv_win = zeros(1,num_tasks);  % who win and take this task
    cbba_auv(i).bundle = [];   % bundle of this auv
    cbba_auv(i).path = [];     % path of this auv
    cbba_auv(i).temp_task = 0;
    commu = ones(1,num_auvs);
    cbba_auv(i).commu = commu;                 % 通讯邻接矩阵
    cbba_auv(i).commu_time = zeros(1,num_auvs);% 通讯时间戳
end