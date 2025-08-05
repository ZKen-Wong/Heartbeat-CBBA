%% CBBA附加函数
function new_path = insert_at(path, task_id, pos)
    % 在path中第pos位置插入task_id
    new_path = [path(1:pos-1), task_id, path(pos:end)];
end

function [reward, total_time] = path_reward(cbba_auv, task_list, obstacle_list, path)
    pos = cbba_auv.pos;
    reward = 0;
    total_time = 0;
    total_mile=0;

    for idx = 1:length(path)
        tid = path(idx);
        entry = task_list(tid).entry;
        [travel_time, len, pos] = APF_path(pos, entry, obstacle_list, 0);
        exec_time = task_list(tid).duration(cbba_auv.id);
        total_time = total_time + travel_time + exec_time;
        reward = reward + task_list(tid).value(cbba_auv.id);
        total_mile = total_mile + len;
    end
end
