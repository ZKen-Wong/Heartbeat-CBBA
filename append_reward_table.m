function append_reward_table(reward_record, seed, excel_file, excel_sheet)
% 从 reward_record 中提取数据并追加写入 Excel
%
% reward_record: struct 数组，至少包含字段：
%   - auv_num
%   - fail_num
%   - reward
% excel_file:    Excel 文件名（默认 'analyze.xlsx'）
% excel_sheet:   sheet 名（默认 'Sheet1'）

    if nargin < 3 || isempty(excel_file),  excel_file  = 'analyze.xlsx'; end
    if nargin < 4 || isempty(excel_sheet), excel_sheet = 'reward';      end

    % 按 AUV 数分组
    auv_nums = unique([reward_record.auv_num]);

    % 表头
    headers = {'Seed','AUV number','Task number','Task Done','Failure number', ...
               'No Failure Reward','Reward of Proposed','Done rate'};

    % 如果文件不存在，写表头
    if ~isfile(excel_file)
        writecell(headers, excel_file, 'Sheet', excel_sheet, 'Range', 'A1');
        next_row = 2;
    else
        try
            raw = readcell(excel_file, 'Sheet', excel_sheet);
        catch
            writecell(headers, excel_file, 'Sheet', excel_sheet, 'Range', 'A1');
            raw = {};
        end
        next_row = size(raw,1) + 1;
        if next_row < 2, next_row = 2; end
    end

    % 遍历每个 AUV 数，提取无失效 & 有失效的收益
    for n_auv = auv_nums
        % 无失效
        idx_no_fail = ([reward_record.auv_num] == n_auv) & ([reward_record.fail_num] == 0);
        if any(idx_no_fail)
            R_no_fail = reward_record(idx_no_fail).reward;
            if numel(R_no_fail) > 1
                R_no_fail = mean(R_no_fail); % 多次取平均
            end
        else
            R_no_fail = NaN;
        end

        % Proposed（有失效）
        idx_fail = ([reward_record.auv_num] == n_auv) & ([reward_record.fail_num] > 0);
        if any(idx_fail)
            R_prop = reward_record(idx_fail).reward;
            if numel(R_prop) > 1
                R_prop = mean(R_prop);
            end
            fail_num = reward_record(find(idx_fail,1)).fail_num;
        else
            R_prop = NaN;
            fail_num = NaN;
        end

        tasks_num = reward_record.task_num;
        task_done = reward_record.task_done;
        done_rate = task_done/tasks_num;

        % 组织一行
        row = {seed, n_auv, tasks_num, task_done, fail_num, R_no_fail, R_prop,done_rate};

        % 写入 Excel
        writecell(row, excel_file, 'Sheet', excel_sheet, ...
                  'Range', sprintf('A%d', next_row));
        next_row = next_row + 1;
    end
end
