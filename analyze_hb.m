function [summary, perAUV] = analyze_hb(hb_record, cbba_auv,seed)
% 分析 heartbeat 失效检测性能
% 输入:
%   hb_record: n×4 矩阵 [target_id, t_soft, t_dead, reserved]
%   cbba_auv:  1×num_auvs struct 数组，需包含字段 fail_time
%   sim_time:  仿真结束时间
%
% 输出:
%   summary:   总体统计结果 (struct)
%   perAUV:    每个 AUV 的检测明细 (struct array)
%
% 指标:
%   - SOFT FP/TP/反应时间
%   - DEAD FP/FN/TP/反应时间

    num_auvs = numel(cbba_auv);
    fail_time_vec = arrayfun(@(a) a.fail_time, cbba_auv); % Inf 表示未失效
    GT_failed = find(isfinite(fail_time_vec));
    GT_alive  = find(~isfinite(fail_time_vec));

    % 将 hb_record 拆成小/大失效时间表
    soft_times = cell(1, num_auvs);
    dead_times = cell(1, num_auvs);
    for r = 1:size(hb_record,1)
        tgt = hb_record(r,1);
        ts  = hb_record(r,2);
        td  = hb_record(r,3);
        if ts > 0, soft_times{tgt}(end+1) = ts; end
        if td > 0, dead_times{tgt}(end+1) = td; end
    end

    perAUV = struct('id',{},'fail_time',{}, ...
        'soft_first',{},'soft_delay',{},'soft_FP',{}, ...
        'dead_first',{},'dead_delay',{},'dead_FP',{},'dead_FN',{});

    soft_FP_total = 0;  dead_FP_total = 0;  dead_FN_total = 0;
    soft_TP_count = 0;  dead_TP_count = 0;

    for j = 1:num_auvs
        ft = fail_time_vec(j);
        ST = sort(soft_times{j});
        DT = sort(dead_times{j});

        rec.id = j;
        rec.fail_time = ft;

        % ---- 小失效 ----
        if isfinite(ft)
            ts_true_idx = find(ST >= ft, 1, 'first');
            if ~isempty(ts_true_idx)
                rec.soft_first = ST(ts_true_idx);
                rec.soft_delay = rec.soft_first - ft;
                soft_TP_count = soft_TP_count + 1;
            else
                rec.soft_first = NaN;
                rec.soft_delay = NaN;
            end
            rec.soft_FP = sum(ST < ft);
        else
            rec.soft_first = NaN;
            rec.soft_delay = NaN;
            rec.soft_FP = numel(ST);
        end
        soft_FP_total = soft_FP_total + rec.soft_FP;

        % ---- 大失效 ----
        if isfinite(ft)
            td_true_idx = find(DT >= ft, 1, 'first');
            if ~isempty(td_true_idx)
                rec.dead_first = DT(td_true_idx);
                rec.dead_delay = rec.dead_first - ft;
                rec.dead_FP = sum(DT < ft);
                rec.dead_FN = 0;
                dead_TP_count = dead_TP_count + 1;
            else
                rec.dead_first = NaN;
                rec.dead_delay = NaN;
                rec.dead_FP = 0;
                rec.dead_FN = 1;
                dead_FN_total = dead_FN_total + 1;
            end
        else
            rec.dead_first = NaN;
            rec.dead_delay = NaN;
            rec.dead_FP = numel(DT);
            rec.dead_FN = 0;
        end
        dead_FP_total = dead_FP_total + rec.dead_FP;

        perAUV(end+1) = rec;
    end

    % ---- 汇总 ----
    N_fail = numel(GT_failed);
    N_alive = numel(GT_alive);
    summary.N_fail  = N_fail;
    summary.N_alive = N_alive;

    summary.SOFT_TP = soft_TP_count;
    summary.SOFT_FP = soft_FP_total;
    summary.SOFT_FP_rate = soft_FP_total / max(1, N_alive);
    soft_delays = [perAUV.soft_delay];
    soft_delays = soft_delays(isfinite(soft_delays));
    summary.SOFT_delay_mean = mean(soft_delays);
    summary.SOFT_delay_p90  = prctile(soft_delays,90);

    summary.DEAD_TP = dead_TP_count;
    summary.DEAD_FP = dead_FP_total;
    summary.DEAD_FP_rate = dead_FP_total / max(1, N_alive);
    summary.DEAD_FN = dead_FN_total;
    summary.DEAD_FN_rate = dead_FN_total / max(1, N_fail);
    dead_delays = [perAUV.dead_delay];
    dead_delays = dead_delays(isfinite(dead_delays));
    summary.DEAD_delay_mean = mean(dead_delays);
    summary.DEAD_delay_p90  = prctile(dead_delays,90);

    % 打印结果
    fprintf('\n==== Failure-detection metrics ====\n');
    fprintf('Real failures: %d, Never-failed: %d\n', N_fail, N_alive);
    fprintf('SOFT  TP=%d, FP=%d, FP_rate=%.3f\n',summary.SOFT_TP, summary.SOFT_FP, summary.SOFT_FP_rate);
    if ~isempty(soft_delays)
        fprintf('SOFT  reaction delay: mean=%.2fs, P90=%.2fs\n', ...
            summary.SOFT_delay_mean, summary.SOFT_delay_p90);
    else
        fprintf('SOFT  reaction delay: (no true positives)\n');
    end
    fprintf('DEAD  TP=%d, FP=%d, FP_rate=%.3f, FN=%d, FN_rate=%.3f\n', ...
        summary.DEAD_TP, summary.DEAD_FP, summary.DEAD_FP_rate, ...
        summary.DEAD_FN, summary.DEAD_FN_rate);
    if ~isempty(dead_delays)
        fprintf('DEAD  reaction delay: mean=%.2fs, P90=%.2fs\n', ...
            summary.DEAD_delay_mean, summary.DEAD_delay_p90);
    else
        fprintf('DEAD  reaction delay: (no true positives)\n');
    end

    %% 写入excel
    file_name = 'analyze.xlsx';
    sheet_name = 'Sheet1';

    % 将结构转换成一行 cell 数组
    data_row = { ...
        seed,...
        num_auvs,...
        summary.N_fail, ...
        summary.N_alive, ...
        summary.SOFT_TP, ...
        summary.SOFT_FP, ...
        summary.SOFT_FP_rate, ...
        summary.SOFT_delay_mean, ...
        summary.SOFT_delay_p90, ...
        summary.DEAD_TP, ...
        summary.DEAD_FP, ...
        summary.DEAD_FP_rate, ...
        summary.DEAD_FN, ...
        summary.DEAD_FN_rate, ...
        summary.DEAD_delay_mean, ...
        summary.DEAD_delay_p90, ...
    };

    % 如果文件不存在，则写入表头
    if ~isfile(file_name)
        headers = { ...
            'Seed', 'auv_num','Real_failures', 'Never_failed', ...
            'SOFT_TP', 'SOFT_FP', 'SOFT_FP_rate', 'SOFT_delay_mean', 'SOFT_delay_P90', ...
            'DEAD_TP', 'DEAD_FP', 'DEAD_FP_rate', 'DEAD_FN', 'DEAD_FN_rate', ...
            'DEAD_delay_mean', 'DEAD_delay_P90' ...
        };
        writecell(headers, file_name, 'Sheet', sheet_name, 'Range', 'A1');
        next_row = 2;
    else
        % 找到下一行
        [~, ~, raw] = xlsread(file_name, sheet_name);
        next_row = size(raw, 1) + 1;
    end

    % 写入数据
    writecell(data_row, file_name, 'Sheet', sheet_name, ...
              'Range', sprintf('A%d', next_row));
end
