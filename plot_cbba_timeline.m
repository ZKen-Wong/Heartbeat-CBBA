function fpath = plot_cbba_timeline(cbba_record, cbba_auv, seed)
% 绘制 CBBA 任务执行时间轴（分配轮次图例 + 失效灰色遮盖在上层）
% 并保存为 SVG：cbba_timeline_seed_<seed>_auv<N>_fail<M>.svg
%
% cbba_record: struct 数组，字段 {auv, task_id, t_start, t_end, assign_round}
% cbba_auv:    struct 数组，字段 {id, fail_time}（或含 alive）
% seed      :  (可选) 数值/字符串；用于文件名和标题
% outdir    :  (可选) 输出目录，默认 'figs'
%
% 返回:
%   fpath    : 保存的 SVG 完整路径

    outdir = 'pic\plot';

    if nargin < 3 || isempty(seed),   seed   = 'unknown'; end
    if ~isfolder(outdir), mkdir(outdir); end

    if isempty(cbba_record)
        warning('cbba_record is empty.'); fpath = '';
        return;
    end

    % —— 基本信息
    auv_ids   = unique([cbba_record.auv], 'stable');
    n_auvs_rec = numel(auv_ids); %#ok<NASGU>         % 记录里出现的 AUV 数
    max_round = max([cbba_record.assign_round]);
    cmap      = lines(max_round);

    % 统计文件名所需的 N_auv / N_fail
    N_auv = numel(cbba_auv);
    if isfield(cbba_auv,'fail_time')
        ft = [cbba_auv.fail_time];
        N_fail = sum(isfinite(ft) & (ft < inf));
    elseif isfield(cbba_auv,'alive')
        N_fail = sum(~[cbba_auv.alive]);
    else
        N_fail = NaN;  % 无字段则记为 NaN
    end

    % 时间上限：用任务条的最大结束时间
    T_max = max([cbba_record.t_end]);

    figure('Color','w'); hold on; grid on;
    yticks(1:numel(auv_ids));
    yticklabels(arrayfun(@(x) sprintf('g%d',x), auv_ids, 'UniformOutput', false));
    xlabel('Time (s)'); ylabel('AUV');
    title(sprintf('CBBA Task Execution Timeline (seed=%s)', string(seed)));

    % —— 1) 任务条
    for r = 1:numel(cbba_record)
        auv_idx  = find(auv_ids == cbba_record(r).auv);
        t0       = cbba_record(r).t_start;
        t1       = cbba_record(r).t_end;
        round_id = cbba_record(r).assign_round;
        col      = cmap(round_id, :);

        rectangle('Position', [t0, auv_idx-0.4, t1-t0, 0.8], ...
                  'FaceColor', col, 'EdgeColor', 'k', 'LineWidth', 0.8, 'FaceAlpha', 0.8);

        text(mean([t0 t1]), auv_idx, sprintf('T%d', cbba_record(r).task_id), ...
             'HorizontalAlignment','center','VerticalAlignment','middle', ...
             'Color','w','FontWeight','bold');
    end

    % —— 2) 失效灰色遮盖（置于上层）
    for k = 1:numel(cbba_auv)
        ft = cbba_auv(k).fail_time;
        if isfinite(ft) && ft < T_max
            auv_idx = find(auv_ids == cbba_auv(k).id);
            if ~isempty(auv_idx)
                rectangle('Position', [ft, auv_idx-0.4, T_max - ft, 0.8], ...
                          'FaceColor',[0.5 0.5 0.5],'EdgeColor','none','FaceAlpha',0.5);
                text(ft, auv_idx+0.45, 'Fail', 'Color',[0.3 0.3 0.3], 'FontWeight','bold');
            end
        end
    end

    % 轴范围
    xlim([0, T_max*1.02]);
    ylim([0.5, numel(auv_ids)+0.5]);

    % 图例（分配轮次）
    legend_entries = arrayfun(@(x) sprintf('Allocation round %d', x), 1:max_round, 'UniformOutput', false);
    lg_handles = arrayfun(@(x) plot(nan, nan, 's', 'MarkerFaceColor', cmap(x,:), 'MarkerEdgeColor','k'), 1:max_round);
    legend(lg_handles, legend_entries, 'Location', 'bestoutside');

    % —— 保存为 SVG（文件名包含 seed、AUV 数和失效数）
    safe_seed = string(seed);
    set(gcf,'Renderer','painters');  % 矢量渲染
    fpath = fullfile(outdir, sprintf('cbba_timeline_seed_%s_auv%d_fail%d.svg', safe_seed, N_auv, N_fail));
    try
        exportgraphics(gcf, fpath, 'ContentType','vector');
    catch
        print(gcf, fpath, '-dsvg', '-painters');
    end
    fprintf('Saved timeline to %s\n', fpath);
end
