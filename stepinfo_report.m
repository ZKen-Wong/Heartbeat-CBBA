function stepinfo_report(y, t, target)
    % 去掉 NaN 或 Inf
    valid = isfinite(y) & isfinite(t);
    y = y(valid);
    t = t(valid);

    % 用 stepinfo 正确计算指标
    info = stepinfo(y, t, target, ...
        'SettlingTimeThreshold', 0.02, ...
        'RiseTimeLimits', [0.1 0.9]);

    % 打印结果
    disp('=== 深度控制阶跃响应指标 ===');
    fprintf('上升时间: %.2f 秒\n', info.RiseTime);
    fprintf('超调量:   %.2f %%\n', info.Overshoot);
    fprintf('稳定时间: %.2f 秒\n', info.SettlingTime);
    fprintf('稳态误差: %.2f m\n', abs(info.SettlingMin - target));
end
