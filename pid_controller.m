function u = pid_controller(id, ref, cur, Ts, Kp, Ki, Kd, reset)
% 多通道 PID 控制器支持，id 表示控制器通道标识
% 输入:
%   ref   - 参考值
%   cur   - 当前值
%   Ts    - 采样时间
%   Kp    - 比例增益
%   Ki    - 积分增益
%   Kd    - 微分增益
%   reset - 可选布尔值，true 表示清空积分和误差状态
% 输出:
%   u     - PID 控制量

    % 使用持久变量字典
    persistent states

    if isempty(states)
        states = struct();
    end

    % 初始化或重置状态
    if nargin == 8 && reset
        states.(id).e_prev = 0;
        states.(id).int_e = 0;
        u = 0;
        return;
    end

    % 如果没有初始化过该 ID
    if ~isfield(states, id)
        states.(id).e_prev = 0;
        states.(id).int_e = 0;
    end

    e = ref - cur;
    % 积分限幅
    if abs(states.(id).int_e) < 20
        states.(id).int_e = states.(id).int_e + e * Ts;
    end
    der_e = (e - states.(id).e_prev) / Ts;

    u = Kp * e + Ki * states.(id).int_e + Kd * der_e;
    states.(id).e_prev = e;
end