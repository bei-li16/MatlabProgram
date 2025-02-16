% BLDC 反电势检测换相控制
clc; clear; close all;

% 设定PWM参数
Fs = 200e3;         % 采样频率 200 kHz
frequency = 20e3;  % PWM频率 20 kHz
duty = 0.6;        % 占空比 60%
duration = 0.1;    % 信号时长 0.1秒

% 时间向量
Ts = 1/Fs;
% Tstep = 0.0000001;
t = 0:Ts:duration;
% t = 0:Tstep:duration;

% 生成方波PWM信号（修正后）
T = 1/frequency;                % 周期 = 50 μs
high_time = T * duty;           % 高电平时间 = 30 μs
pwm_signal = double(mod(t, T) < high_time);

% 模拟反电势信号
bemf_freq = 50; % 反电势频率 50 Hz
bemf_signal = sin(2*pi*bemf_freq*t);
zero_crossing = diff(sign(bemf_signal)); % 过零点检测

% 绘图
subplot(2,1,1);
plot(t, pwm_signal, 'r'); hold on;
plot(t, bemf_signal, 'b');
title('PWM 信号与反电势');
legend('PWM', 'Back-EMF');
xlim([0 0.02]); % 显示前20ms波形

subplot(2,1,2);
plot(t(1:end-1), zero_crossing, 'k');
title('反电势过零检测');
xlabel('时间 (s)');
ylabel('检测信号');