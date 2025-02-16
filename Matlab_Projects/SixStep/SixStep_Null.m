%% 无传感器的六步换向控制（完整代码）
clc; clear; close all;

% 电机参数
PolePairs = 4;       % 极对数
Vdc = 24;            % 直流母线电压 (V)
R = 0.1;             % 相电阻 (Ω)
L = 0.001;           % 相电感 (H)
Ke = 0.01;           % 反电势系数 (V/(rad/s))
J = 0.001;           % 转动惯量 (kg·m²)
B = 0.001;           % 阻尼系数 (N·m·s/rad)
TL = 0;              % 负载转矩 (N·m)

% 仿真参数
Ts = 1e-6;           % 仿真步长 (s)
Tfinal = 0.1;        % 总仿真时间 (s)
t = 0:Ts:Tfinal;

% 初始化变量
theta_e = zeros(size(t));    % 电角度 (rad)
omega_e = zeros(size(t));    % 电角速度 (rad/s)
Ia = zeros(size(t));         % 相电流 (A)
Ib = zeros(size(t));
Ic = zeros(size(t));
Te = zeros(size(t));         % 电磁转矩 (N·m)
BEMF_A = zeros(size(t));     % 反电势 (V)
BEMF_B = zeros(size(t));
BEMF_C = zeros(size(t));
CommutationStep = zeros(size(t)); % 换向步骤 (0-5)
ZCD_Detected = zeros(size(t));    % 过零检测标志

% 反电势模型 (理想梯形波)
BEMF_Amplitude = Ke * 100; % 假设转速 100 rad/s
for k = 1:length(t)
    theta_e(k) = 2*pi*PolePairs*100*t(k); % 电角度
    phase = mod(theta_e(k), 2*pi);
    
    % 生成梯形反电势（完整6步）
    if phase < pi/3
        BEMF_A(k) = BEMF_Amplitude;
        BEMF_B(k) = -BEMF_Amplitude;
        BEMF_C(k) = 0;
    elseif phase < 2*pi/3
        BEMF_A(k) = BEMF_Amplitude;
        BEMF_B(k) = 0;
        BEMF_C(k) = -BEMF_Amplitude;
    elseif phase < pi
        BEMF_A(k) = 0;
        BEMF_B(k) = BEMF_Amplitude;
        BEMF_C(k) = -BEMF_Amplitude;
    elseif phase < 4*pi/3
        BEMF_A(k) = -BEMF_Amplitude;
        BEMF_B(k) = BEMF_Amplitude;
        BEMF_C(k) = 0;
    elseif phase < 5*pi/3
        BEMF_A(k) = -BEMF_Amplitude;
        BEMF_B(k) = 0;
        BEMF_C(k) = BEMF_Amplitude;
    else
        BEMF_A(k) = 0;
        BEMF_B(k) = -BEMF_Amplitude;
        BEMF_C(k) = BEMF_Amplitude;
    end
end

% 过零检测与换向逻辑
ZCD_Threshold = 0.1;         % 过零检测阈值 (V)
CommutationDelay = 30e-6;    % 换向延迟补偿 (30 μs)

for k = 2:length(t)
    % 检测B相反电势过零点
    if (BEMF_B(k-1) < ZCD_Threshold) && (BEMF_B(k) >= ZCD_Threshold)
        [~, idx] = min(abs(t - (t(k) + CommutationDelay)));
        if idx <= length(CommutationStep)
            CommutationStep(idx:end) = mod(CommutationStep(idx), 6) + 1;
            ZCD_Detected(k) = 1;
        end
    end
    
    % 根据换向步骤选择导通相
    State = CommutationStep(k);
    switch State
        case 1 % A+ B- C off
            Vab = Vdc;
            Vbc = 0;
        case 2 % A+ C- B off
            Vab = 0;
            Vbc = Vdc;
        case 3 % B+ C- A off
            Vab = -Vdc;
            Vbc = Vdc;
        case 4 % B+ A- C off
            Vab = -Vdc;
            Vbc = 0;
        case 5 % C+ A- B off
            Vab = 0;
            Vbc = -Vdc;
        case 6 % C+ B- A off
            Vab = Vdc;
            Vbc = -Vdc;
        otherwise
            Vab = 0;
            Vbc = 0;
    end
    
    % 电机动态方程
    dIa = (Vab - R*Ia(k-1) - BEMF_A(k)) / L * Ts;
    dIb = (Vbc - R*Ib(k-1) - BEMF_B(k)) / L * Ts;
    dIc = (-Vab - Vbc - R*Ic(k-1) - BEMF_C(k)) / L * Ts;
    
    Ia(k) = Ia(k-1) + dIa;
    Ib(k) = Ib(k-1) + dIb;
    Ic(k) = Ic(k-1) + dIc;
    
    % 计算电磁转矩
    Te(k) = (Ke * (Ia(k) + Ib(k) + Ic(k))) / PolePairs;
    
    % 更新转速和角度
    omega_e(k) = omega_e(k-1) + (Te(k) - B*omega_e(k-1) - TL)/J * Ts;
    theta_e(k) = theta_e(k-1) + omega_e(k) * Ts;
end

% 绘图
figure;
subplot(3,1,1);
plot(t, BEMF_B, 'b'); title('B相反电势 (V)');
subplot(3,1,2);
stairs(t, CommutationStep, 'r'); title('换向步骤');
subplot(3,1,3);
plot(t, Ia, 'g', t, Ib, 'b', t, Ic, 'r'); 
title('相电流 (A)'); legend('Ia', 'Ib', 'Ic');