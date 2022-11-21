%% General-Purpose sliding-mode controller for DC/DC converter
clear all; close all; clc;

%% Cuk converter

% System parameters
Ug_cuk = 24;            % [V]
U2_cuk = 15;            % [V]
L1_cuk = 3.3e-3;        % [H]
R1_cuk = 1;             % [Ohm]
C1_cuk = 100e-6;        % [F]
RL_cuk = 10;            % [Ohm] load resistance
Ki_cuk = 1.2;           % [-] Current controller gain
Ku_cuk = 1;             % [-] Voltage controller gain
Ilim_cuk = 1.5;         % [A] switch current limit
fg_cuk = 50e3;          % [Hz] switching frequency
L2_cuk = 2.2e-3;        % [H]
R2_cuk = 0.5;           % [Ohm]
C2_cuk = 47e-6;         % [F]
tau_HPF_Cuk = 0.15e-3;  % [s] time constant high-pass current filter
tau_PI_Cuk = 0.15e-3;   % [s] time constant PI controller

% Variables defined as symbols
syms i1_cuk i2_cuk I1s_cuk I2s_cuk Ui_cuk U1s_cuk U2s_cuk, 'real';

% Steady state currents and voltages:
I2s_cuk = U2_cuk/RL_cuk;
I1s_cuk = I2s_cuk*(U2_cuk/Ug_cuk);
U1s_cuk = Ug_cuk;
U2s_cuk = U2_cuk;

syms i1 i2 u1 u2, 'real';
x = [i1-I1s_cuk, i2-I2s_cuk, u1-U1s_cuk, u2-U2s_cuk]';

% set u1 to steady state values?
u1 = U1s_cuk;

% State-space
A_cuk = [-(R1_cuk/L1_cuk), 0, -(1/L1_cuk), 0;
    0, -(R2_cuk/L2_cuk), 0, -(1/L2_cuk);
    (1/C1_cuk), 0, 0, 0;
    0, (1/C2_cuk), 0, -(1/(RL_cuk*C2_cuk))];

B_cuk = [(Ug_cuk/L1_cuk), (Ug_cuk/L2_cuk), ((-i1_cuk-i2_cuk)/C1_cuk), 0]';      % TODO check currents i1 and i2
G_cuk = [Ug_cuk-R1_cuk*I1s_cuk-U1s_cuk; (-R2_cuk*I2s_cuk-U2s_cuk)/L2_cuk; I1s_cuk/C1_cuk; (I2s_cuk-(U2s_cuk/RL_cuk))/C2_cuk];
K_cuk = [Ki_cuk, 0, 0, Ku_cuk]';

% x vector consists of the differences between the measured and steady
% state values
if false
    % since controller only steers the sliding mode on i1 and u2, dI2_cuk and dU1_cuk are set
    % to 0.
    dI2_cuk = 0;
    dU1_cuk = 0;
    
    % variations for I1 and U2
    dI1vars_cuk = -1.5:0.1:1.5;
    dU2vars_cuk = -15:0.1:5;
    %
    dPhiLow_cuk = zeros(length(dI1vars_cuk),length(dU2vars_cuk));
    dPhiHigh_cuk = dPhiLow_cuk;
    aa = 0;     % Counter
    bb = aa;    % Counter
    for a = dI1vars_cuk
        dI1_cuk = a;    % Iterate for i1
        aa = aa + 1;
        bb = 0;         % Reset
        
        for b = dU2vars_cuk
            dU2_cuk = b;    % iterate for u2
            bb = bb + 1;
            
            % sliding mode matrices
            x_cuk = [dI1_cuk dI2_cuk dU1_cuk dU2_cuk]';
            dPhiLow_cuk(aa,bb) = K_cuk'*A_cuk*x_cuk + K_cuk'*G_cuk;                    % switch open
            dPhiHigh_cuk(aa,bb) = K_cuk'*A_cuk*x_cuk + K_cuk'*B_cuk + K_cuk'*G_cuk;    % switch closed
        end
    end
    
    figure()
    surf(dU2vars_cuk, dI1vars_cuk, dPhiLow_cuk);
    title('dPhiLow Cuk')
    
    figure()
    surf(dU2vars_cuk, dI1vars_cuk, dPhiHigh_cuk);
    title('dPhiHigh Cuk')
end

%% Sepic converter

% System parameters
Ug_sepic = 15;              % [V]
U2_sepic = 20;              % [V]
L1_sepic = 700e-6;          % [H]
R1_sepic = 1;               % [Ohm]
C1_sepic = 6.8e-6;          % [F]
Ki_sepic = 1.1;             % [-]
Ku_sepic = 1;               % [-]
n_sepic = 1.5;              % [-]
Ilim_sepic = 3.5;           % [A]
fg_sepic = 50e3;            % [Hz]
L2_sepic = 380e-6;          % [H]
C2_sepic = 100e-6;          % [F]
RL_sepic = 200;             % [Ohm]
tau_HPF_sepic = 0.5e-3;     % [s]
tau_PI_sepic = 0.5e-3;      % [s]

HPFenable = 1;
w_Amplitude = 0.3;

% State-space
A_sepic = [-(R1_sepic/L1_sepic), 0, -(1/L1_sepic), -(1/(n_sepic*L1_sepic));
    0, 0, 0, -(1/(n_sepic*L2_sepic));
    (1/C1_sepic), 0, 0, 0;
    (1/(n_sepic*C2_sepic)), (1/(n_sepic*C2_sepic)), 0, -(1/(RL_sepic*C2_sepic))];

K_sepic = [Ki_sepic, 0, 0, Ku_sepic]';

% vector with measured states
syms i1_sepic i2_sepic u1_sepic u2_sepic 'real';

% Steady-states:
syms I1s_sepic I2s_sepic U1s_sepic U2s_sepic 'real';

% Setting some things that dont matter to zero
if true
    %     i1_sepic = 0.15;      % steady-state value i1
    %     u2_sepic = 20;        % steady-state value u2
    mI1 = 0:0.25:2.5;       % Vary in this range
    mU2 = 0:2:24;           % Vary in this range
    i2_sepic = 0.1125;      % Computed from winding difference
    u1_sepic = Ug_sepic;    % ~Input voltage
    I2s_sepic = 0.1125;     % Computed from winding difference
    U1s_sepic = Ug_sepic;   % ~Input voltage
    I1s_sepic = 0.15;       % Determined in simulation
    U2s_sepic = U2_sepic;   % Output voltage
end

divFactor_i1 = 1; % e6;
divFactor_u2 = 1; % e3;
scale = 5e-6;

if false
    for i1_sepic = mI1
        for u2_sepic = mU2
            
            % states for sliding function:
            v_sepic = [i1_sepic i2_sepic u1_sepic u2_sepic]';
            Vs_sepic = [I1s_sepic I2s_sepic U1s_sepic U2s_sepic]';
            x_sepic = v_sepic - Vs_sepic;
            
            % B and G matrices for sepic converter
            B_sepic =  [(((u1_sepic+u2_sepic)/n_sepic)/L1_sepic);
                ((u1_sepic+u2_sepic)/n_sepic)/L2_sepic;
                -(i1_sepic+i2_sepic)/C1_sepic;
                -(i1_sepic+i2_sepic)/(n_sepic*C2_sepic)];
            
            G_sepic =  [(-R1_sepic*I1s_sepic+Ug_sepic-(U1s_sepic+U2s_sepic/n_sepic))/L1_sepic;
                -U2s_sepic/(n_sepic*L2_sepic);
                I1s_sepic/C1_sepic;
                ((I1s_sepic+I2s_sepic)/n_sepic-U2s_sepic/RL_sepic)/C2_sepic];
            
            % Sliding function deratives
            dPsidtOff = K_sepic'*A_sepic*x_sepic + K_sepic'*G_sepic;
            dPsidtOn = K_sepic'*A_sepic*x_sepic + K_sepic'*B_sepic + K_sepic'*G_sepic;
            
            % dx/dt functions
            dx_on = A_sepic*x_sepic + B_sepic*1 + G_sepic;
            dx_off = A_sepic*x_sepic + B_sepic*0 + G_sepic;
            %
            dxr_on = [dx_on(1); dx_on(4)];
            dxr_off = [dx_off(1); dx_off(4)];
            %
            figure(99)
            hold on
            quiver(i1_sepic,u2_sepic,dxr_on(1)/divFactor_i1,dxr_on(2)/divFactor_u2,'black','AutoScaleFactor',scale);
            quiver(i1_sepic,u2_sepic,dxr_off(1)/divFactor_i1,dxr_off(2)/divFactor_u2,'red','AutoScaleFactor',scale);
            plot(i1_sepic,u2_sepic,'blue.')
            hold off
            grid on
            xlabel('i1 [A]')
            ylabel('u2 [V]')
            legend('Switch closed','Switch open','Operation point','Location','Best')
            title('Vector field of Sepic converter')
        end
    end
end

if false
    %% Simulations for report
    % Simulate different HPF
    for i = 1:4
        switch i
            case 1
                HPFenable = 0               % disable HPF
            case 2
                HPFenable = 1
                tau_HPF_sepic = 0.25e-3     % 0.5*normal value
            case 3
                HPFenable = 1
                tau_HPF_sepic = 0.75e-3     % 1.5*normal value
            case 4
                HPFenable = 1               % enable HPF
                tau_HPF_sepic = 0.5e-3      % normal value
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(10)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('High pass filter comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % psi f
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('Control out [Psi_f]')
        legend('HPF off','t_H_P_F*0.5','t_H_P_F*1.5','HPF normal')
    end
    
    % Simulate different PI controllers
    for i = 1:4
        switch i
            case 1
                tau_PI_sepic = 0               % disable PI control
            case 2
                tau_PI_sepic = 0.25e-3          % PI time constant * 0.5
            case 3
                tau_PI_sepic = 0.75e-3          % PI time constant * 1.5
            case 4
                tau_PI_sepic = 0.5e-3           % Normal PI time constant
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(11)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('PI controller time constant comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % psi f
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('Control out [Psi_f]')
        legend('PI off','t_P_I*0.5','t_P_I*1.5','PI normal')
    end
    
    % Simulate different w signals
    for i = 1:4
        switch i
            case 1
                w_Amplitude = 0             % disable trigger signal
            case 2
                w_Amplitude = 0.15          % w amplitude * 0.5
            case 3
                w_Amplitude = 0.45          % w amplitude * 1.5
            case 4
                w_Amplitude = 0.3           % Normal w amplitude
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(12)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('w amplitude comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % psi f
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('Control out [Psi_f]')
        legend('w off','w*0.5','w*1.5','w normal')
    end
    
    % Simulate different current limitations
    for i = 1:4
        switch i
            case 1
                Ilim_sepic = 100           % disable current limit
            case 2
                Ilim_sepic = 1.75          % current limit * 0.5
            case 3
                Ilim_sepic = 5.25          % current limit * 1.5
            case 4
                Ilim_sepic = 3.5           % Normal
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(13)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('Current limit comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % switch voltage
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('Control out [Psi_f]')
        legend('I_l_i_m off','I_l_i_m*0.5','I_l_i_m*1.5','I_l_i_m normal')
    end
    
    % Simulate different Ki values
    for i = 1:4
        switch i
            case 1
                Ki_sepic = 0                % disable
            case 2
                Ki_sepic = 1.1*0.5          % * 0.5
            case 3
                Ki_sepic = 1.1*1.5          % * 1.5
            case 4
                Ki_sepic = 1.1              % Normal
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(14)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('K_i comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % psi f
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('Control out [psi_f]')
        legend('K_i off','K_i*0.5','K_i*1.5','K_i normal')
    end
    
    % Simulate different Ku values
    for i = 1:4
        switch i
            case 1
                Ku_sepic = 0            % disable
            case 2
                Ku_sepic = 0.5          % * 0.5
            case 3
                Ku_sepic = 1.5          % * 1.5
            case 4
                Ku_sepic = 1            % Normal
        end
        
        clear out
        out = sim('Sepic.slx')
        
        figure(15)
        subplot 311 % u2
        hold on
        plot(out.u2_scope_sepic.Time,out.u2_scope_sepic.Data)
        hold off
        grid on
        ylabel('u2 [V]')
        title('K_u comparison for Sepic converter')
        subplot 312 % i1
        hold on
        plot(out.i1_scope_sepic.Time,out.i1_scope_sepic.Data)
        hold off
        grid on
        ylabel('i1 [A]')
        subplot 313 % psi f
        hold on
        plot(out.psiF.Time,out.psiF.Data)
        hold off
        grid on
        xlabel('Time [s]')
        ylabel('control out [psi_f]')
        legend('K_u off','K_u*0.5','K_u*1.5','K_u normal')
    end
    
    
end


if false
    %% Determine steady-state switching frequency
    % Determine sample time
    Tsample = out.sw_voltage.Time(end)/length(out.sw_voltage.Time);
    
    % Determine sample where time is @ 0.06 and 0.01 (start and stop of no-load steady-state)
    start_noload = find(out.sw_voltage.Time == 0.007)
    stop_noload = find(out.sw_voltage.Time == 0.008)
    
    figure()
    plot(out.sw_voltage.Time(start_noload:stop_noload), out.sw_voltage.Data(start_noload:stop_noload))
    xlabel('Time [s]')
    ylabel('Switch voltage [V]')
    title('Steady-state switch voltage setpoint during no load')
    
    X_noload = fft(out.sw_voltage.Data(start_noload:stop_noload));
    
    % Plot FFT of no load frequency
    freq_noload = 1:length(X_noload);
    freq_noload = freq_noload*(1/Tsample);
    figure()
    plot(freq_noload,db(X_noload))
    xlabel('Frequency [Hz]')
    ylabel('Signal strength [dB]')
    grid on
    
    % Determine sample where time is @ 0.012 and 0.0145 (start and stop of no-load steady-state)
    start_load = find(out.sw_voltage.Time == 0.014)
    stop_load = find(out.sw_voltage.Time == 0.0145)
    
    figure()
    plot(out.sw_voltage.Time(start_load:stop_load), out.sw_voltage.Data(start_load:stop_load))
    xlabel('Time [s]')
    ylabel('Switch voltage [V]')
    title('Steady-state switch voltage setpoint during load')
    
    X_load = fft(out.sw_voltage.Data(start_load:stop_load));
    
    % Plot FFT of load frequency
    freq_load = 1:length(X_load);
    freq_load = freq_load*(1/Tsample);
    figure()
    plot(freq_load,db(X_load))
    xlabel('Frequency [Hz]')
    ylabel('Signal strength [dB]')
    title('FFT of steady-state switch voltage during load step')
    grid on
    
    figure()
    subplot 211
    plot(out.sw_freq_counter.Time(start_noload:stop_noload), out.sw_freq_counter.Data(start_noload:stop_noload))
    xlabel('Time [s]')
    ylabel('Counter value [-]')
    title('Counter during no-load steady-state')
    grid on
    xlim([0.007 0.0075])
    
    subplot 212
    plot(out.sw_freq_counter.Time(start_load:stop_load), out.sw_freq_counter.Data(start_load:stop_load))
    xlabel('Time [s]')
    ylabel('Counter value [-]')
    title('Counter during load steady-state')
    grid on
    xlim([0.014 0.0145])
    
end

