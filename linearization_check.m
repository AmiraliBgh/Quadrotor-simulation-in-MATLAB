function linearization_check(run_recent_tasks)
% =====================================================
% TASK 5 – NONLINEAR vs LINEARIZED (MULTIPLE INPUTS)
% =====================================================

clc;
if nargin==0
    run_recent_tasks = false ;
end
%% -----------------------------------------------------
% Load system (NO task 1–4 plots)
% -----------------------------------------------------
SYS = system_model(run_recent_tasks);
[A,B] = system_jacobian(SYS,'hover');

P = SYS.P;
f = SYS.f;

%% -----------------------------------------------------
% Simulation settings
% -----------------------------------------------------
tspan = [0 5];
dt = 0.01;
t = tspan(1):dt:tspan(2);

nx = 12;
nu = 4;

%% -----------------------------------------------------
% Initial condition (non-zero)
% -----------------------------------------------------
x0 = zeros(nx,1);
x0(3) = 0.2;
x0(4) = 5*pi/180;
x0(7) = 0.1;

%% -----------------------------------------------------
% Equilibrium
% -----------------------------------------------------
xe = SYS.operating_points.hover.xe;
ue = SYS.operating_points.hover.ue;

%% -----------------------------------------------------
% Multiple step inputs (different magnitudes)
% -----------------------------------------------------
input_steps = [ ...
    0.05;
    0.10;
    0.20];

states_to_plot = [3 4 7 9];
labels = {'z (m)','\phi (rad)','v_x (m/s)','v_z (m/s)'};

%% -----------------------------------------------------
% Loop over different inputs
% -----------------------------------------------------
for k = 1:length(input_steps)

    du = [input_steps(k)*P.m*P.g; 0; 0; 0];
    u  = ue + du;

    %% Nonlinear simulation
    [t_nl, x_nl] = ode45(@(t,x) f(x,u), tspan, x0);

    %% Linearized simulation
    x_tilde0 = x0 - xe;
    u_tilde  = u  - ue;

    sys_lin = ss(A,B,eye(nx),zeros(nx,nu));
    [y_lin, ~] = lsim(sys_lin, ...
                      repmat(u_tilde',length(t),1), ...
                      t, x_tilde0);

    x_lin = y_lin + xe';

    %% Interpolation
    x_nl_i = interp1(t_nl,x_nl,t);

    %% Errors
    abs_err = abs(x_nl_i - x_lin);
    rms_err = sqrt(mean(abs_err.^2));

    %% Plot
    figure('Name',sprintf('Step Input = %.0f%% mg',input_steps(k)*100))
    for i = 1:4
        idx = states_to_plot(i);
        subplot(4,1,i)
        plot(t,x_nl_i(:,idx),'k','LineWidth',1.5); hold on
        plot(t,x_lin(:,idx),'r--','LineWidth',1.5)
        plot(t,abs_err(:,idx),'b:')
        ylabel(labels{i})
        grid on
        if i==1
            title(['Nonlinear vs Linearized – \Delta u = ' ...
                   num2str(input_steps(k)*100) '% mg'])
            legend('Nonlinear','Linearized','Abs Error')
        end
    end
    xlabel('Time (s)')

    %% Display metrics
    disp(['=== INPUT STEP = ' num2str(input_steps(k)*100) '% mg ==='])
    disp('RMS error per state:')
    disp(rms_err')

end

end