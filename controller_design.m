
function controller_design(A,B)

disp('--- Task 8: State Feedback Design ---')

n = size(A,1);

%% ===============================
% METHOD 1: Pole Placement
% ===============================

zeta = 0.7;
Ts = 2;
wn = 4/(zeta*Ts);

% dominant complex poles
p1 = -zeta*wn + 1j*wn*sqrt(1-zeta^2);
p2 = conj(p1);

% additional real poles (distinct!)
p_real = linspace(-4*wn, -8*wn, n-2);

p_des = [p1 p2 p_real];

% Pole placement
K_pp = place(A,B,p_des);
eig_pp = eig(A-B*K_pp);

%% ===============================
% METHOD 2: LQR
% ===============================

Q = eye(n);
R = eye(size(B,2));

[K_lqr,~,eig_lqr] = lqr(A,B,Q,R);

%% ===============================
% Comparison plot
% ===============================

figure
plot(real(eig_pp),imag(eig_pp),'rx','LineWidth',2)
hold on
plot(real(eig_lqr),imag(eig_lqr),'bo','LineWidth',2)
grid on
legend('Pole Placement','LQR')
title('Task 8: Closed-loop Pole Comparison')
xlabel('Real Axis')
ylabel('Imaginary Axis')


disp('--- Task 9: Closed-Loop Linear Simulation ---')

%% ===============================
% Closed-loop system
% ===============================
Acl = A - B*K_pp;
n   = size(A,1);
C   = eye(n);

%% ===============================
% Reference tracking (prefilter)
% ===============================
% assume first output is z-position
Cr = C(1,:);
N  = -1/(Cr*(Acl\B(:,1)));

%% ===============================
% Simulation setup
% ===============================
t = 0:0.01:10;

% non-zero initial condition
x0 = zeros(n,1);
x0(3) = 0.5;   % initial altitude error
x0(4) = 5*pi/180; % initial roll angle

% reference (step)
r = ones(size(t));

%% ===============================
% Closed-loop dynamics
% ===============================
sys_cl = ss(Acl, B(:,1)*N, C(1,:), 0);

[y,t,x] = lsim(sys_cl, r, t, x0);

%% ===============================
% Performance metrics
% ===============================
info = stepinfo(y,t);

settling_time = info.SettlingTime;
overshoot     = info.Overshoot;
rise_time     = info.RiseTime;
ss_error      = abs(r(end)-y(end));

%% ===============================
% Plots
% ===============================
figure
plot(t,y,'LineWidth',2)
hold on
plot(t,r,'--','LineWidth',1.5)
grid on
xlabel('Time (s)')
ylabel('Output')
title('Task 9: Closed-loop Response')
legend('y(t)','reference')

%% ===============================
% Display results
% ===============================
disp('Performance metrics:')
fprintf('Settling time: %.2f s\n',settling_time)
fprintf('Overshoot: %.2f %%\n',overshoot)
fprintf('Rise time: %.2f s\n',rise_time)
fprintf('Steady-state error: %.4f\n',ss_error)

end