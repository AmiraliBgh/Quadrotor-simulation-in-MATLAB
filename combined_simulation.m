function combined_simulation(A,B)
% =====================================================
% TASK 14 – STATE FEEDBACK + OBSERVER
% TASK 16 – INTEGRAL CONTROL
% TASK 17 – OBSERVER + INTEGRAL CONTROLLER
% =====================================================



%% -----------------------------------------------------
% Dimensions
% -----------------------------------------------------
n = size(A,1);
m = size(B,2);

%% -----------------------------------------------------
% Output definitions
% -----------------------------------------------------
% Full-state measurement (for observer)
C = eye(n);

% Single output for integral action (altitude z)
C_int = zeros(1,n);
C_int(1,3) = 1;

%% -----------------------------------------------------
% Time and reference
% -----------------------------------------------------
t = 0:0.01:10;
N = length(t);
r = ones(N,1);      % step reference

%% =====================================================
% TASK 14 – STATE FEEDBACK + OBSERVER
% =====================================================

disp('=== Task 14: State feedback + Observer ===')

% Controllability check
assert(rank(ctrb(A,B)) == n , 'System not controllable');

% Controller poles
p_ctrl = linspace(-1,-3,n);
K = place(A,B,p_ctrl);      % K: m × n

% Observer poles
p_obs = p_ctrl - 2;

try
    L = place(A',C',p_obs)';
catch
    L = acker(A',C',p_obs)';
end

% Closed-loop systems
sys_x    = ss(A-B*K, B, C, zeros(n,m));
sys_xhat = ss(A-L*C, B, C, zeros(n,m));

% Input
u = repmat(r,1,m);

x0    = zeros(n,1);
xhat0 = zeros(n,1);

[x,t]    = lsim(sys_x,u,t,x0);
[xhat,~] = lsim(sys_xhat,u,t,xhat0);

e = x - xhat;

figure
subplot(3,1,1), plot(t,x(:,3)),     title('x (z)'), grid on
subplot(3,1,2), plot(t,xhat(:,3)),  title('x̂ (z)'), grid on
subplot(3,1,3), plot(t,e(:,3)),     title('Estimation error'), grid on

%% =====================================================
% TASK 16 – INTEGRAL CONTROL
% =====================================================

disp('=== Task 16: Integral control ===')

A_aug = [ A          zeros(n,1);
         -C_int      0        ];

B_aug = [ B;
          zeros(1,m) ];

% Augmented controllability
assert(rank(ctrb(A_aug,B_aug)) == n+1 , ...
       'Augmented system not controllable');

% Augmented poles
p_aug = [p_ctrl -4];

K_aug = place(A_aug,B_aug,p_aug);   % size: m × (n+1)

Kx = K_aug(:,1:n);    % m × n   ✅
Ki = K_aug(:,end);    % m × 1   ✅

%% =====================================================
% TASK 17 – OBSERVER + INTEGRAL CONTROLLER
% =====================================================

disp('=== Task 17: Observer + Integral Controller ===')

A_cl = [
    A              -B*Kx            -B*Ki;
    L*C       (A-B*Kx-L*C)           -B*Ki;
   -C_int          zeros(1,n)         0
];

B_cl = [ zeros(2*n,1);
         1 ];

sys_cl = ss(A_cl,B_cl,eye(2*n+1),0);

X0 = zeros(2*n+1,1);

[X,t] = lsim(sys_cl,r,t,X0);

% Extract states
x    = X(:,1:n);
xhat = X(:,n+1:2*n);
xi   = X(:,end);

u = -xhat*Kx' - Ki*xi;

figure
subplot(4,1,1), plot(t,x(:,3)),    title('z'), grid on
subplot(4,1,2), plot(t,xhat(:,3)), title('ẑ'), grid on
subplot(4,1,3), plot(t,xi),        title('Integral state \xi'), grid on
subplot(4,1,4), plot(t,u),         title('Control input u'), grid on

disp('✅ Tasks 14, 16, 17 completed successfully')
end