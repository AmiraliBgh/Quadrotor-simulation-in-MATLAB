function observer_design(A,B)

disp('--- Task 11 & 12: Observer Design and Simulation ---')

%% ===============================
% Dimensions
% ===============================

n = size(A,1);
C = eye(n);
p = size(C,1);

%% ===============================
% Task 11 – Observer design
% ===============================
% Desired observer poles (faster than controller poles)
p_obs = -5 * ones(n,1);

% Observer gain
L = place(A', C', p_obs)';
disp('Observer gain L computed.')

% Verify eigenvalues
eig_obs = eig(A - L*C);
disp('Eigenvalues of (A-LC):')
disp(eig_obs.')

%% ===============================
% Task 12 – Observer simulation
% ===============================
t = 0:0.01:8;

% Input (same for system & observer)
u = ones(size(t));

% Initial conditions (non-zero & different)
x0  = zeros(n,1);
x0(3) = 0.5;

xhat0 = zeros(n,1);
xhat0(3) = -0.5;

%% ===============================
% Augmented system (x & x_hat)
% ===============================
A_aug = [ A           zeros(n)
          L*C   (A - L*C) ];

B_aug = [ B
          B ];

C_aug = [ eye(n)   zeros(n)
          zeros(n) eye(n) ];

sys_aug = ss(A_aug, B_aug(:,1), C_aug, 0);

[x_all,~,~] = lsim(sys_aug, u, t, [x0; xhat0]);

x     = x_all(:,1:n);
x_hat = x_all(:,n+1:end);
e     = x - x_hat;

%% ===============================
% Plots
% ===============================
for i = 1:min(4,n)
    figure
    plot(t, x(:,i),'LineWidth',2); hold on
    plot(t, x_hat(:,i),'--','LineWidth',2)
    plot(t, e(:,i),':','LineWidth',2)
    grid on
    xlabel('Time (s)')
    ylabel(['State x_' num2str(i)])
    legend('True','Estimated','Error')
    title(['Observer Performance for State x_' num2str(i)])
end

disp('Observer simulation completed.')

end