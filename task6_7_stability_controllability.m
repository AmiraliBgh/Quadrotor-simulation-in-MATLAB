function task6_7_stability_controllability(run_recent_tasks)
% =====================================================
% TASK 6 & 7 – OPEN-LOOP STABILITY + CONTROLLABILITY
% =====================================================

clc;
if nargin==0
    run_recent_tasks = false ;
end
%% Load system (NO plots from previous tasks)
SYS = system_model(run_recent_tasks);
[A,B] = system_jacobian(SYS,'hover');

n = size(A,1);

%% =====================================================
% TASK 6 – OPEN LOOP STABILITY ANALYSIS
% =====================================================
disp('==============================')
disp(' TASK 6: OPEN-LOOP STABILITY ')
disp('==============================')

%% Eigenvalue analysis
lambda = eig(A);

disp('Eigenvalues of A:')
disp(lambda.')

%% Stability classification
if all(real(lambda) < 0)
    stability = 'Asymptotically Stable';
elseif any(real(lambda) > 0)
    stability = 'Unstable';
else
    stability = 'Marginally Stable';
end

disp(['Stability classification: ' stability])

%% Time constants
tau = -1./real(lambda);
disp('Time constants tau_i = -1/Re(lambda_i):')
disp(tau.')

%% Dominant pole
[~,idx] = max(real(lambda));
dominant_pole = lambda(idx);

disp('Dominant pole:')
disp(dominant_pole)

%% Pole plot
figure;
plot(real(lambda),imag(lambda),'rx','MarkerSize',10,'LineWidth',2)
xlabel('Real Axis'); ylabel('Imag Axis')
title('Task 6: Open-loop Poles')
grid on

%% Open-loop step response (thrust input only)
sys_ol = ss(A,B,eye(n),zeros(n,4));

t = 0:0.01:5;
u = zeros(length(t),4);
u(:,1) = 0.1;   % small thrust step

figure;
lsim(sys_ol,u,t)
title('Task 6: Open-loop Step Response')
xlabel('Time (s)')
grid on

%% =====================================================
% TASK 7 – CONTROLLABILITY & OBSERVABILITY
% =====================================================
disp('==============================')
disp(' TASK 7: CONTROLLABILITY & OBSERVABILITY ')
disp('==============================')

%% Controllability
Cmat = ctrb(A,B);
rankC = rank(Cmat);

disp(['Rank of Controllability Matrix: ' num2str(rankC)])
disp('Controllabiliy matrix is:')
disp(Cmat)
if rankC == n
    disp('System is FULLY CONTROLLABLE')
else
    disp('System is NOT fully controllable')
end

%% Observability
C = eye(n);   % full-state measurement
Omat = obsv(A,C);
rankO = rank(Omat);

disp(['Rank of Observability Matrix: ' num2str(rankO)])
disp('Observability matrix is:')
disp(Omat)
if rankO == n
    disp('System is FULLY OBSERVABLE')
else
    disp('System is NOT fully observable')
end

%% =====================================================
% INTERPRETATION (for report)
% =====================================================
disp('--- Interpretation ---')
disp('Open-loop quadrotor is generally unstable or marginally stable.')
disp('Full controllability implies all states can be driven by inputs.')
disp('Full observability implies all states can be reconstructed from outputs.')
disp('These properties justify state-feedback and observer design in next tasks.')

end