
function [A,B] = system_jacobian(SYS, op)
% =====================================================
% TASK 4 – LINEARIZATION (JACOBIAN MATRICES)
% =====================================================
% SYS : output of system_model
% op  : 'hover' or 'climb'
% =====================================================

P = SYS.P;
f = SYS.f;

% -----------------------------------------------------
% Select operating point
% -----------------------------------------------------
switch op
    case 'hover'
        xe = SYS.operating_points.hover.xe;
        ue = SYS.operating_points.hover.ue;
    case 'climb'
        xe = SYS.operating_points.climb.xe;
        ue = SYS.operating_points.climb.ue;
    otherwise
        error('Unknown operating point')
end

nx = length(xe);
nu = length(ue);

eps = 1e-6;

A = zeros(nx,nx);
B = zeros(nx,nu);

% -----------------------------------------------------
% Numerical Jacobian
% -----------------------------------------------------
for i = 1:nx
    dx = zeros(nx,1);
    dx(i) = eps;
    A(:,i) = ( f(xe+dx,ue) - f(xe-dx,ue) )/(2*eps);
end

for j = 1:nu
    du = zeros(nu,1);
    du(j) = eps;
    B(:,j) = ( f(xe,ue+du) - f(xe,ue-du) )/(2*eps);
end

disp('Linearization completed.')
end