clear; clc;
SYS= system_model(true);
disp(fieldnames(SYS))
[A,B] = system_jacobian(SYS,'hover') ;
size(A)
size(B)
disp(A)
disp(B)
linearization_check(true);
task6_7_stability_controllability(true);
controller_design(A,B);
observer_design(A,B);
combined_simulation(A,B);