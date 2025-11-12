% Generate every figure used in the LaTeX write-up.
clear; clc; close all

fprintf('Running 2.7.1 step_validation...\n');  step_validation
fprintf('Running 2.7.2 integral_step...\n');    integral_step
fprintf('Running 2.7.3 door_open_test...\n');   door_open_test
fprintf('Running 2.7.4 observer_demo...\n');    observer_demo
fprintf('Running 2.7.5 profile_tracking...\n'); profile_tracking
fprintf('Running 2.7.6 robustness_mc...\n');    robustness_mc
fprintf('Running 2.7.7 demo_brunovsky...\n');   demo_brunovsky

fprintf('\nAll done. PDFs written to:\n  fig_step_validation.pdf\n  fig_integral_step.pdf\n  fig_door_open.pdf\n  fig_observer.pdf\n  fig_profile_tracking.pdf\n  fig_robustness.pdf\n  fig_brunovsky_demo.pdf\n');
