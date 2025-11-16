function plot_disturbance(t, result, u, scenario)
%  PLOT_ALL  Plot key simulation outputs in a 2×2 grid.
% ACKNOWLDEGEMENT: DRAFTED BY CHATGPT to save me a minute
    if nargin < 4
        scenario = "";
    end

    figure;

    % --- T_f ---
    subplot(2,2,1);
    plot(t, result(1,:), 'LineWidth', 1.2);
    title('T_f');
    grid on;
    %yticks([50 90 130 170]);
    %%%%xticks([600 1200 200]);
    xlim([0 100]);

    % --- T_c ---
    subplot(2,2,2);
    plot(t, result(2,:), 'LineWidth', 1.2);
    hold on;
    % horizontal lines
    %yline(90,  '--k', '90',  'LabelVerticalAlignment','bottom');
    %yline(110, '--k', '110', 'LabelVerticalAlignment','bottom');
    %yline(130, '--k', '130', 'LabelVerticalAlignment','bottom');
    % vertical lines
    %xline(600,  '--k', '600',  'LabelHorizontalAlignment','right');
    %xline(1200, '--k', '1200', 'LabelHorizontalAlignment','right');
    %xline(200, '--k', '200', 'LabelHorizontalAlignment','right');
    hold off;
    title('T_c');
    grid on;
    %%yticks([50 90 130]);
    %%%%xticks([600 1200 200]);
    xlim([0 100]);

    % --- x_I ---
    subplot(2,2,3);
    plot(t, result(7,:), 'LineWidth', 1.2);
    title('x_i');
    grid on;
    %%%%xticks([600 1200 200]);
    xlim([0 100]);

    % --- Control inputs ---
    subplot(2,2,4);
    plot(t, u(1,:), 'LineWidth', 1.2); hold on;
    plot(t, u(2,:), 'LineWidth', 1.2);
    title('u_p and u_f');
    legend('u_p','u_f');
    grid on;
    %%%%xticks([600 1200 200]);
    xlim([0 100]);

    % --- Main title ---
    sgtitle(scenario, 'FontWeight', 'bold');

end
