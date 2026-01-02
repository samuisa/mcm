function plotVelocities(t_hist, x_dot_tool_hist, x_dot_ee_hist)
    
    figure
    hold on;
    title('TOOL VELOCITIES')
    plot(t_hist, x_dot_tool_hist)
    legend('omega x', 'omega y', 'omega z', 'xdot', 'ydot', 'zdot')
    hold off;

    figure
    hold on;
    title('END-EFFECTOR VELOCITIES')
    plot(t_hist, x_dot_ee_hist)
    legend('omega x', 'omega y', 'omega z', 'xdot', 'ydot', 'zdot')
    hold off;

end