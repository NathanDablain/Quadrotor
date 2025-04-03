function h = reference_plot(x, Sim, MCU, Reference, labels, position, y_range)
    h = figure('position',position);
    for i = 1:3
        subplot(3,1,i)
        plot(x, Sim(:,i), x, MCU(:,i), 'k--', x, Reference(:,i), 'r:','LineWidth',2)
        ylabel(labels{i})
        ylim(y_range)
    end
    sgtitle(labels{4})
    legend('Simulation','MCU','Reference')
end