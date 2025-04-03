function h = vector_plot(x, reference, state, labels, position, y_range)
    h = figure('position',position);
    for i = 1:3
        subplot(3,1,i)
        plot(x, reference(:,i), x, state(:,i), 'k--','LineWidth',2)
        ylabel(labels{i})
        ylim(y_range)
    end
    sgtitle(labels{4})
    legend('Simulation','MCU')
end