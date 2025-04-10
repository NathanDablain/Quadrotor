function h = vector_plot(x, reference, state, labels, y_range)
    h = figure('Name',labels{4},'WindowStyle','Docked');
    for i = 1:3
        subplot(3,1,i)
        plot(x, reference(:,i), x, state(:,i), 'k--','LineWidth',2)
        ylabel(labels{i})
        ylim(y_range)
    end
    sgtitle(labels{4})
    legend('Simulation','MCU')
end