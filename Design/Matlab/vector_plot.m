function h = vector_plot(x, reference, state, labels, position)
    h = figure('position',position);
    for i = 1:3
        subplot(3,1,i)
        plot(x, reference(:,i), 'k--', x, state(:,i))
        ylabel(labels{i})
    end
    sgtitle(labels{4})
    legend('Reference','Model')
end