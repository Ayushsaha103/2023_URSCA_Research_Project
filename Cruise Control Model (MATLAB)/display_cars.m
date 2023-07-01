function display_cars(cars)
    % display cars
    for i = 1:numel(cars)
        cars(i).boxplot();
        cars(i).show("r");
        hold on;
    end
    hold off;
    axis([-100 100 -100 100]);
end

