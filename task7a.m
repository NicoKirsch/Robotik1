for i = 1:6
    figure;
    plot(1:50, jointAngles(i,:));
    xlabel('Sample-Nummer');
    ylabel('Gelenkwinkel');
    title(['Gelenkwinkel von Gelenk ', num2str(i)]);
end
