v_test = rand(1e6, 1);
z_test = rand(1e6, 1);

tic;
for k = 1:1e6
    % LuGre math here
end
t_lugre = toc;

tic;
for k = 1:1e6
    % Surrogate model 1 math here
end
t_surrogate = toc;

improvement = (t-lugre - t_surrogtate) / t_lugre * 100;
fprintf('Surrogate 1 is %.2f%% faster than LuGre\n', improvement);