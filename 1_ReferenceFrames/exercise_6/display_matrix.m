function display_matrix(T)
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);

    c = ["r", "g", "b"]
    
    % Estrai la matrice di rotazione 3x3
    R_sub = T(1:3, 1:3); 

    for i = 1:3 % Per ogni asse (X, Y, Z)
        u = R_sub(1, i); % Componente X
        v = R_sub(2, i); % Componente Y
        w = R_sub(3, i); % Componente Z
        
       quiver3(x, y, z, u, v, w,c(i), 'LineWidth', 2);
        hold on;
    end
end