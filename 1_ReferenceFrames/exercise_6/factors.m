function T_homog = factors(R, v) % Rinominato T in v per chiarezza
    % Costruisce la matrice di trasformazione omogenea 4x4
    T_homog = [R, v; 0 0 0 1];
end