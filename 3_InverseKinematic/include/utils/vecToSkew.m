function H = vecToSkew(h)
    
h1 = h(1);
h2 = h(2);
h3 = h(3);

H = [0 -h3 h2;
    h3 0 -h1;
    -h2 h1 0];

end