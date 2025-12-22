figure
R0 = [1 0 0; 0 1 0; 0 0 1];
v0 = [0; 0; 0];
T0_0 = factors(R0, v0);
display_matrix(T0_0), hold on, axis equal;

%

R1 = [1 0 0; 
      0 1 0; 
      0 0 1];
v1 = [0;
      0;
      1.75];
T0_1 = factors(R1, v1);
display_matrix(T0_0 * T0_1);

%

R2 = [-1 0 0; 
       0 0 1; 
       0 1 0];
v2 = [0; 
      0;
      0.98];
T1_2 = factors(R2, v2);
display_matrix(T0_0 * T0_1 * T1_2);

%

R3 = [0 0 1;
      0 1 0; 
      -1 0 0];
v3 = [1.05;
      0; 
      0];
T2_3 = factors(R3, v3);
display_matrix(T0_0 * T0_1 * T1_2 * T2_3);

%

R4 = [0 0 -1;
      0 -1 0;
      -1 0 0];
v4 = [0;
      1.4550;
      3.2650];
T3_4 = factors(R4, v4);
display_matrix(T0_0 * T0_1 * T1_2 * T2_3 * T3_4);

%

R5 = [0 0 1;
      -1 0 0;
      0 -1 0];
v5 = [0.35;
      0;
      0];
T4_5 = factors(R5, v5);
display_matrix(T0_0 * T0_1 * T1_2 * T2_3 * T3_4 * T4_5);

%

R6 = [0 1 0;
      0 0 1;
      1 0 0];
v6 = [0;
      0;
      3.85];
T5_6 = factors(R6, v6);
display_matrix(T0_0 * T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6);

%

R7 = [0 0 1;
      1 0 0;
      0 1 0];
v7 = [1.53;
      0;
      0];
T6_7 = factors(R7, v7);
display_matrix(T0_0 * T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7);

%

R8 = R7';
v8 = -R8*v7;
T7_6 = factors(R8, v8);

%

T6_6 = factors(eye(3), [0; 0; 0]);