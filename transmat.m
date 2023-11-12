function [T10,T20,T30,T40,T50,T60,T70,T80,T90]=transmat(theta1,theta2,theta3,theta4,theta5,theta6)
   
   % Rozmery ramen manipulatoru
   a1 = 12.5;
   a2 = 15;
   a3 = 45;
   a4 = 40;
   a5 = 10; 
   a6 = 7.5;
   
   theta4=theta4-pi;          % Uprava pro UR5e
  
   
   cos1 = cos(theta1);
   sin1 = sin(theta1);
   cos2 = cos(theta2);
   sin2 = sin(theta2);
   cos3 = cos(theta3);
   sin3 = sin(theta3);
   cos4 = cos(theta4);
   sin4 = sin(theta4);
   cos5 = cos(theta5);
   sin5 = sin(theta5);
   cos6 = cos(theta6);
   sin6 = sin(theta6);
   
   % Vypocet tranformacnich matic
   T10 = [cos1 -sin1     0     0;
           sin1  cos1    0     0;
           0     0     1     a1;
           0     0     0     1];
       
   %posun       
   T21 = [1  0  0           -a2;
           0  1  0           0;
           0     0     1     0;
           0     0     0     1];
      
   T32 = [1     0     0         0;
          0     cos2  -sin2     a3*cos2;
          0     sin2  cos2      a3*sin2;
          0     0     0         1];
      
   %posun   
   T43 = [1  0  0           a2;
           0  1  0           0;
           0     0     1     0;
           0     0     0     1];  
 
   T54 = [1     0     0         0;
          0     cos3  -sin3     a4*cos3;
          0     sin3  cos3      a4*sin3;
          0     0     0         1];
   
   %posun
   T65 = [ 1     0     0      -a5;
           0    cos4  -sin4    0;
           0    sin4   cos4    0;
           0     0     0       1];    
       
   
   T76 = [cos5   -sin5    0         0;
          sin5    cos5    0         0;
          0     0         1         a5;
          0     0         0         1];
      
   T87 = [1     0     0            -a6;
          0     cos6  -sin6        0;
          0     sin6     cos6      0;
          0     0     0      1];
      
   T98  = [1     0     0         0;
           0     cos6  -sin6     0;
           0     sin6  cos6      0;
           0     0     0         1];
      
  % Vztazeni transformacnich matic k zakladne     
  T20 = T10 * T21;
  T30 = T20 * T32;
  T40 = T30 * T43;
  T50 = T40 * T54;
  T60 = T50 * T65;
  T70 = T60 * T76;
  T80 = T70 * T87;
  T90 = T80 * T98;

end