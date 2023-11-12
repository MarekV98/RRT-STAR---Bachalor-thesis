
function [] = cesta(trasaA,trasaB,trasaC,trasaD,trasaE,trasaF)  

% Slouzi k vykreslovani pohybu manipulatoru 

[obstacles,n]=obs();
figure(1);
for i=1:1:n
obstacle=obstacles(:,:,i); 
hold on
face = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';

X = obstacle(:,1);
Y = obstacle(:,2);
Z = obstacle(:,3);
patch(X(face), Y(face), Z(face), 'g', 'facealpha', 0.9); 
end

 for i=1:1:20

    theta1= trasaA(i);
    theta2= trasaB(i);
    theta3= trasaC(i);
    theta4= trasaD(i);
    theta5= trasaE(i);
    theta6= trasaF(i);

    [T10,T20,T30,T40,T50,T60,T70,T80,T90]=transmat(theta1,theta2,theta3,theta4,theta5,theta6);
    
   ax = T10(1,4);   ay = T10(2,4);   az = T10(3,4);
   bx = T20(1,4);   by = T20(2,4);   bz = T20(3,4);
   cx = T30(1,4);   cy = T30(2,4);   cz = T30(3,4);
   dx = T40(1,4);   dy = T40(2,4);   dz = T40(3,4);
   ex = T50(1,4);   ey = T50(2,4);   ez = T50(3,4);
   fx = T60(1,4);   fy = T60(2,4);   fz = T60(3,4);
   gx = T70(1,4);   gy = T70(2,4);   gz = T70(3,4);
   hx = T80(1,4);   hy = T80(2,4);   hz = T80(3,4);
   kx = T90(1,4);   ky = T90(2,4);   kz = T90(3,4);
    
    link1XCoords = [125 ax+125];
    link1YCoords = [125 ay+125];
    link1ZCoords = [125 az+125];

    link2XCoords = [ax+125 bx+125];
    link2YCoords = [ay+125 by+125];
    link2ZCoords = [az+125 bz+125];
   
    link3XCoords = [bx+125 cx+125];
    link3YCoords = [by+125 cy+125];
    link3ZCoords = [bz+125 cz+125];
    
    link4XCoords = [cx+125 dx+125];
    link4YCoords = [cy+125 dy+125];
    link4ZCoords = [cz+125 dz+125];
  
    link5XCoords = [dx+125 ex+125];
    link5YCoords = [dy+125 ey+125];
    link5ZCoords = [dz+125 ez+125];
  
    link6XCoords = [ex+125 fx+125];
    link6YCoords = [ey+125 fy+125];
    link6ZCoords = [ez+125 fz+125];
    
      
    link7XCoords = [fx+125 gx+125];
    link7YCoords = [fy+125 gy+125];
    link7ZCoords = [fz+125 gz+125];
    
      
    link8XCoords = [gx+125 hx+125];
    link8YCoords = [gy+125 hy+125];
    link8ZCoords = [gz+125 hz+125];
    
      
    link9XCoords = [hx+125 kx+125];
    link9YCoords = [hy+125 ky+125];
    link9ZCoords = [hz+125 kz+125];
    
    
    p1=plot3(link1XCoords,link1YCoords,link1ZCoords,'b',link2XCoords,link2YCoords,link2ZCoords,'k',link3XCoords,link3YCoords,link3ZCoords,'k',...
             link4XCoords,link4YCoords,link4ZCoords,'r',link5XCoords,link5YCoords,link5ZCoords,'r',link6XCoords,link6YCoords,link6ZCoords,'b',...
             link7XCoords,link7YCoords,link7ZCoords,'b',link8XCoords,link8YCoords,link8ZCoords,'k');
    
    set(p1,'Linewidth',2); 
    pause(0.1);
    axis([0 250 0 250 0 250]);
 end


end