function collision = interCheck(pointx, pointy)
    
   [obstacles,n]=obs();
   
   for i=1:1:n                     % Cyklus pro n-prekazek
    
   obstacle=obstacles(:,:,i);      % Nacteni matice s body prekazky
   
   % Kontrola pruseciku vsech hran kvadru se spojnici dvou bodu 
   intr1 = lineintr(obstacle(1,1:2),obstacle(2,1:2),pointx(1:2),pointy(1:2));
   intr2 = lineintr(obstacle(2,1:2),obstacle(3,1:2),pointx(1:2),pointy(1:2));
   intr3 = lineintr(obstacle(3,1:2),obstacle(4,1:2),pointx(1:2),pointy(1:2));
   intr4 = lineintr(obstacle(1,1:2),obstacle(4,1:2),pointx(1:2),pointy(1:2));  
   intr5 = lineintr(obstacle(1,[1,3]),obstacle(2,[1,3]),pointx([1,3]),pointy([1,3]));
   intr6 = lineintr(obstacle(2,[1,3]),obstacle(6,[1,3]),pointx([1,3]),pointy([1,3]));
   intr7 = lineintr(obstacle(5,[1,3]),obstacle(6,[1,3]),pointx([1,3]),pointy([1,3]));
   intr8 = lineintr(obstacle(1,[1,3]),obstacle(5,[1,3]),pointx([1,3]),pointy([1,3]));
   intr9  = lineintr(obstacle(1,2:3),obstacle(5,2:3),pointx(2:3),pointy(2:3));
   intr10 = lineintr(obstacle(5,2:3),obstacle(8,2:3),pointx(2:3),pointy(2:3));
   intr11 = lineintr(obstacle(8,2:3),obstacle(4,2:3),pointx(2:3),pointy(2:3));
   intr12 = lineintr(obstacle(1,2:3),obstacle(4,2:3),pointx(2:3),pointy(2:3));
   
   % Kontrola pohledu xy
   if (intr1 == 1 || intr2 == 1 || intr3 == 1 || intr4 == 1)   || ...
      (pointx(1)>obstacle(1,1) && pointx(1) < obstacle(3,1)    && ...
       pointx(2) > obstacle(1,2) && pointx(2) < obstacle(3,2)  && ...
       pointy(1)>obstacle(1,1) && pointy(1) < obstacle(3,1)    && ...
       pointy(2) > obstacle(1,2) && pointy(2) < obstacle(3,2))
     
       xy = 1;
   else
       collision = 1;
       continue
   end
     
   % Kontrola pohledu xz
   if (intr5 == 1 || intr6 == 1 || intr7 == 1 || intr8 == 1)   || ...
      (pointx(1)>obstacle(1,1) && pointx(1) < obstacle(6,1)    && ...
       pointx(3) > obstacle(1,3) && pointx(3) < obstacle(6,3)  && ...
       pointy(1)>obstacle(1,1) && pointy(1) < obstacle(6,1)    && ...
       pointy(3) > obstacle(1,3) && pointy(3) < obstacle(6,3))
      
       xz = 1;
   else
       collision = 1;
       continue
   end
   
   % Kontrola pohledu zy
   if (intr9 == 1 || intr10 == 1 || intr11 == 1 || intr12 == 1) || ...
      (pointx(2)<obstacle(4,2) && pointx(2) > obstacle(5,2)     && ...
       pointx(3) > obstacle(4,3) && pointx(3) < obstacle(5,3)   && ...
       pointy(2)<obstacle(4,2) && pointy(2) > obstacle(5,2)     && ...
       pointy(3) > obstacle(4,3) && pointy(3) < obstacle(5,3))
      
       zy = 1;
   else
       collision = 1;
       continue
   end
   
   % Finalni vyhodnoceni ze doslo ke kolizi
   if xy == 1 && xz == 1 && zy == 1   
       collision = 0;
       break
   end
   
   end
end

