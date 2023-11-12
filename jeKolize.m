function  Ano = jeKolize(Node1,Node2)
    
   % Vyplneni prostoru mezi konfiguracemi 1 a 2 pomoci linspace 
   NodeA=linspace(Node1(1),Node2(1),25);
   NodeB=linspace(Node1(2),Node2(2),25);
   NodeC=linspace(Node1(3),Node2(3),25);
   NodeD=linspace(Node1(4),Node2(4),25);
   NodeE=linspace(Node1(5),Node2(5),25);
   NodeF=linspace(Node1(6),Node2(6),25);
   
   % Kontrola kolize pro vsechny konfigurace manipulatoru
   for u = 1:1:25
   the1 = NodeA(u);
   the2 = NodeB(u);
   the3 = NodeC(u);
   the4 = NodeD(u);
   the5 = NodeE(u);
   the6 = NodeF(u);
   
   % Zjisteni poloh koncovych bodu ramen z uhlu natoceni pomoci funkce transmat
   [T10,T20,T30,T40,T50,T60,T70,T80,T90]=transmat(the1,the2,the3,the4,the5,the6);

   % Extrakce bpdu souradnic z transformacnich matic   
   ax = T10(1,4);   ay = T10(2,4);   az = T10(3,4);
   bx = T20(1,4);   by = T20(2,4);   bz = T20(3,4);
   cx = T30(1,4);   cy = T30(2,4);   cz = T30(3,4);
   dx = T40(1,4);   dy = T40(2,4);   dz = T40(3,4);
   ex = T50(1,4);   ey = T50(2,4);   ez = T50(3,4);
   fx = T60(1,4);   fy = T60(2,4);   fz = T60(3,4);
   gx = T70(1,4);   gy = T70(2,4);   gz = T70(3,4);
   hx = T80(1,4);   hy = T80(2,4);   hz = T80(3,4);
   kx = T90(1,4);   ky = T90(2,4);   kz = T90(3,4);

   % Posun techto bodu do naseho souradneho systemu
   pozice0 = [125 125 125];
   pozice1 = [ax+pozice0(1) ay+pozice0(2)  az+pozice0(3)];
   pozice2 = [bx+pozice0(1) by+pozice0(2)  bz+pozice0(3)];
   pozice3 = [cx+pozice0(1) cy+pozice0(2)  cz+pozice0(3)];
   pozice4 = [dx+pozice0(1) dy+pozice0(2)  dz+pozice0(3)];
   pozice5 = [ex+pozice0(1) ey+pozice0(2)  ez+pozice0(3)];
   pozice6 = [fx+pozice0(1) fy+pozice0(2)  fz+pozice0(3)];
   pozice7 = [gx+pozice0(1) gy+pozice0(2)  gz+pozice0(3)];
   pozice8 = [hx+pozice0(1) hy+pozice0(2)  hz+pozice0(3)];
   pozice9 = [kx+pozice0(1) ky+pozice0(2)  kz+pozice0(3)];
   
   % Detekce kolizi usecek reprezentujicich ramena manipulatoru
  
   
   if  (the4 < pi && the4 > 0 && the5 < 3/2*pi && the5 > 1/2*pi ...
     || the4 < 3*pi && the4 > 2*pi && the5 < 3/2*pi && the5 > 1/2*pi ...
     || the4 < pi && the4 > pi && the5 < 7/2*pi && the5 > 5/2*pi ...
     || the4 < 3*pi && the4 > 2*pi && the5 < 7/2*pi && the5 > 5/2*pi)
    
       Ano = 0;
       break
   end
       
   
   if (interCheck(pozice2,pozice3) && interCheck(pozice4,pozice5) && interCheck(pozice5,pozice6) && interCheck(pozice6,pozice7) && interCheck(pozice7,pozice8))
        Ano = 1;    
   else
        Ano = 0;
        break 
   end
   end
end