 % RRR6D
 close all, clear all, clc,
%% Nastavení

% Cilova pozice
endpos =  [270 45 315 45 90 0];           %Zadat ve stupnich



% Parametry
NodesMax = 2500;                   % Pocet bodu
EPS = 3*pi;                        % Maximalni vzdalenost noveho bodu
R = 5*pi;                          % Polomer RRT*
    

%% Zakladni priprava

%Rozmery prostoru
xmax = 4*pi;                   
ymax = 4*pi;                     

%Prepocet cile na radiany
theta1 = endpos(1)*pi/180;      
theta2 = endpos(2)*pi/180;
theta3 = endpos(3)*pi/180;
theta4 = endpos(4)*pi/180;
theta5 = endpos(5)*pi/180;
theta6 = endpos(6)*pi/180;


%Vypocet moznych konfiguraci cile
konec = [theta1 theta2 theta3 theta4 theta5 theta6];
konec_alt = [];
konec_options = [];
comb = importdata('combinations_coord6.mat');       % Matice vsech moznych kombinaci pro 6D obecne
for i = 1:length(konec)                             
    if konec(i) >= 2*pi
        konec_alt(i) = konec(i) - 2*pi;             % Pricteni a odecteni 2*pi od zadanych hodnot pro ziskani obou smeru toceni
    end
    if konec(i) < 2*pi      
        konec_alt(i) = konec(i) + 2*pi;
    end
end
coord = [konec;konec_alt];                          % V matici coord jsou oba smery toceni pro vsechny ramena
% Vytvoreni matice obsahujici vsechny mozne cile vzhledem k zadani
for i = 1:length(comb)
    konec_options(i,:) = [coord(comb(1,i),1); coord(comb(2,i),2);coord(comb(3,i),3);coord(comb(4,i),4);coord(comb(5,i),5);coord(comb(6,i),6)];
end
goal.position=konec_options;

% Deklarace promennych
% ur = UR_robot('robot','ip','10.1.1.5');               % Odkomentovat pri pouziti s UR5e
% S1 = -(ur.config)                                     % Odkomentovat pri pouziti s UR5e
% S1(1)=-(S1(1));                                       % Odkomentovat pri pouziti s UR5e
% S1(5)=-(S1(5));                                       % Odkomentovat pri pouziti s UR5e

S1 = [0 0 0 0 0 0];                                     % Zakomentovat pri pouziti s UR5e
S2 = [2*pi 2*pi 2*pi 2*pi 2*pi 2*pi];
start.position= S1' + S2;
start.value = 0;
start.parent = 0;
goal.value = 0;
nodes(1) = start;
Loading=0;


%% RRT Cast
 tic
% Telo algoritmu
for i = 1:1:NodesMax
    Loading=Loading+(1/NodesMax*100)    
    randomNode = [(rand(1)*xmax) (rand(1)*ymax)  (rand(1)*ymax) (rand(1)*xmax) (rand(1)*xmax) (rand(1)*xmax)];     %Vyvoreni nahodneho bodu
    distance = [];
  
    % Nalezeni nejblizsiho bodu
    for j=1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.position, randomNode);
        distance = [distance tmp];
    end 
    [val, idx] = min(distance);
    nearNode = nodes(idx);
    
    % Vytvoreni noveho bodu ve vzdalenosti EPS
    newNode.position = steer(randomNode, nearNode.position, val, EPS);
  
    % Detekce kolize mezi nahodnym a nejblizsim bodem
     if jeKolize(newNode.position, nearNode.position)
        newNode.value = dist(randomNode, nearNode.position) + nearNode.value;
   
            
        
%% RRT* Cast
        
        nearestNode = [];     
        neighborNodes = 1;
        
        % V tomto cyklu jsou zjisteny vsechny body v okruhu r kolem noveho bodu
        for j = 1:1:length(nodes)
            if dist(nodes(j).position, newNode.position) <= R && jeKolize(nodes(j).position, newNode.position)
                % Pokud je spojeni bez kolize jsou zapsany jako nearestNode
                nearestNode(neighborNodes).position = nodes(j).position;
                nearestNode(neighborNodes).value = nodes(j).value;
                neighborNodes = neighborNodes+1;
            end

        end
        
        % Deklarovani nejblizsiho bodu a nejmensi hodnoty
        minNode = nearNode;
        minCost = newNode.value;
        
        % V tomto cyklu je zjisten nejblizsi bod s nejmensi hodnotou
        for k = 1:1:length(nearestNode)
            if nearestNode(k).value + dist(nearestNode(k).position, newNode.position) < minCost ... 
               && jeKolize(nearestNode(k).position, newNode.position)
                %Pokud ma bod mensi hodnotu nez minCost je novy minNode
                minNode = nearestNode(k);
                minCost = nearestNode(k).value + dist(nearestNode(k).position, newNode.position);
            end
        end
        
        % minNode se stane novym rodicovskym bodem
        for j = 1:1:length(nodes)
            if nodes(j).position == minNode.position 
                newNode.parent = j;
            end
        end
        
        % Zapsani bodu do vektoru k ostatnim 
        nodes =  [nodes newNode];
        
    end
end

%% Spojeni cile se stromovou strukturou

distGoal =[];
finalparent =[];
Endlim = 4*pi;      % Polomer pro vyhledavani cile

% Tento cyklus probehne pro vsech 64 moznych cilu
for k=1:1:length(goal.position)
 
    D = [];     % D je vektor obsahujici vsechny body spojitelne s cilem k   
  
    % Kontrola vsech bodu ve strome 
    for j = 1:1:length(nodes)
        
         % Kombinace vsech bodu a cilu je zkontrolovana
         % V okruhu Endlim jsou kontrolovany kolize
         if dist(nodes(j).position,goal.position(k,:)) < Endlim && jeKolize(nodes(j).position,goal.position(k,:))
             
             % Pokud bod-cil projde podminkou je zmerena jejich vzdalenost
             tempDistance = dist(nodes(j).position, goal.position(k,:));
             D = [D tempDistance];      % Vektor vzdalenosti         
         else
             D = [D NaN];               % Pokud bod-cil je v kolizi nebo dal nez Endlim je hodnota nahrazena NaN
         end
    end

    % Deklarace promenne skip v pripade ze cil k nema zadny vhodny bod
    if isnan(min(D)) == 1
        skip=1;
    else
        skip=0;
    end
    % Pro kazdy cil k je vybran nejblizsi bod a jeho rodicovsky bod
    [val, idx] = min(D);        
    goal.parent = idx;                
    v=0;                    % Promenna v obsahuje hodnotu cile
    tempgoal=goal;

    while tempgoal.parent ~= 0
        % V tomto cyklu je zjistena hodnota cile
        next = tempgoal.parent; 
        v = v + dist(goal.position(k,:),nodes(next).position);   
        tempgoal = nodes(next); 
        % Pokud vektor D neobsahoval zadny bod je v = NaN
              if skip == 1    
                 v = NaN;
              end
    end
      % Ve vektoru disGoal je zapis hodnot vsech cilu
      % Ve vektrou finalparent je zapis vsech rodicovskych bodu 
      % Poradi hodnot vtechto vektorech vzajemne odpovidaji
      distGoal= [distGoal v];
      finalparent = [finalparent goal.parent];
end

% V pripade ze nebyla nalezena zadna cesta je vypsana chyba
if isnan(min(distGoal)) == 1
    disp('Chyba: Cesta nenalezena');
    return
end

% Zde je z vektoru disGoal vybran cil s nejmensi hodnotou
[val, idx] = min(distGoal);
goal.position = goal.position(idx,:);       % Prepsani mnozini cilu na jeden vysledny
goal.parent = finalparent(idx);             % Prirazeni spravneho rodicovskeho bodu cili
End = goal;
nodes = [nodes goal];
CoordM = [];                                % Finalni koordinacni matice pro manipulator
counter = 0;                                % Promenna pro cyklus k vykreslovani

% Zpetne dohledavani cesty od cile po pocatek
while End.parent ~= 0
    next = End.parent;
    CoordM= [CoordM; End.position];         
    counter = counter + 1;
    End = nodes(next);
end
toc
CoordM = [CoordM ; start.position];        % Kompletni koordinacni matice

% Uprava hodnot koordinacni matice pro rizeni manipulatoru UR5e
coords = CoordM';
l=size(coords,2);
om=ones(6,l)*2*pi;
finalcoord=-(coords-om);
finalcoord=flip(finalcoord,2);
finalcoord(1,:)=-(finalcoord(1,:));
finalcoord(5,:)=-(finalcoord(5,:));


%% Vykreslovani 

counter = counter + 1;
for c = counter:(-1):2
   
    trasaA=linspace(CoordM(c,1),CoordM(c-1,1),20);
    trasaB=linspace(CoordM(c,2),CoordM(c-1,2),20);
    trasaC=linspace(CoordM(c,3),CoordM(c-1,3),20);
    trasaD=linspace(CoordM(c,4),CoordM(c-1,4),20);
    trasaE=linspace(CoordM(c,5),CoordM(c-1,5),20);
    trasaF=linspace(CoordM(c,6),CoordM(c-1,6),20);
    
    cesta(trasaA,trasaB,trasaC,trasaD,trasaE,trasaF);

end


% Rizeni manipulatoru UR5e
% ur.a_joint = 3;                  % Celou sekci odkomentovat pri pouziti s UR5e  
% ur.v_joint = 5;
% ur.r = 0.05;
% ur.set_config(finalcoord);


