
function [obstacles,n] = obs() 

% Zde se voli bod A a velikost prekazek
sizes = [10 10 10           % Nemìnit
        90 50 125
        30 50 40
        80 83 130
        250 85 250
        45 35 185];

A_S = [120 120 137.5        % Nemìnit
     45 100 0
     60 100 126
     0  167 0
     0 0 0
     145 145 0];
 
n =size(A_S,1);

% Urceni ostatnich 7 bodu kvadru
for i=1:1:n
    Size = sizes(i,:);
    a = A_S(i,:);

    b = [a(1)+Size(1)   a(2)           a(3)];
    c = [a(1)+Size(1)   a(2)+Size(2)   a(3)];
    d = [a(1)           a(2)+Size(2)   a(3)];
    e = [a(1)           a(2)           a(3)+Size(3)];
    f = [a(1)+Size(1)   a(2)           a(3)+Size(3)];
    g = [a(1)+Size(1)   a(2)+Size(2)   a(3)+Size(3)];
    h = [a(1)           a(2)+Size(2)   a(3)+Size(3)];       

% Ulozeni bodu v 3D matici
obstacles(:,:,i) = [ a; b; c; d; e; f; g; h];

end
end