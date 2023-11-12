Stručný návod k provozu:
__________________________________________________________________

Spuštění a parametry:

Otevřít soubor RRTSTAR.m
Nastavení cílové pozice "endpos" na řádku 6
Nastavení parametrů "NodeMax", "EPS" a "R" na řádcích 11-13

Pro použití skriptu bez UR5e:
Zakomentovat řádky 50-54 a 243-246
Odkomentovat řádek 56
__________________________________________________________________

Nastavení překážek:

Otevřít soubor obs.m
Nastavení rozměrů překážek na řádku 5
Nastavení pozice bodu A na řádku 12

___________________________________________________________________

Příkazy řídící manipulátor jsou z knihovny UR_robot, která slouží k řízení manipulátoru.
Pro změnu nastavení použijte UR_Matlab_manual.

Tato knihovna byla poskytnuta v mechlabu, ale bohužel nebyl dohledán autor.