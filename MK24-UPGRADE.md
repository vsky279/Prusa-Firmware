# MK2.4 – přechod na Prusa Firmware 3.14.1

## Výchozí stav a rozsah

Nová větev `MK24-3.14.1` vychází přímo z oficiálního tagu `v3.14.1`
(commit `74a577bc0e5902767b072ee85f499dc1361bf6ba`). Původní větev byla
stažena do `custom/MK24-3.14` na commitu
`24fab9457edab4273ed608fe0c233b08017b2637`. Její historie nebyla změněna.
Nic nebylo flashováno, EEPROM tiskárny nebyla čtena ani změněna.

Audit všech šesti vlastních commitů:

| Commit | Obsah a rozhodnutí |
|---|---|
| `6e7a0f88` | LF pro shell skripty, kompatibilnější zjištění větve v PF-build, oprava názvu proměnné komunitních jazyků. Nepřenášeno: oficiální CMake/Ninja cestu tyto skripty nepoužívá. PF-build není tímto portem ověřen. |
| `31b15ced` | Opravy překladu bez senzoru filamentu a bez TMC/fancheck. Přeneseny jako malé podmíněné úpravy aktuálních souborů; navíc ošetřena nová `filament_presence_check()` v util.cpp. |
| `01f44f97` | Vlastní varianta odvozená od MK2.5. Nová varianta vznikla z aktuálního upstream MK25-RAMBo13a, nikoli nahrazením nových souborů starými. |
| `c11e59f1` | E=161.3, Ymin=-2.2, bez fancheck/senzoru, hotend PID 40.925/4.875/86.085, FILAMENTCHANGE_RFEED=400. Zachováno. |
| `585fdb5a` | Vadný třetí XYZ bod: zachováno vynechání měření, původní nestabilní rekonstrukce nahrazena fitováním tří skutečně měřených bodů. |
| `24fab945` | Host UART2 / 250000 baud. Zachováno pouze pro vlastní variantu; doplněna výhradní kontrola portu a zablokování MMU komunikace/startu. |

Další zachované parametry varianty: miniRAMBo 1.3, identifikace MK2.5 pro kompatibilitu
G-code, LCD název MK2.4, HEATBED_V2, STEEL_SHEET, NEW_XYZCAL, kroky XYZ
100/100/400, home Y=-2.2, pracovní rozsahy X=0..250, Y=-2.2..210,
Z=0.15..210, posuny sondy 23/5/-0.4, proudy motorů, rychlosti, akcelerace,
preheat, termistory, PINDA kompenzace a PID podložky 126.13/4.30/924.76.
Definice mesh mezí 24..228 / 6..210 zůstaly; aktuální kód skutečné mezní
polohy odvozuje z BED_X0/BED_Xn, BED_Y0/BED_Yn a posunů sondy a dává stejné hodnoty.
Staré END_FILE_SECTION=20000 nebylo přeneseno: zachována upstream hodnota 30720
pro kontrolu konce tiskového souboru. Nové upstream volby a ochrany zůstaly.

## Volba kalibrační matematiky

Čtyři XYZ reference jsou jiné body než výšková síť 3×3 / 7×7.
Pořadí XYZ na tomto HEATBED_V2 je přední levý, přední pravý, zadní pravý,
zadní levý. Chybí index 2. Měří se pouze 0, 1, 3.

| Přístup | Posouzení |
|---|---|
| Původní rekonstrukce | Dělí y1-y2, obsahuje odečítání velkých téměř stejných členů a volbu odmocniny. Pro vodorovnou přední hranu vznikne NaN; předpoklad pravého úhlu zkresluje právě měřený skew. Zamítnuto. |
| Rovnoběžník p2=p1+p3-p0 | Numericky jednoduchý, zachovává afinní geometrii včetně skew. Syntetický bod ale není nezávislé měření; jeho přidání do stejně váženého fitu mění váhy a koreluje šum. Není potřeba jej vyrábět. |
| Tři skutečné reference | Zvoleno. Zachován model upstreamu se dvěma jednotkovými osami a posunem (čtyři neznámé, šest měřených souřadnic). Nejde o volný šestiparametrový afinní fit, který by přesně absorboval každý šum a chybu měřítka. |

Zachován aktuální Gauss–Newton / Gauss–Seidel solver, váhy, kontroly residuí,
orthogonalizace pod 0.12° a inverze world→machine. Všechny součty, kontroly
a přepočet posunu vynechávají původní index 2 bez změny mapování do PROGMEM.
Velký pravoúhlý trojúhelník má dostatečné prostorové rozložení. Kontrola
orientovaného obsahu vyžaduje součin délek alespoň 100 mm² a sin úhlu mezi
hranami alespoň 0.5. Dále se kontrolují konečná čísla, diagonály normálních
rovnic, konvergence posledního kroku (posun 0.001 mm / úhel 1e-5 rad),
residua ≤0.8 mm i po orthogonalizaci a determinant před inverzí ≥0.5.
Pro MK2.4 se výsledek nad upstream hranicí 0.25° (skew nebo úhel osy)
odmítá místo přijetí extrémní korekce. Tyto prahy vyžadují fyzické ověření.

Vynechání probíhá před jakýmkoli pohybem ve smyčce XYZ, v obou pokusech
i čtyřbodových diagnostických smyčkách. Neúspěšný druhý sken se nepoužije pro fit.
Chyba detekce ihned vrací POINT_NOT_FOUND; vyčerpaný retry vrací
POINT_SCAN_FAILED. G80, sample_z a vyhodnocení magnetů výškové sítě se nemění.
Samostatné ruční pohyby/G-code mohou stále přikázat pohyb na libovolné místo;
toto není zákaz souřadnice pro běžný tisk ani pro výškový mesh.

## Závislosti, EEPROM a bezpečnost

`Marlin_main.cpp` řídí XYZ, vyvolá `find_bed_offset_and_skew()`, následně
`sample_z()`. Vyhledávání cíle provádí současný `xyzcal.cpp` a jeho obrazový
scanner. Fit poskytne vec_x/vec_y/center; `world2machine_update()` a planner
je používají pro pohyby, a mesh přidává výškovou korekci. Záznamy
EEPROM_BED_CALIBRATION_CENTER, VEC_X, VEC_Y, Z_JITTER, XYZ_CAL_SKEW
a záznamy ocelových plátů zachovávají upstream formát a adresy.
Žádná vlastní migrace ani nový EEPROM záznam nebyly přidány. Upstream 3.14.1
přidává zejména záznamy power-panic nastavení a kontroly filamentu v dolní
části EEPROM; nepřepisuje kvůli tomuto portu formát XYZ transformace.
Pomocný údaj skew se u MK2.4 uloží až po úspěšných kontrolách.
Před aktualizací je přesto nutná úplná záloha: firmware při startu a při
kalibraci EEPROM běžně mění; kompatibilita celého stavu při downgrade nebyla
na tiskárně ověřena. Výchozí kroky/PID v hlavičce nepřepíší automaticky již
uložené uživatelské hodnoty; po flashi je nutné porovnat skutečné nastavení.

Teplotní runaway, MINTEMP/MAXTEMP, watchdog, safety timer, soft/endstop limity,
ochrana studené extruze a řízení chladicího ventilátoru zůstávají zapnuté podle
upstream varianty. Chybí pouze hardwarově nepodporovaný fancheck a senzor filamentu.
UART2 host nemůže současně sloužit MMU: Start() je blokován i při starém
EEPROM příznaku, ovladač MMU nečte/nepíše tento port a nevlastní RX interrupt.
MMU není podporována v této konfiguraci. USB UART0 zůstává portem bootloaderu.

## Sestavení a testy

Oficiální připnuté závislosti: AVR GCC 7.3.0 (Microchip 3.7.0.1796),
prusa3dboards 1.0.6, CMake 3.22.5, Ninja 1.12.1; Python 3.12.14.

```powershell
python utils/bootstrap.py
python build_local.py
python build_local.py --cs
python tests/run_mk24_calibration.py
python tests/run_host_tests.py
```

`build_local.py` používá lokální oficiální závislosti a explicitní Python.
Varianta bez externí xflash má český cíl `MK24-RAMBo13a_en-cs`, nikoli MULTILANG.
Přímé standardní CMake příkazy jsou v hlavním README. V tomto Windows sandboxu
bylo nutné nástroje spouštět s povolením mimo sandbox. Certifikáty se nevypínaly.
Pro host testy byl lokálně přidán Zig 0.13.0 / Clang 18.1.6
(`python -m pip install --target .dependencies/host ziglang==0.13.0`), nikoli
pro sestavení firmwaru. Skript kalibrace umí také přijmout cestu k běžnému C++ překladači.

Baseline: přesný export původního commitu, vlastní izolovaná Git metadata,
oficiální CMake/Ninja. Anglický build prošel: program 194908 B, statická RAM
5336 B. Verze vycházela z Configuration.h (3.14.0, 8066); nejde o důkaz
bitově identického výsledku s dříve flashovaným obrazem.

Kalibrační test extrahuje a překládá skutečnou aktuální fitovací funkci
a skutečnou XYZ smyčku s hardwarovými stuby; původní funkce a rekonstrukce jsou
v testovací fixture přesně exportované z Git commitu `24fab945`.
Proběhne normální i SUPPORT_VERBOSITY build; test je také registrován v CTest.
97 případů v každém: 81 kombinací posunu, rotace, skew a deterministického šumu
0 / 0.02 / 0.08 mm, plus NaN, nekonečno, opakovaný bod, kolinearita,
zrcadlení, téměř kolinearita, změna délky, overflow a extrémní skew;
sedm dalších kontrol těsně pod/nad 0.12° a 0.25° a neplatného počtu referencí.
Chybějící slot obsahuje NaN, aby jeho nechtěné použití bylo odhalitelné.
Skenovací test kontroluje přesné pořadí 0,1,3, selhání senzoru, retry úspěch
a retry vyčerpání; při chybě nesmí dojít k zápisu EEPROM.
Na stejném bezšumovém skew příkladu je součet chyb ve třech měřených bodech
u původního řešení 0.580615 mm, u nového 0.0000352938 mm. Vodorovná přední
hrana původní rekonstrukci prokazatelně vede na nekonečnou/NaN souřadnici.
Tyto hodnoty nejsou měřenou přesností tiskárny ani statistickou garancí.

CTest celkem: 6 ze 7 prošlo (5 ze 6 původních a nová kalibrační sada).
Nezměněný `Prusa_statistics test`
selhává v `tests/PrusaStatistics_test.cpp:649/652`: `string::back()` na
prázdném bufferu; host libc++ to odmítá. Není vydáváno za úspěšný test.
Soubor je totožný s upstreamem a tento test netestuje novou kalibraci.

## První flash, ověření a rollback – pouze ruční postup

1. Uložit původní používaný HEX, výpis konfigurace M503, hodnoty Live Z,
   pláty, PID, kroky, stav kalibrace a kompletní čitelnou zálohu EEPROM.
   Zálohovat pokud možno i flash; přístup přes USB bootloader ověřit předem.
   Vygenerovaný baseline je záložní build zdrojů, nikoli náhrada vlastní zálohy.
2. Odpojit automatické odesílání tisků a host makra. Tiskárna studená, bez
   rozpracovaného tisku, s volným prostorem a dostupným vypínačem.
3. Ručně nahrát nejprve EN přes důvěryhodný uploader pro miniRAMBo/ATmega2560
   a stávající bootloader. Zachovat EEPROM, nevolit factory reset ani erase
   EEPROM. UART2/250000 je provozní host port, nikoli změna bootloader baudrate.
4. Zkontrolovat identifikaci MK2.4/3.14.1, komunikaci UART2, rozumné pokojové
   teploty, činnost chladicího ventilátoru, PINDA a endstopy (M119 / LED).
   S vypnutým ohřevem ověřit krátké pohyby a správné směry; nejezdit naslepo k podložce.
5. Porovnat M503 se zálohou: zejména E161.3, PID, limity a pohybové hodnoty.
   Nespouštět M502/M500 jen kvůli aktualizaci. Nesoulad nejprve vyřešit ručně.
6. XYZ spustit pod přímým dohledem podle pokynů LCD pro sejmutí/nasazení
   ocelového plátu. Musí měřit jen reference 0,1,3 (displej může zachovat
   původní označení 1/4,2/4,4/4). Při pokusu o vadný bod, špatném pohybu,
   nereagující sondě či neobvyklém kontaktu okamžitě přerušit napájením.
7. Po úspěchu ověřit transformaci a limity, samostatně 3×3 a případně 7×7
   G80, opakovatelnost měření a první vrstvu s bezpečným Live Z. Výšková síť
   potřebuje ocelový plát a funkční detekci ve všech skutečně sondovaných místech.
8. Až poté pod dohledem postupně ověřit ohřev/PID, malý tisk a český obraz.
   Elektrické teplotní chyby záměrně nevyvolávat a žádné ochrany nevypínat.
9. Při neúspěchu zastavit tisk, vypnout ohřev a vrátit původní ověřený HEX
   stejným USB postupem. Pokud byl mezitím změněn kalibrační/EEPROM stav,
   obnovit ověřenou vlastní úplnou zálohu vhodným nástrojem nebo znovu
   bezpečně kalibrovat původní firmware. Nová verze nemá automatický downgrade.

Zbývá fyzicky ověřit PINDA opakovatelnost, skutečný mechanický skew, bezpečné
výšky a pohyby, UART2 elektrické připojení, teplotní chování a dynamické
využití stacku. Statická RAM statistika neprokazuje rezervu stacku za provozu.
