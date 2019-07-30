# Odroid - testy

Skrypty do początkowych testów Okonia - sterowanie silnikami, odczyty czujników i IMU, wprowadzenie regulacji.

### Konfiguracja

Podstawowym skryptem zawierającym definicje i prowadzenie wątków jest *MainTests.py*.  
Skrypt *RUN_TEST.py* uruchamia go w odpowiedniej konfiguracji (z zapisem danych wyjściowych do pliku *output.log*) i to jego uruchamiamy
```
sudo python3 RUN_TEST.py
```
Odczyt danych wyjściowych należy zrealizować w osobnym terminalu, który będzie śledził zmiany pliku *output.log*. Można użyć do tego skryptu *tailOutput.sh*:
```
sudo ./tailOutput.sh
```
lub po prostu polecenia:
```
sudo tail -f output.log
```

### Zadawanie poleceń

W terminalu, w którym odpaliliśmy główny skrypt możemy wpisywać polecenia zadające ruch silnikom w formule:
```
<nazwa silnika> <prędkość> (<czas działania>)
```
gdzie  
< nazwa silnika > = hl / hr / vl / vm / vr (horizontal right, itd.) lub h / v (2 silniki horyzontalne / 3 wertykalne na raz)  
< prędkość > = wartości [-1000, 1000] (lub odpowiednio zależnie od ustawień klasy MotorControl.py)  
(opcjonalnie < czas działania > = czas w sekundach).  
Przykłady:
```
hl -150
v 500 5
vm 90 2.5
```

##### Zatrzymanie 

Dodatkowo po wpisaniu
```
s
```
wszystkie silniki powinny się zatrzymać.
