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
