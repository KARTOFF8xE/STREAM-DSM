# Ziel ist die Erstellung einer latencies.csv, welche als header "n, freq, latency" hat

* schaue ob als Startbefehl angegebener Ordner existiert
    > in ihm liegt folgende Ordnerstruktur vor: freq/n/i/*.txt

* Algorithmus:
```
für alle n:
   für alle freq:
       latenzSumme=0
       für alle i:
           a=berechne die jeweiligen Schnittlatenzen
           latenSumme+=a
       a/=max(i)
       schreibe in Datei: n, freq, latency        
```
