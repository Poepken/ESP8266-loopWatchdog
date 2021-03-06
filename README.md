# ESP8266-loopWatchdog

## English
Example Code for a ESP8266 based sensor that delivers data to 
a MQTT broker and utilizes the "loop watchdog" on top of the
hardware and software watchdogs. See [here](https://github.com/esp8266/Arduino/issues/1532) for the original idea.

The change of adding a "loop watchdog" has improved reliability greatly - instead of 
the sensor freezing every couple of days, it runs now for
a couple of months without interruption. 

## Deutsch

Beispielcode für einen ESP8266 basierten Sensor, der Daten an einen
MQTT Broker liefert, und mit einem "loop watchdog" arbeitet, der
zusätzlich zu dem "hardware watchdog" und "software watchdog" dafür
sorgt, dass der Sensor weiter arbeitet. Siehe [hier](https://github.com/esp8266/Arduino/issues/1532) für den Originalpost.

Durch Hinzufügen des "loop watchdog" konnte ich die Zuverlässigkeit
des Sensors deutlich verbessern. Statt alle paar Tage nicht mehr zu
reagieren, läuft der Sensor nun schon für Montate zuverlässig und liefert
seine Daten.

Siehe [in meinem Blog](http://tim.poepken.net/index.php?id=blog&post=esp8266-watchdog) für
eine kurze Übersicht mit weiteren Links.

