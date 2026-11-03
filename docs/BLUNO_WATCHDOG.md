# Bluno MEGA + Raspberry Pi 3 B+ — USB e watchdog

Confermati: ATmega2560, comunicazione USB, quattro motori con encoder e pin
uguali al `parameters.h` ricevuto. Dalla foto le ruote sembrano Mecanum;
disposizione, versi e cinematica vanno ancora verificati sul robot.

## Caricamento e compatibilità

Aprire `firmware_watchdog/serial_link/serial_link.ino`, mantenendo nella stessa
cartella `parameters.h` e `protocol.h`. `firmware_received/serial_link` conserva
i file ricevuti senza modifiche. Non mescolare gli sketch: quello nuovo non usa
`Task.cpp`, `Sercomm.cpp`, `Motor.cpp` o `MyServo.cpp`.

In Arduino IDE selezionare **Arduino Mega or Mega 2560 / ATmega2560**, pacchetto
ufficiale Arduino AVR Boards, e la porta USB Bluno. Verificare che il bootloader
della vostra Bluno accetti l'upload Mega 2560 standard; non modificare bootloader
o fuse per aggirare errori di upload. Con CLI e core già installati:

```bash
arduino-cli compile --fqbn arduino:avr:mega firmware_watchdog/serial_link
```

**Compilazione AVR non eseguita**: toolchain assente e proxy cloud irraggiungibile.
I test eseguono lo sketch compilato con g++ e I/O Arduino simulato, anche tramite
il driver Python reale. Non verificano timer/PWM, USB o comportamento fisico.

Il driver `motor_bridge.py` richiede **protocollo v2**, incompatibile con il vecchio
A12/C senza frame. Aggiornare firmware e Python insieme. Nessun fallback legacy.
La variante gestisce motori, encoder e stato watchdog: non attiva il servo camera
sul pin 26 e non implementa i vecchi comandi servo, IR o ultrasuoni. Non fornisce
ancora protezione ostacoli o navigazione autonoma completa.

## Comportamento del watchdog di comunicazione

- Avvio, reset e handshake: PWM zero e motori disarmati.
- `arm()` è esplicito: azzera le uscite, arma e avvia il timer.
- Solo comandi motori validi e riarmo esplicito aggiornano il timer.
  Le letture encoder/stato non mantengono il movimento.
- Dopo 350 ms senza comandi motori validi: PWM e consigne zero, stato disarmato.
  Una successiva consigna non nulla viene rifiutata: nessun riarmo automatico.
- `stop()` azzera subito PWM/consigne senza rampa; mantiene l'eventuale stato
  armato. `disarm()` ferma e chiude la sessione.
- CRC errato, frame incompleto scaduto (40 ms), sequenza errata o PWM fuori
  limite provocano disarmo. Un frame incompleto non viene applicato ai motori.
- Rampa: 10 PWM ogni 20 ms, limite assoluto 100. Un cambio verso passa per zero.
  Il limite non rappresenta una velocità fisica validata.
- Coppie encoder lette atomicamente; richieste anteriori/posteriori separate,
  quindi non simultanee per le quattro ruote.

È un watchdog **nel loop**, non un watchdog hardware CPU: non garantisce arresto
se il microcontrollore stesso si blocca. L'eventuale watchdog CPU richiede verifica
del bootloader e dei reset Bluno. Serve ancora un arresto indipendente.
PWM zero non significa necessariamente frenata attiva o ruote istantaneamente ferme.

Non viene avviato un thread che ripeta automaticamente l'ultimo PWM: potrebbe
nascondere un blocco del controllo. Il futuro controllore deve inviare consigne
fresche, indicativamente a 20 Hz, verificando le latenze sul Pi 3 B+ con camera
attiva. Una pausa oltre il timeout deve fermare il robot.

## Prima prova senza movimento

Sul Raspberry installare `pyserial` nel proprio ambiente Python. Individuare
la porta effettiva, preferibilmente il percorso stabile `/dev/serial/by-id/...`;
non è necessariamente `/dev/ttyACM0`.

```python
from motor_bridge import MotorBridge, MotorConfig

with MotorBridge(MotorConfig(port='/dev/ttyACM0'), allow_hardware=True) as motors:
    print(motors.get_encoder_counts())
    motors.stop()
    # Nessun arm(): movimento non autorizzato.
```

Prime prove con ruote sollevate, supervisione e interruzione fisica dell'alimentazione
motori disponibile. Verificare una ruota alla volta: FR/FL/RR/RL, direzione,
encoder e PWM minimo. Chiamare `arm()` solo dopo predisposizione della prova;
anche `supervised_pulse()` richiede il riarmo esplicito.

Verificare e misurare sul robot:

1. Stop immediato del PWM senza rampa.
2. Stop dopo interruzione dei comandi o scollegamento USB entro il timeout nominale
   più latenza del loop, misurando anche il tempo di arresto meccanico.
3. Nessuna ripartenza alla successiva consigna senza riarmo.
4. Nessun movimento a reset o riconnessione.
5. Letture encoder periodiche che non impediscono il timeout motori.

`test_turn_controller.py` riabilita i motori solo dopo flag hardware/bench,
calibrazione e conferma BENCH. Non dedurre angoli fisici da ruote sollevate:
slittamento e cinematica Mecanum richiedono prove sul terreno successive.

## Protocollo v2

115200 baud, 8N1. Frame fissi di 17 byte, little-endian:

| Byte | Contenuto |
|---|---|
| 0–1 | `A5 5A` |
| 2 | versione `02` |
| 3 | comando ASCII; risposta = comando OR `80` |
| 4–5 | sequenza uint16 |
| 6 | richiesta zero; risposta stato |
| 7–14 | payload di 8 byte |
| 15–16 | CRC16-CCITT-FALSE dei byte 2–14 |

CRC: polinomio `1021`, iniziale `FFFF`, nessuna riflessione/XOR finale.
Stati: 0 OK, 1 disarmato, 2 argomento errato, 3 comando sconosciuto, 4 sequenza
o sessione errata. Incremento sequenza modulo 65536. `H` apre una sessione
disarmata; `X` è accettato con qualsiasi sequenza se il CRC è valido.

| Comando | Significato / payload risposta |
|---|---|
| `H` | handshake: versione u8, timeout u16, limite PWM u16, padding |
| `R` | riarmo esplicito, uscite zero |
| `C` | richiesta: quattro PWM int16 FR/FL/RR/RL |
| `X` | stop, disarmo e chiusura sessione |
| `N` / `n` | due conteggi int32 anteriori / posteriori |
| `Q` | primo byte 1 armato, 0 disarmato |

CRC rileva corruzioni, non autentica il mittente. Nessun reconnect o retry
automatico dei comandi di movimento.
