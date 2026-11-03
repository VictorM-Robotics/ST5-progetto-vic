# ST5 — navigation sur grille

Base software Python per navigare su una griglia di linee bianche su sfondo nero,
consegnare a un incrocio o a metà segmento e tornare a `START`.
Python 3.9+, libreria standard per simulazioni e test. I moduli Python allegati
sono stati importati come base. Il firmware ricevuto è conservato in
`firmware_received/`; la nuova variante watchdog è in `firmware_watchdog/`.
I test firmware richiedono anche `g++`.

## Verifica senza hardware

```bash
cd /workspace/ST5-progetto-vic
python -m unittest discover -s tests -v
python simulation_mission.py
python simulation_navigation.py
python simulation_odometry.py
```

La nuova simulazione esegue Start → 75% di B–C → consegna → Start con sensori
simulati, senza aprire porte seriali. I test coprono tutti i 25 incroci e 240
destinazioni intermedie (80 archi diretti × tre frazioni), oltre ai casi di errore.

## Componenti e ciclo missione

`grid_map.py`, `path_planning.py` e `localization.py` definiscono mappa, percorsi
e stato reale confermato. Dijkstra minimizza il costo degli archi senza penalità
di svolta; le lunghezze non misurate restano nominali. La base esterna è `START`,
collegata a Y; `S` è un incrocio diverso.

`navigation_controller.py` richiede manovre e suivi de ligne. Ora permette
ripartenze a metà segmento con `load_mission(..., stopped=True)`: il riferimento
si inverte solo dopo conferma fisica della manovra e acquisizione della linea.
`odometry_adapter.py` conserva la frazione iniziale; chiamare `begin_following()`
dopo ogni manovra confermata per escludere le rotazioni dalla distanza percorsa.

`mission_controller.py` supervisiona andata, consegna e rientro:

```python
from grid_map import GridMap
from localization import RobotLocalizer
from navigation_controller import NavigationController
from mission_controller import MissionController

grid = GridMap()
mission = MissionController(NavigationController(grid, RobotLocalizer(grid)))
event = mission.start({'type': 'edge', 'edge': ('B', 'C'), 'progress': 0.75})
```

Un unico esecutore sul robot deve gestire le richieste restituite dai metodi:

1. `next_command()` richiede la manovra; `maneuver_result()` la conferma solo dopo
   esecuzione reale e acquisizione del ramo corretto.
2. Durante il seguito linea, inoltrare `progress_update()` e gli arrivi validati
   tramite `confirmed_intersection()`. Alla base questo evento rappresenta
   l'arrivo al punto di arresto, non un incrocio.
3. Per un punto intermedio il controllore fisico deve rallentare e fermarsi;
   `reached_partial_target(stopped=True)` verifica posizione e arresto.
4. A destinazione viene emesso `STOP_REQUEST`: eseguire l'arresto e chiamare
   `confirm_stop(stopped=True)` su conferma fisica. La fase è `WAITING_DELIVERY`.
5. Solo la procedura di consegna può chiamare
   `confirm_delivery(delivered=True, stopped=True)`: registra `DELIVERY_COMPLETED`
   e pianifica il ritorno dalla posizione effettiva, anche a metà segmento.
6. Al ritorno un'altra conferma di arresto abilita `RETURN_COMPLETED`.

`mission.events` conserva gli eventi del ciclo missione e i feedback francesi;
gli eventi di navigazione sono in `mission.navigation.events`. I metodi restituiscono
le richieste da gestire immediatamente. Non sono oggetti thread-safe: serializzare
le chiamate. Dopo `STOPPED` serve riconciliazione esplicita, senza riavvio automatico.

## Integrazioni ancora necessarie

Vedere [requisiti PC, camera e robot](docs/INTEGRATION_REQUIREMENTS.md).
Interfaccia PC, rete, visione, suivi de ligne, attuatore della consegna,
allineamento Start–Y, arresto riconoscibile alla base e recupero ostacoli non sono
ancora implementati. Servono misure reali, calibrazione e controllo della frenata:
la soglia software sul superamento del target non realizza un arresto fisico.

`motor_bridge.py` ora usa il protocollo USB v2 con CRC e riarmo esplicito;
aggiornare insieme firmware e driver. Vedere [Bluno e watchdog](docs/BLUNO_WATCHDOG.md)
per caricamento, limiti e prove. La compilazione AVR rimane da eseguire.
`closed_loop_turn.py` resta da validare fisicamente. L'accesso seriale richiede
`pyserial`. Gli ACK non provano l'arresto
fisico. Verificare firmware/watchdog e arresto indipendente prima delle prove
autonome. Nessun test qui valida l'hardware.

Il [README ricevuto](docs/README_received.md) è conservato come riferimento storico:
descrive anche cartelle e test non allegati, che non vanno considerati presenti
o verificati in questo checkout.
