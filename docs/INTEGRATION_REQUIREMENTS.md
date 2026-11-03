# Requisiti concordati: PC, robot e camera

Hardware confermato: Bluno MEGA (ATmega2560), Raspberry Pi 3 B+, collegamento
USB e pin motori/encoder come negli allegati. Dalla foto le ruote sembrano
Mecanum: orientamento dei rulli e cinematica devono essere verificati sul robot.
Camera su supporto inclinabile, modello/collegamento ancora da identificare.
La variante [watchdog USB](BLUNO_WATCHDOG.md) è predisposta e testata in simulazione;
restano compilazione AVR, caricamento e validazione fisica.

## Missione

Partenza da una linea esterna di Start collegata alla griglia, consegna a un
incrocio o a una frazione di segmento e ritorno al punto di arresto di Start.
Verificare sulla pista la connessione Start–Y assunta dal codice. Il criterio
osservabile di arresto alla base resta da definire.

## Interfaccia PC in francese — da implementare

- « Où souhaitez-vous livrer ? »
- « Type de destination : intersection / point sur un segment »
- « Choisissez l’intersection » oppure « Choisissez le segment » e
  « Position depuis le premier point (%) ».
- B → C, 75 % equivale a C → B, 25 %: mostrare sempre il punto di riferimento.
- « Envoyer la mission », « Arrêter », connessione, fase e anomalie.
- Feedback distinti: « Point de livraison atteint. », « Livraison effectuée. »,
  « Retour au point de départ terminé. ».

Mostrare « Départ (START) » per distinguere la base dall'incrocio S. La conferma
di consegna proverrà dal meccanismo reale o dall'operatore, senza inferirla dall'arrivo.

## Comunicazione PC–robot — da implementare

Il PC invia destinazioni e riceve feedback; il robot mantiene localmente
localizzazione, pianificazione, ciclo missione, controllo e sorveglianza motori.
Il trasporto resta da scegliere. Prevedere identificativo missione, validazione,
accettazione distinta dal completamento, gestione duplicati e sincronizzazione
alla riconnessione. Definire e verificare il comportamento alla perdita del link.
Serializzare gli eventi sul robot. I messaggi PC non sostituiscono conferme fisiche.

## Camera condivisa — da implementare

Una sola camera per suivi de ligne e detection degli incroci su linee bianche e
sfondo nero. Condividere acquisizione e segmentazione, separando errore laterale,
orientamento e visibilità della linea dai candidati incrocio e rami osservati.
Libreria, frequenze e soglie non sono ancora fissate.

Gestire avvicinamento, posizione di manovra, svolta, acquisizione del ramo corretto
e uscita. Filtrare su più frame e riarmare dopo l'uscita per evitare conteggi doppi.
Associare gli eventi al nodo atteso con distanza e rami osservati: incroci identici
non forniscono da soli un'identità assoluta. Gestire linea persa e incoerenze.
L'angolo encoder non dimostra la riacquisizione della linea; dopo un'inversione
evitare acquisizioni premature. La curva Start–Y richiede una procedura dedicata.

## Prove fisiche successive

Segmento con arresto → incrocio diritto/sinistra/destra → inversione e acquisizione
→ Start–incrocio–Start → Start–punto intermedio–Start.
Prima calibrare geometria/odometria e validare watchdog e arresto indipendente.
