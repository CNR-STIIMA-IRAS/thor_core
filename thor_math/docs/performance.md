# Prestazioni di `ThorQP`: ottimizzazioni e misure

Questo documento raccoglie le ottimizzazioni del codice fatte su `thor_math`,
il motivo di ciascuna, l'effetto misurato e come è stato verificato che la
soluzione non cambi. Raccoglie anche le misure di costo delle scelte
algoritmiche (formulazioni della CBF, ri-linearizzazione, robustezza) e delle
opzioni di build.

La sezione 3.1 di [api.md](api.md) ne è il riassunto in inglese.

---

## 1. Come sono state fatte le misure

| | |
|---|---|
| Scena | `examples/cbf_mpc_example.cpp`: UR10, 5 intervalli su 0.2 s, periodo 2 ms, 3000 cicli (6 s) |
| Ostacoli | 3 (corpo, mano, fixture); il corpo è una capsula dal commit `4919e7e` |
| Cosa si misura | solo `ThorQP::solve()`, con `std::chrono::steady_clock`, solve falliti compresi |
| Statistiche | media, mediana, p99 e massimo sui 3000 cicli; dove indicato, media su più run |
| Build | Release, Pinocchio di sistema (ROS Jazzy) salvo dove indicato |
| Macchina | AMD Ryzen 9 5900HX; il 30 ottobre kernel `6.8.1-1015-realtime`, governor `powersave` |

**I valori assoluti di giorni diversi non sono confrontabili** (§7). I
confronti affidabili sono quelli fatti nella stessa sessione, nelle stesse
condizioni, e in particolare quelli con le versioni alternate run per run.

Per la correttezza conta che ogni ottimizzazione lasci **la stessa
soluzione**: per ognuna è indicato il test che lo verifica.

---

## 2. Riepilogo

Tempo medio di `solve()` sulla scena dell'esempio (29 ottobre, stessa
sessione di misura per ogni riga):

| | discreta | continua | senza CBF | N = 4, β = 0.7, toll. 0.5 |
|---|---|---|---|---|
| prima delle ottimizzazioni | ~115 µs | ~83 µs | ~32 µs | ~142 µs |
| primo giro (§3.1–3.2) | ~77 µs | ~53 µs | ~19 µs | ~96 µs |
| secondo giro (§3.3–3.6) | **~58 µs** | **~36 µs** | **~9 µs** | **~72 µs** |
| guadagno totale | −50% | −57% | −72% | −49% |

Allocazioni sullo heap per `solve()`:

| | prima | dopo |
|---|---|---|
| discreta | 437 | **1** |
| continua | 131 | **1** |
| senza CBF | 35 | **1** |
| N = 4, β = 0.7, toll. 0.5 | 538 | **1** |

L'unica allocazione rimasta è `ThorOutput::next_acc`, che `solve()`
restituisce per valore; toglierla richiederebbe di cambiare l'API. Il
secondo giro non ha aggiunto allocazioni.

Il p99 è sceso con la media: nel caso N = 4 da 363 a 251 µs nel secondo
giro.

---

## 3. Ottimizzazioni del codice

### Profilo di partenza

Formulazione discreta, una linearizzazione, Release, prima di ogni modifica:

| fase | µs per solve | quota |
|---|---|---|
| solver QP (qpmad) | 52 | 46% |
| cinematica (Pinocchio, 6 nodi × 5 frame) | 25 | 22% |
| barriere e righe CBF | 22 | 19% |
| costruzione del costo | 7 | 6% |
| assemblaggio dei vincoli | 6 | 5% |
| **totale** | **~115** | |

Il QP ha 35 variabili (30 accelerazioni e 5 scaling) e ~325 vincoli, di cui
70 sono limiti su singole variabili.

**Eigen non è il collo di bottiglia.** A queste dimensioni il tempo va nelle
iterazioni dell'active-set, nella cinematica e, prima, nelle allocazioni, non
nella velocità dei prodotti matriciali. Librerie come MKL o BLAS rendono su
matrici grandi; qui l'overhead di chiamata annullerebbe il guadagno.

### 3.1 Limiti semplici passati a qpmad come tali (primo giro)

- **Prima.** I limiti `|u| ≤ DDqmax` e `min_scaling ≤ s ≤ max_scaling` erano
  70 righe della matrice dei vincoli generali, ciascuna con un solo ±1.
  qpmad le trattava come le altre: a ogni iterazione un prodotto scalare di
  35 elementi per riga, e un aggiornamento della fattorizzazione `O(n²)`
  quando una diventava attiva.
- **Dopo.** Passano come `lb ≤ x ≤ ub`, che qpmad gestisce nativamente con
  operazioni molto più economiche. `m_CI` mantiene il layout, con quelle
  colonne in testa, ma non vengono passate al solver.
- **Effetto** (prototipo, solo questa modifica e la successiva): qpmad da 52
  a 32 µs; totale discreta da 114 a 92 µs, continua da 84 a 59, senza CBF da
  ~30 a 24.
- **Verifica.** `SimpleBoundsGiveTheSameSolutionAsGeneralRows`: per 20 cicli,
  in entrambe le formulazioni, la soluzione coincide entro 1e-8 con quella di
  un qpmad indipendente che riceve il problema nel formato originale. Il test
  controlla anche che i limiti siano davvero saturati in alcuni cicli.

### 3.2 Nessuna allocazione nel ciclo (primo giro)

- **Prima.** A ogni ciclo si creava un nuovo `qpmad::Solver` e si
  allocavano la matrice dei vincoli (~255×35), una copia dell'Hessiano,
  vettori temporanei per ogni coppia e nodo e copie di `q`/`dq`.
  `frameVelocityDerivatives()` allocava due matrici 6×nv a ogni chiamata.
- **Dopo.** Solver e buffer sono membri (`Workspace`), dimensionati una volta
  in `configure()`/`updateMatrices()`. I prodotti usano `noalias()`, `q` e
  `dq` passano come viste, e `solve()` copia l'input solo quando deve
  completarlo con i default.
- **Effetto.** Da 437 a 1 allocazione per solve. Insieme al §3.1: discreta
  da 111 a 77 µs (−31%), continua da 83 a 53 (−36%), senza CBF da 32 a 19
  (−41%), N = 4 da 142 a 98 (−31%).
- **Perché conta oltre la media.** Un `malloc` nel loop di controllo ha
  durata variabile: toglierli rende il tempo di calcolo più deterministico.
- **Verifica.** Soluzioni identiche nei test d'integrazione e nell'esempio.
  Il conteggio delle allocazioni è stato misurato su una variante
  dell'esempio che intercetta `malloc`. Non c'è un test automatico, perché
  dipenderebbe da glibc.

**Bug trovato nel frattempo.** `computeActualMatrices()` sommava
`λ_jerk JᵀJ` all'Hessiano a ogni ciclo senza azzerarlo, quindi con
`lambda_jerk > 0` il peso del jerk cresceva nel tempo: dopo 5 cicli con input
identici la differenza aveva norma 25.5. Il termine è costante e ora sta
nell'Hessiano fisso. Un test di regressione verifica che il costo non cambi
tra cicli con gli stessi input.

### Profilo dopo il primo giro

Formulazione discreta: righe CBF ~33 µs, di cui ~14 per le righe `G` della
barriera linearizzata; qpmad ~27 µs, di cui ~7 per fattorizzare e invertire
l'Hessiano. Circa l'88% dei vincoli generali era già implicato dai limiti.
Da qui le quattro modifiche del secondo giro.

### 3.3 Hessiano fattorizzato a blocchi (secondo giro)

- **Osservazione.** Il blocco 30×30 dell'Hessiano sulle accelerazioni è
  costante; cambiano a ogni ciclo solo le righe dei 5 scaling, che dipendono
  dalla velocità target.
- **Dopo.** `updateMatrices()` calcola una volta il fattore di Cholesky
  inverso del blocco costante. A ogni ciclo si completa il fattore con il
  complemento di Schur degli scaling (5×5) e lo si passa a qpmad come
  `HESSIAN_INVERTED_CHOLESKY_FACTOR`. Con più linearizzazioni per ciclo il
  fattore si calcola una volta sola.
- **Sicurezza.** Se una classe derivata cambia il blocco costante, `solve()`
  torna da sola alla fattorizzazione completa.
- **Verifica.** Un test controlla che il fattore a blocchi soddisfi
  `U Uᵀ = H⁻¹`.

### 3.4 Vincoli implicati dai limiti esclusi (secondo giro)

- **Osservazione.** Una riga `CIᵀx + ci0 ≥ 0` il cui minimo su
  `lb ≤ x ≤ ub` è già ≥ 0 non può essere violata da nessuna soluzione
  ammissibile. Sulla scena dell'esempio sono ~224 delle 255 righe generali.
- **Dopo.** Queste righe non vengono passate a qpmad. Il minimo sulla scatola
  si precalcola per le righe costanti e si ricalcola a ogni linearizzazione
  per le righe CBF. `m_CI` e `m_ci0` mantengono il loro layout.
- **Perché non cambia la soluzione.** L'insieme ammissibile è lo stesso,
  quindi è lo stesso anche l'ottimo di un QP strettamente convesso.
- **Verifica.** Il test del §3.3, esteso, controlla che le righe siano
  davvero escluse; soluzioni identiche in tutti i test.

### 3.5 Righe CBF solo sulle colonne non nulle (secondo giro)

- **Osservazione.** Lo stato a fine intervallo `i` dipende solo dagli
  ingressi `u_0 … u_i`, quindi le righe `G` della formulazione discreta sono
  nulle sulle colonne successive.
- **Dopo.** Si calcolano solo le colonne non nulle. La struttura si ricava
  dalle risposte forzate in `updateMatrices()`, invece di darla per
  scontata.

### 3.6 qpmad non rialloca (secondo giro)

- **Problema.** Con il §3.4 il numero di righe passate a qpmad cambia da un
  ciclo all'altro, e qpmad riallocava due vettori interni a ogni cambio: si
  sarebbe persa la proprietà "zero allocazioni".
- **Dopo.** Nella copia inclusa di qpmad i buffer crescono ma non si
  riducono. Le modifiche sono marcate `thor_math:` in
  `include/qpmad/implementation.h` e annotate in `third_party/qpmad/VERSION`,
  come richiede la licenza Apache-2.0. `ThorQP` li riserva per il problema
  più grande.
- **Verifica.** Un test confronta un solver riusato con uno nuovo su
  problemi di dimensione variabile. La prima versione della modifica faceva
  fallire 9 test; il test dedicato rileva l'errore se la modifica viene
  tolta (verificato introducendolo di proposito). Tutti i test passano anche
  con AddressSanitizer e UndefinedBehaviorSanitizer.

### Effetto del secondo giro

Media su 5 run, Pinocchio di sistema, stessa sessione:

| | prima del secondo giro | dopo | |
|---|---|---|---|
| discreta | 77 µs | **58 µs** | −24% |
| continua | 52 µs | **36 µs** | −32% |
| senza CBF | 19 µs | **9 µs** | −54% |
| N = 4, β = 0.7, toll. 0.5 | 96 µs | **72 µs** | −26% |

### 3.7 Disattivazione delle righe CBF

Se la barriera più bassa sull'orizzonte supera
`cbf_activation_distance`, le righe CBF non entrano nel QP: la soluzione è la
stessa e il QP è più piccolo. Con il §3.4, anche le righe CBF singole già
implicate dai limiti vengono escluse.

---

## 4. Costo delle scelte algoritmiche

Queste non sono ottimizzazioni ma scelte di formulazione, con il loro costo
misurato sulla stessa scena. I tempi sono quelli della sessione in cui sono
state introdotte, cioè **prima** delle ottimizzazioni del §3: oggi sono
tutti più bassi, ma i rapporti restano indicativi.

### 4.1 Formulazione della CBF

| | `h` minimo lungo il piano | cicli con `h < −1 cm` | clearance reale (mano / fixture) | solve medio |
|---|---|---|---|---|
| continua | −1.19 m | 254 | 0.355 / 0.064 m | ~80 µs |
| discreta | −0.04 m | 53 | 0.356 / 0.064 m | ~110 µs |

La discreta costa ~30 µs in più (una passata di cinematica in più per nodo e
righe dense in `u`) e rende il piano coerente. Il moto eseguito era sicuro in
entrambi i casi.

### 4.2 Ri-linearizzazione

| N | β | tolleranza | `h` minimo lungo il piano | cicli < −1 cm | QP medi per ciclo | solve medio / max |
|---|---|---|---|---|---|---|
| 1 | – | – | −0.040 m | 53 | 1.00 | 114 / 210 µs |
| 2 | 1 | 0 | −0.027 m | 6 | 2.00 | 236 / 680 µs |
| 6 | 1 | 0 | −0.027 m | 50 | 5.16 | 560 / 1210 µs |
| 6 | 0.7 | 0 | −0.010 m | 0 | 6.00 | 643 / 1030 µs |
| **4** | **0.7** | **0.5** | **−0.001 m** | **0** | **1.25** | **144 / 635 µs** |

- Con il passo pieno (β = 1) l'iterazione oscilla: N = 6 è peggio di N = 2.
- L'arresto anticipato rende la ri-linearizzazione economica in media
  (1.25 QP per ciclo), ma **il caso peggiore resta N QP**: in tempo reale va
  dimensionato su quello.
- Il moto eseguito non cambia: questi parametri migliorano la coerenza del
  piano, non la sicurezza del moto reale.
- I conteggi sono piccoli (cicli su 3000) e vengono da una sola scena.

### 4.3 Robustezza alla percezione (30 ottobre)

`obstacle_sigma`, `C_escape` e `rate_bound`
([robust_cbf_method.md](robust_cbf_method.md)) aggiungono al calcolo di ogni
coppia solo operazioni scalari e qualche prodotto 3D: i bound su ψ, lo
spostamento delle velocità lungo `u` e la formula analitica di `rate_bound`.
Niente cinematica in più, niente differenze finite, nessuna allocazione.

- **Con i σ a zero** l'output dell'esempio è identico, cifra per cifra, a
  quello della versione precedente.
- **Tempo con i σ a zero:** nessuna differenza misurabile. In tre coppie di
  run consecutive la versione precedente ha dato 83.5, 84.6 e 72.7 µs, quella
  nuova 87.3, 79.7 e 82.6 µs.
- **Tempo con i σ diversi da zero:** non misurato. L'esempio non ha
  un'opzione per attivarli.

---

## 5. Opzioni di build

Misurate sulla stessa scena con Pinocchio 3.3.1 compilato dai sorgenti
(riduzione del tempo medio, discreta / continua / senza CBF). Nessuna cambia
la soluzione:

| opzione | guadagno |
|---|---|
| `-march=x86-64-v3` (AVX2/FMA) su Pinocchio e `thor_math` | 5 / 8 / 10% |
| `-march=x86-64-v3 -fno-semantic-interposition -fno-plt` | 6 / 8 / 9% |
| `-march=x86-64-v3`, Pinocchio senza istanze precompilate | 1 / 10 / 10% |
| `-march=x86-64-v3` con LTO su `thor_math` | 3 / 5 / 3% |
| `-march=x86-64-v3 -fno-math-errno -fno-trapping-math` | 3 / 4 / 2% |

- **Rendono poco** rispetto alle modifiche del codice, e da sole non
  bastano.
- **SIMD richiede flag coerenti.** Il Pinocchio dei pacchetti ROS è compilato
  per x86-64 generico. Compilare `thor_math` con `-march=native` contro quel
  Pinocchio rende i due binari incompatibili (Eigen cambia il layout degli
  oggetti) e l'esempio va in segmentation fault, anche con
  `EIGEN_MAX_ALIGN_BYTES=16`. Pinocchio, `thor_math` e l'applicazione devono
  avere gli stessi flag, e nel processo deve esserci un solo Pinocchio.
- **`-march=native` contro `x86-64-v3`.** `native` lega il binario alla CPU
  che compila; `x86-64-v3` (Intel da Haswell, AMD da Zen) è più portabile.
- **Pinocchio senza istanze precompilate**
  (`-DENABLE_TEMPLATE_INSTANTIATION=OFF`) si compila in ~20 s invece di
  ~3.5 min con 3 job, senza perdere velocità. È il default di
  `scripts/build_thor.sh`.

`scripts/build_thor.sh` compila Pinocchio e `thor_math` nello stesso
prefisso con flag coerenti (`--arch`, `--native`, `--extra-flags`), e
supporta anche la cross-compilazione (`--toolchain`).

---

## 6. Caso peggiore e tempo reale

Fotografia del 30 ottobre, codice attuale, 5 run per configurazione:

| | media | mediana | p99 | massimo | solve oltre 2 ms |
|---|---|---|---|---|---|
| discreta | 79 µs | 77 µs | 121 µs | 430 µs | 0 |
| continua | 50 µs | 48 µs | 93 µs | 234 µs | 0 |
| senza CBF | 12 µs | 12 µs | 18 µs | 204 µs | 0 |
| N = 4, β = 0.7, toll. 0.5 | 96 µs | 78 µs | 342 µs | 730 µs | 0 |

(Media, mediana e p99 sono medie sulle 5 run; il massimo è il massimo
assoluto.)

- **Nessun solve ha superato il periodo di 2 ms.** Il caso peggiore misurato,
  730 µs con N = 4, è circa un terzo del ciclo.
- **I massimi cadono soprattutto sul primo ciclo** (cache fredde) e su picchi
  sporadici dovuti allo scheduling. Con la CBF attiva i picchi arrivano
  anche quando l'ostacolo è vicino e il solver active-set fa più iterazioni.
- **Per un'applicazione reale contano p99 e massimo più della media.**
  Servono un ciclo di warm-up prima di partire, nessuna allocazione nel loop
  (garantita dal §3.2), un kernel `PREEMPT_RT`, la memoria bloccata
  (`mlockall`) e un core dedicato.

---

## 7. Variabilità delle misure

Il 30 ottobre lo stesso binario ha dato 79 µs di media in una serie e circa
114 µs in un'altra, a pochi minuti di distanza, con il governor della CPU in
`powersave`. Per capire se la differenza rispetto ai 58 µs del 29 ottobre
fosse una regressione, sono state misurate tre versioni alternandole run per
run, così che la deriva della macchina pesasse su tutte allo stesso modo
(formulazione discreta, 6 run ciascuna):

| versione | media | p99 |
|---|---|---|
| `dfc070a` (fine secondo giro di ottimizzazioni) | 114 µs | 204 µs |
| `4919e7e` (capsule) | 109 µs | 186 µs |
| attuale | 114 µs | 189 µs |

**Le tre versioni sono equivalenti entro il rumore:** non c'è regressione. La
differenza con i 58 µs del 29 ottobre dipende dalle condizioni della
macchina, non dal codice.

**Protocollo consigliato per le misure future:**

1. governor `performance` (`cpupower frequency-set -g performance`) e, se
   possibile, frequenza fissa;
2. processo su un core isolato (`taskset`, `isolcpus`);
3. versioni da confrontare **alternate** run per run, almeno 5 run ciascuna;
4. riportare media, p99 e massimo, e le condizioni della macchina;
5. build Release; l'esempio avvisa se è compilato senza ottimizzazioni.

**Build senza ottimizzazioni.** Con `CMAKE_BUILD_TYPE` vuoto (il default di
colcon) `solve()` sull'esempio impiega 16.1 ms di media contro 0.106 ms in
Release, circa 150 volte di più: tutti i 3000 solve superano i 2 ms. Nella
demo ROS questo si vede come overrun di 32–38 ms per ciclo. Il
`controller_manager` stampa gli overrun con throttle a 1 s, quindi nel log
sembrano periodici anche se riguardano ogni ciclo. Da ora `thor_math` e
`thor_controllers` usano `Release` quando non è indicato un tipo di build.

---

## 8. Cosa non è stato fatto

| Idea | Stato | Note |
|---|---|---|
| Saltare cinematica e barriere delle coppie lontane da ogni ostacolo | non fatto | con molti ostacoli o nuvole di punti è il guadagno principale; sulla scena dell'esempio modesto |
| Warm start dell'active set | non fatto | qpmad non ha hot start; qpOASES sì |
| Altri solver (HPIPM, ProxQP, qpOASES) | non fatto | richiede un benchmark sulla stessa scena; con 35 variabili un active-set denso è già una scelta sensata |
| Parallelizzazione sui nodi | sconsigliata | a questa scala l'overhead dei thread è dello stesso ordine del lavoro e peggiora il determinismo |
| Test automatico sul numero di allocazioni | non fatto | richiederebbe di intercettare `malloc` in modo specifico per glibc |
| Profilo per fasi dopo il secondo giro | non fatto | i tempi delle singole fasi sul codice attuale non sono stati misurati |
| Misura con `obstacle_sigma` diversi da zero | non fatto | l'esempio non ha un'opzione per attivarli |
