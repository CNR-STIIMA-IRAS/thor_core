# Modello SSM con control barrier function

Questo documento descrive il modello di riferimento della barriera di
sicurezza di `thor_math` e dello stop protettivo di `thor_controllers`, nei
termini dello *speed and separation monitoring* (SSM) di ISO/TS 15066 e
ISO 10218-2. Copre:

- la formula SSM della norma e i suoi termini;
- la manovra di arresto e la separazione minima prevista;
- la distanza di protezione e la barriera;
- la robustezza all'incertezza di percezione e del modello dell'ostacolo;
- la condizione CBF nel QP;
- lo stop protettivo e la ripresa;
- il termine di escape, opzionale e fuori dall'SSM;
- la scelta dei parametri, con i richiami normativi.

**Stato.** È il modello di riferimento per le prossime modifiche. Il codice
attuale implementa la versione descritta in
[robust_cbf_method.md](robust_cbf_method.md), con la vecchia nomenclatura. Le
differenze da colmare sono elencate nel §12, la corrispondenza dei simboli nel
§13.

**Posizione rispetto alla norma.** Il modello non replica la formula della
norma alla lettera: la generalizza. La norma prende velocità di caso peggiore
fisse perché presuppone un sistema che misura posizioni e non può fidarsi
delle velocità che ne ricava. Qui le velocità si usano, con un'incertezza
dichiarata e valutata nel caso peggiore. La norma ne è il caso limite (§5.3).

**Sicurezza funzionale.** THOR calcola la distanza di protezione e attiva lo
stop, ma non è una funzione di sicurezza certificata (ISO 13849-1). In una
cella SSM il rilevamento e lo stop devono essere eseguiti da parti di
sicurezza certificate (§8.4). Il documento descrive un modello coerente con
l'SSM: la conformità dipende dalla catena di sicurezza dell'impianto.

---

## Indice

1. [Riferimenti](#1-riferimenti)
2. [SSM nella norma](#2-ssm-nella-norma)
3. [Nomenclatura](#3-nomenclatura)
4. [Manovra di arresto e separazione minima prevista](#4-manovra-di-arresto-e-separazione-minima-prevista)
5. [Distanza di protezione e barriera](#5-distanza-di-protezione-e-barriera)
6. [Incertezza di percezione e del modello](#6-incertezza-di-percezione-e-del-modello)
7. [Condizione CBF nel QP](#7-condizione-cbf-nel-qp)
8. [Stop protettivo e ripresa](#8-stop-protettivo-e-ripresa)
9. [Termine di escape (opzione fuori SSM)](#9-termine-di-escape-opzione-fuori-ssm)
10. [Scelta dei parametri](#10-scelta-dei-parametri)
11. [Ipotesi e limiti](#11-ipotesi-e-limiti)
12. [Differenze rispetto al codice attuale](#12-differenze-rispetto-al-codice-attuale)
13. [Corrispondenza con la nomenclatura precedente](#13-corrispondenza-con-la-nomenclatura-precedente)

---

## 1. Riferimenti

| Sigla | Documento |
|---|---|
| [10218-1] | ISO 10218-1:2025, *Robotics — Safety requirements — Part 1: Industrial robots* (dati di arresto, Annex B nell'edizione 2011) |
| [10218-2] | ISO 10218-2:2025, *Part 2: Industrial robot applications and robot cells* (SSM: §5.14.5 e Annex L) |
| [15066] | ISO/TS 15066:2016, *Robots and robotic devices — Collaborative robots* (SSM, ora integrata in [10218-2]) |
| [13855] | ISO 13855, *Positioning of safeguards with respect to the approach speeds of parts of the human body* |
| [60204] | IEC 60204-1, *Electrical equipment of machines* (categorie di stop) |
| [13849] | ISO 13849-1, *Safety-related parts of control systems* |
| [61496] | IEC 61496, *Electro-sensitive protective equipment*; IEC/TS 62046 per il rilevamento di persone |
| [9283] | ISO 9283, *Manipulating industrial robots — Performance criteria* (incertezza di posizione del robot) |
| [MN17] | J. A. Marvel, R. Norcross, *Implementing speed and separation monitoring in collaborative robot workcells*, Robotics and Computer-Integrated Manufacturing 44 (2017) 144–155, doi:10.1016/j.rcim.2016.08.001 |

Le formule normative sono riportate come in [MN17] (per [15066]) e come
riassunte per [10218-2]:2025 da fonti secondarie. Vanno verificate sul testo
delle norme prima di usarle in un risk assessment.

---

## 2. SSM nella norma

### 2.1 Distanza di protezione

La separazione `S` tra il robot e l'operatore deve restare almeno pari alla
distanza di protezione `S_p` ([15066], eq. (1) di [MN17]):

$$
S(t_0) \;\ge\; S_p(t_0) =
\underbrace{\int_{t_0}^{t_0+T_R+T_S} v_H(\tau)\,d\tau}_{S_h}
+ \underbrace{\int_{t_0}^{t_0+T_R} v_R(\tau)\,d\tau}_{S_r}
+ \underbrace{\int_{t_0+T_R}^{t_0+T_R+T_S} v_S(\tau)\,d\tau}_{S_s}
+ C + Z_d + Z_r .
$$

| Termine | Significato |
|---|---|
| `S_h` | spazio percorso dall'operatore verso il robot finché il robot non è fermo |
| `S_r` | spazio percorso dal robot verso l'operatore durante il tempo di reazione `T_R` |
| `S_s` | spazio percorso dal robot durante la frenata, di durata `T_S` |
| `C` | *intrusion distance*: quanto una parte del corpo può penetrare oltre la zona di rilevamento ([13855]) |
| `Z_d` | incertezza di posizione dell'operatore dovuta al sensore (`Z_S` in [15066] e [MN17]) |
| `Z_r` | incertezza di posizione del robot |

`v_H`, `v_R` e `v_S` sono *directed speeds*: velocità nella direzione
dell'altro. `T_S` dipende dalla velocità e dal carico del robot e si ricava
dai dati di arresto di [10218-1]. La forma lineare
`S_p = v_H(T_R + T_S) + v_R T_R + B + C + Z_d + Z_r`, con `B` la distanza di
frenata, è un'approssimazione della forma integrale ([MN17], eq. (3)).

Se `v_H` non è misurata, la norma la prende costante e diretta verso il robot:
2.0 m/s, oppure 1.6 m/s se la distanza supera 500 mm ([13855]). La norma
ammette di misurarla direttamente e di usare profili di accelerazione più
realistici ([MN17], §3).

### 2.2 Comportamento sotto la soglia

Se `S < S_p`, il sistema riduce la velocità o avvia uno stop protettivo
([10218-2] §5.14.5). In [15066] lo stop è uno stop controllato (*safety-rated
controlled stop*), cioè di categoria 1 o 2 secondo [60204]:

| Categoria | Controllato | Alimentazione rimossa | Freni |
|---|---|---|---|
| 0 | no | sì | sì |
| 1 | sì | sì, a robot fermo | sì |
| 2 | sì | no | no |

Dopo uno stop di categoria 2 il robot resta in fermo monitorato (*monitored
standstill*): se si muove oltre i limiti, scatta uno stop di categoria 0 o 1.

### 2.3 Ripresa

La ripresa automatica è ammessa solo quando la separazione richiesta è
ristabilita e nessuna persona resta nello spazio protetto dall'SSM
([10218-2]). La norma non prevede che il robot si allontani mentre la
separazione è sotto `S_p` ([MN17] non tratta il caso).

---

## 3. Nomenclatura

### 3.1 Geometria e stato della coppia

Per ogni coppia (sfera `j` del robot, ostacolo `k`):

| Simbolo | Significato | Unità |
|---|---|---|
| `p_r, v_r, a_r` | centro della sfera del robot, velocità, accelerazione | m, m/s, m/s² |
| `p_o, v_o, a_o` | centro dell'ostacolo (o punto più vicino dell'asse di una capsula), velocità, accelerazione | m, m/s, m/s² |
| `D = ‖p_r − p_o‖` | distanza tra i centri | m |
| `u = (p_r − p_o)/D` | direzione dall'ostacolo al robot | – |
| `P = I − u uᵀ` | proiettore sul piano normale a `u` | – |
| `ρ` | somma dei raggi | m |
| `S = D − ρ` | separazione attuale tra le superfici | m |
| `v_r∥ = u·v_r` | velocità del robot lungo `u` (positiva: si allontana) | m/s |
| `v_o∥ = u·v_o` | velocità dell'ostacolo lungo `u` (positiva: si avvicina) | m/s |
| `a_o∥ = u·a_o` | accelerazione dell'ostacolo lungo `u` (positiva: verso il robot) | m/s² |
| `ψ = [S, v_r∥, v_o∥, a_o∥]` | stato della coppia | – |

Convenzione di segno: `Ṡ = v_r∥ − v_o∥`, negativa quando la coppia si avvicina.

### 3.2 Manovra di arresto

| Simbolo | Significato | Unità |
|---|---|---|
| `T_r` | tempo di reazione: dalla misura all'inizio della frenata (`T_R` della norma) | s |
| `T_S` | durata della frenata dello stop, funzione della velocità articolare (§4.1) | s |
| `T_f = T_r + T_S` | fine della manovra: robot fermo | s |
| `φ(τ), Φ(τ)` | profilo di velocità normalizzato della manovra e suo integrale (§4.1) | –, s |
| `S(τ)` | separazione prevista all'istante `τ` della manovra, `S(0) = S` | m |
| `S_min` | separazione minima prevista, `min_{τ ∈ [0, T_f]} S(τ)` | m |
| `τ*` | istante del minimo | s |

### 3.3 Distanza di protezione

| Simbolo | Significato | Unità |
|---|---|---|
| `C` | intrusion distance ([13855]) | m |
| `Z_d` | incertezza di posizione dell'ostacolo dovuta al sensore; nella forma costante coincide con `δ_H`, nella forma dipendente dallo stato è la scatola dei σ (§6) | m |
| `Z_r` | incertezza di posizione del robot ([9283]) | m |
| `Z_acc` | incertezza residua del modello di accelerazione dell'ostacolo, non coperta da `Δa_max` | m |
| `Z_ctrl` | incertezza legata alle prestazioni del controllo (ciclo perso) | m |
| `S_p,0 = C + Z_r + Z_acc + Z_ctrl` | parte statica della distanza di protezione | m |
| `S_p` | distanza di protezione, dipendente dallo stato (§5.1) | m |

### 3.4 Incertezza

| Simbolo | Significato | Unità |
|---|---|---|
| `σ_p, σ_v, σ_a` | deviazioni standard dello stimatore di posizione, velocità e accelerazione dell'ostacolo | m, m/s, m/s² |
| `k` | numero di deviazioni standard della scatola (`sigma_bound`) | – |
| `b_p, b_v, b_a` | semiampiezze della scatola sullo stato 3D dell'ostacolo | m, m/s, m/s² |
| `b_S, b_vr, b_vo, b_ao` | semiampiezze della scatola su ψ (Lemma 6.1) | m, m/s, m/s, m/s² |
| `Δa_max` | massima variazione dell'accelerazione dell'ostacolo lungo `u`, rispetto alla stima, entro la manovra | m/s² |
| `j_max` | limite del jerk dell'ostacolo lungo `u` nel periodo di controllo (solo formulazione continua, §6.6) | m/s³ |
| `ψ_w` | angolo peggiore della scatola (Proposizione 6.4) | – |

### 3.5 Controllo ed escape

| Simbolo | Significato | Unità |
|---|---|---|
| `h` | barriera | m |
| `γ` | guadagno della funzione di classe K | 1/s |
| `Δ_s = S_min − S_p,0` | scarto dalla distanza di protezione (= `S − S_p`) | m |
| `S_e` | distanza di fuga del termine di escape (§9) | m |
| `v_e = S_e/T_r` | velocità di fuga | m/s |
| `H(x)` | funzione di Heaviside, `H(x) = 1` per `x > 0`, `0` altrimenti | – |

---

## 4. Manovra di arresto e separazione minima prevista

### 4.1 Manovra di arresto

`S_min` risponde a una domanda: *se adesso scattasse lo stop protettivo,
quanto si avvicinerebbero al minimo robot e ostacolo prima che il robot sia
fermo?* La manovra considerata è lo stop che il sistema esegue davvero.

Lo stop di `ThorCore` frena tutti i giunti in modo sincronizzato, ciascuno al
più al proprio limite di accelerazione:

$$
T_S(\dot q) = \max_j \frac{\lvert \dot q_j \rvert}{\ddot q_{\max,j}},
\qquad
\dot q(t) = \dot q_0 \left(1 - \frac{t}{T_S}\right), \quad 0 \le t \le T_S .
$$

Tutti i giunti, e quindi tutti i punti del robot, si fermano nello stesso
istante. La velocità di ogni punto si riduce in proporzione:
$v_r(t) = J(q(t))\,\dot q(t) \approx v_r\,(1 - t/T_S)$ al primo ordine nella
variazione di `J` durante la frenata.

**Ipotesi (A-stop).** Durante la manovra la velocità del punto del robot è
$v_r\,\varphi(\tau)$, con

$$
\varphi(\tau) =
\begin{cases}
1 & 0 \le \tau \le T_r \quad \text{(reazione)}\\[2pt]
1 - \dfrac{\tau - T_r}{T_S} & T_r < \tau \le T_f \quad \text{(frenata)}
\end{cases}
\qquad
\Phi(\tau) = \int_0^\tau \varphi(s)\,ds =
\begin{cases}
\tau & \tau \le T_r\\[2pt]
\tau - \dfrac{(\tau - T_r)^2}{2\,T_S} & T_r < \tau \le T_f
\end{cases}
$$

e `T_S` è la durata della frenata dello stop eseguito. Con il robot fermo
(`T_S = 0`) la finestra si riduce a `[0, T_r]`.

Due conseguenze da notare:

- **La frenata vale anche per un robot che si allontana.** Lo stop frena tutto
  il moto, compresa la componente di allontanamento. Mentre il robot rallenta,
  l'ostacolo che lo segue gli si avvicina.
- **La decelerazione lungo `u` non è una costante.** Vale
  $v_{r\parallel}/T_S$, ed è piccola per un punto che si muove soprattutto in
  direzione tangenziale. Una decelerazione lungo `u` fissa sovrastimerebbe la
  frenata del componente di avvicinamento.

### 4.2 Separazione prevista

L'ostacolo mantiene l'accelerazione `a_o∥` (il §6.4 estende il risultato ad
accelerazioni variabili). Lungo la direzione `u` dell'istante iniziale:

$$
S(\tau) = S + v_{r\parallel}\,\Phi(\tau) - v_{o\parallel}\,\tau - \tfrac12\,a_{o\parallel}\,\tau^2,
\qquad 0 \le \tau \le T_f ,
$$

$$
S_{\min}(\psi) = \min_{\tau \in [0, T_f]} S(\tau).
$$

`S(τ)` è continua con derivata continua, perché `φ` è continua in `T_r`:

$$
\dot S(\tau) = v_{r\parallel}\,\varphi(\tau) - v_{o\parallel} - a_{o\parallel}\,\tau .
$$

**Lemma 4.1 (minorante).** Siano `Δp_r(τ)` e `Δp_o(τ)` gli spostamenti veri,
in 3D, del robot e dell'ostacolo, e supponiamo che le loro componenti lungo la
`u` iniziale seguano il modello. Allora la distanza vera tra i centri soddisfa
$D(\tau) \ge D + u \cdot (\Delta p_r(\tau) - \Delta p_o(\tau))$, quindi la
separazione vera è almeno `S(τ)`.

*Dimostrazione.* $D(\tau) = \lVert r + \Delta p_r - \Delta p_o \rVert$ con
$r = D u$, e $\lVert x \rVert \ge u \cdot x$ per ogni `x` (Cauchy–Schwarz). ∎

Il moto tangenziale e la rotazione di `u` durante la manovra sono quindi
trascurati in modo conservativo. Questo non riguarda la rotazione di `u` nel
ciclo di controllo, che entra nelle derivate di Lie (§7.1).

### 4.3 Candidati del minimo

`S(τ)` è un polinomio di secondo grado in ciascuna fase, quindi il minimo è
agli estremi di una fase o in un punto stazionario interno:

| Candidato | Valore | Quando |
|---|---|---|
| `0` | – | sempre |
| `T_r` | – | sempre |
| `T_f` | `T_r + T_S` | `T_S > 0` |
| `τ₁` | $\dfrac{v_{r\parallel} - v_{o\parallel}}{a_{o\parallel}}$ | `a_o∥ ≠ 0`, `τ₁ ∈ (0, T_r)` |
| `τ₂` | $\dfrac{v_{r\parallel}\,(1 + T_r/T_S) - v_{o\parallel}}{a_{o\parallel} + v_{r\parallel}/T_S}$ | `T_S > 0`, denominatore non nullo, `τ₂ ∈ (T_r, T_f)` |

`τ₂` annulla $\dot S$ nella fase di frenata. Il calcolo è in tempo costante.

### 4.4 Gradiente

La finestra `[0, T_f]` non dipende da ψ: dipende solo da `q̇`, che il robot
conosce esattamente. Per il teorema dell'inviluppo:

$$
\frac{\partial S_{\min}}{\partial S} = 1,
\qquad
\frac{\partial S_{\min}}{\partial v_{r\parallel}} = \Phi(\tau^*) \ge 0,
\qquad
\frac{\partial S_{\min}}{\partial v_{o\parallel}} = -\tau^*,
\qquad
\frac{\partial S_{\min}}{\partial a_{o\parallel}} = -\frac{\tau^{*2}}{2}.
$$

Rispetto a `T_S`, che dipende dall'ingresso tramite `q̇`:

$$
\frac{\partial S_{\min}}{\partial T_S} =
v_{r\parallel}\,\frac{\partial \Phi}{\partial T_S}(\tau^*) + [\tau^* = T_f]\,\dot S(T_f),
\qquad
\frac{\partial \Phi}{\partial T_S}(\tau) = \frac{(\tau - T_r)^2}{2\,T_S^2}\;\;(\tau > T_r),
$$

dove il secondo termine compare solo se il minimo è all'estremo mobile `T_f`.
Nei punti in cui due candidati danno lo stesso minimo, `S_min` non è
differenziabile; si usa il gradiente di uno dei candidati attivi.

---

## 5. Distanza di protezione e barriera

### 5.1 Definizione

$$
S_p(\psi) = \underbrace{\big(S - S_{\min}(\psi)\big)}_{S_h + S_r + S_s} + S_{p,0},
\qquad
S_{p,0} = C + Z_r + Z_{acc} + Z_{ctrl},
$$

$$
h(\psi) = S - S_p = S_{\min}(\psi) - S_{p,0} = \Delta_s .
$$

Il safe set $\{h \ge 0\}$ coincide con la condizione della norma $S \ge S_p$.
Poiché $\tau = 0$ è sempre un candidato, $S - S_{\min} \ge 0$ e
$S_p \ge S_{p,0}$: il modello non scende mai sotto i termini statici. `Z_d`
non compare in $S_{p,0}$, perché entra calcolando `S` e `S_p` nell'angolo
della scatola (§6). Sommarlo anche qui conterebbe due volte lo stesso rumore.

### 5.2 Corrispondenza con l'eq. (1) della norma

Se la coppia si avvicina per tutta la manovra ($\dot S \le 0$ su `[0, T_f]`),
il minimo è in `T_f` e

$$
S - S_{\min} = \int_0^{T_f} v_{o\parallel}(\tau)\,d\tau
+ \int_0^{T_r} (-v_{r\parallel})\,d\tau
+ \int_{T_r}^{T_f} (-v_{r\parallel}\,\varphi(\tau))\,d\tau ,
$$

termine per termine `S_h + S_r + S_s` dell'eq. (1), con le velocità
direzionali lungo `u` e il profilo di frenata (A-stop).

### 5.3 In cosa il modello generalizza la norma

| | Norma | Modello |
|---|---|---|
| Velocità del robot | modulo, `≥ 0`: un robot che si allontana non riduce `S_p` | con segno lungo `u`: l'allontanamento compensa l'avvicinamento dell'ostacolo |
| Velocità dell'ostacolo | `v_H` di caso peggiore fissa, oppure misurata | stimata, con incertezza dichiarata nell'angolo peggiore e `Δa_max` (§6) |
| Forma | spazio percorso fino all'arresto | minimo nel tempo della separazione prevista |
| Direzione | *directed speed*, senza la direzione reale del moto | proiezione lungo `u` per ogni coppia di sfere |

Le motivazioni:

1. **Credito al robot che si allontana.** La sfiducia della norma nelle
   velocità nasce dalla misura: ricavare una velocità da posizioni rumorose
   produce errori di centinaia di mm/s anche con pochi millimetri di rumore
   ([MN17], §3 e Tab. 2). La velocità del robot invece viene da encoder e
   cinematica (ipotesi A2), quindi il credito è giustificato. Con velocità
   relativa nulla il robot non deve fermarsi: `S_p` contiene solo
   l'avvicinamento durante la frenata (§4.1).
2. **Velocità dell'ostacolo con incertezza esplicita.** La norma è il caso
   limite in cui la velocità misurata non vale nulla. Se la scatola soddisfa
   $\hat v_{o\parallel} + b_{vo} = v_H$, con accelerazione nulla e
   $\Delta a_{\max} = 0$, l'angolo restituisce lo `S_h` della norma con
   velocità costante. Con uno stimatore validato la scatola si
   stringe e `S_p` si riduce, senza perdere la garanzia.
3. **Minimo nel tempo.** Coincide con la forma della norma se la coppia si
   avvicina per tutta la manovra. Se l'ostacolo decelera o inverte il moto, il
   minimo cade prima della fine e la somma dei tratti sottostimerebbe
   l'avvicinamento.
4. **Direzione reale.** In [MN17], Fig. 1, un operatore fermo fa scattare lo
   stop dopo che il robot lo ha già superato, perché `S_p` cresce con il
   modulo della velocità del robot. Con la proiezione lungo `u` un robot che
   si allontana non fa crescere `S_p`. [MN17] (§3) suggerisce proprio di
   misurare la variazione della distanza relativa per evitare stop inutili.
5. **Tutti i punti del robot.** [MN17] raccomanda distanze di protezione
   indipendenti per tutti i giunti, includendo utensili e pezzi tenuti dal
   robot. Il modello lavora per coppie di sfere: utensile e pezzo devono
   essere coperti da sfere del robot (§10.3).

### 5.4 Esempio: robot che si allontana

`S = 1 m`. Il robot si allontana a 1 m/s lungo `u` e la persona si avvicina a
1 m/s; `T_r = 0.15 s`, `T_S = 0.2 s`.

| `τ` [s] | Fase | `v_r∥ φ(τ)` | Spostamento del robot | Spostamento della persona | `S(τ)` |
|---|---|---|---|---|---|
| 0 | inizio | 1.0 m/s | 0 | 0 | 1.000 m |
| 0.15 | fine reazione | 1.0 m/s | +0.150 m | −0.150 m | 1.000 m |
| 0.25 | frenata | 0.5 m/s | +0.225 m | −0.250 m | 0.975 m |
| 0.35 | robot fermo | 0 | +0.250 m | −0.350 m | 0.900 m |

Il modello dà `S − S_min = 0.10 m`. La norma senza compensazione darebbe
`S_h = 1 · 0.35 = 0.35 m`; un modello che ignora la frenata del robot che si
allontana darebbe 0, sbagliando dalla parte non sicura.

---

## 6. Incertezza di percezione e del modello

### 6.1 Ipotesi

- **(A1) Errore di misura limitato.** Lo stato vero dell'ostacolo dista da
  quello stimato al più `b_p`, `b_v`, `b_a` in norma euclidea:
  $\lVert p_o - \hat p_o \rVert \le b_p$, e così via. Con errori gaussiani
  isotropi e $b = k\sigma$, la probabilità che valga il bound in norma è
  quella di una distribuzione chi a 3 gradi di libertà (circa 97% per
  `k = 3`). Le dimostrazioni usano però solo proiezioni: la componente lungo
  `u` (una dimensione, 99.7% per `k = 3`) e quella normale (due dimensioni,
  circa 98.9%).
- **(A2) Robot esatto.** Posizione e velocità del robot sono note da encoder e
  cinematica. L'incertezza residua è in `Z_r`.
- **(A3) Errori lentamente variabili.** Nel periodo di controllo gli errori di
  misura sullo stato 3D dell'ostacolo, $\hat p_o - p_o$, $\hat v_o - v_o$ e
  $\hat a_o - a_o$, sono costanti. Serve solo per la robustezza della
  derivata (§6.6).
- **(A4) Accelerazione limitata.** Durante la manovra l'accelerazione vera
  dell'ostacolo lungo `u` soddisfa
  $a_{o\parallel}(t) \le \hat a_{o\parallel} + b_{ao} + \Delta a_{\max}$.
  `Δa_max` copre le variazioni non previste dalla stima, compresa la partenza
  di una persona ferma. Le accelerazioni oltre questo bound vanno in `Z_acc`.

### 6.2 Dai bound sullo stato ai bound su ψ

**Lemma 6.1.** Sotto (A1)–(A2), al primo ordine in $b_p/D$, lo stato vero ψ
soddisfa $\lvert \psi_i - \hat\psi_i \rvert \le b_i$, con

$$
b_S = b_p,
\qquad
b_{vr} = b_p\,\frac{\lVert P v_r \rVert}{D},
\qquad
b_{vo} = b_v + b_p\,\frac{\lVert P v_o \rVert}{D},
\qquad
b_{ao} = b_a + b_p\,\frac{\lVert P a_o \rVert}{D}.
$$

*Dimostrazione.* `D` è 1-Lipschitz nella posizione, quindi
$\lvert \Delta D \rvert \le b_p$. La direzione `u` ha jacobiano $-P/D$
rispetto a $p_o$, quindi $\lVert \Delta u \rVert \le b_p/D$ al primo ordine,
con $\Delta u$ ortogonale a `u`. Per una proiezione,
$\Delta(w \cdot u) = \Delta w \cdot u + (P w) \cdot \Delta u$, e
$\lvert (P w) \cdot \Delta u \rvert \le \lVert P w \rVert\, b_p / D$. Con
$\Delta w = 0$ per il robot (A2) e $\lVert \Delta w \rVert \le b_v$ o $b_a$
per l'ostacolo si ottengono i quattro bound. ∎

I bound sono somme, non radici di somme di quadrati: descrivono una scatola di
caso peggiore, non una covarianza. Anche `v_r∥` è incerta, pur con il robot
misurato esattamente, perché l'errore di posizione dell'ostacolo ruota `u`.

### 6.3 Monotonia

**Lemma 6.2.** `S_min(ψ)` è non decrescente in `S` e in `v_r∥`, e non
crescente in `v_o∥` e in `a_o∥`.

*Dimostrazione.* Per ogni `τ` fissato,
$\partial S(\tau)/\partial S = 1$,
$\partial S(\tau)/\partial v_{r\parallel} = \Phi(\tau) \ge 0$,
$\partial S(\tau)/\partial v_{o\parallel} = -\tau \le 0$,
$\partial S(\tau)/\partial a_{o\parallel} = -\tau^2/2 \le 0$. La finestra
$[0, T_f]$ non dipende da ψ, e il minimo su un insieme fisso di una famiglia
di funzioni monotone in un parametro è monotono nello stesso verso. ∎

### 6.4 Accelerazioni variabili

**Proposizione 6.3.** Sia $a(\cdot)$ una qualunque storia di accelerazione
dell'ostacolo lungo `u` con $a(t) \le \bar a$ su $[0, T_f]$. Allora
$S(\tau; a(\cdot)) \ge S(\tau; \bar a)$ per ogni `τ`, e quindi
$S_{\min}(a(\cdot)) \ge S_{\min}(\bar a)$.

*Dimostrazione.*

$$
S(\tau; a(\cdot)) = S + v_{r\parallel}\Phi(\tau) - v_{o\parallel}\tau - \int_0^\tau\!\!\int_0^s a(r)\,dr\,ds,
\qquad
\int_0^\tau\!\!\int_0^s a(r)\,dr\,ds \le \bar a\,\frac{\tau^2}{2}. \quad ∎
$$

L'ipotesi di accelerazione costante non serve più: basta un limite superiore.
Il caso peggiore è l'accelerazione costante pari al limite.

### 6.5 L'angolo peggiore

**Proposizione 6.4.** Sia $B = \{\psi : \lvert \psi_i - \hat\psi_i \rvert \le b_i\}$
la scatola del Lemma 6.1 e

$$
\psi_w = \big(\hat S - b_S,\;\; \hat v_{r\parallel} - b_{vr},\;\; \hat v_{o\parallel} + b_{vo},\;\; \hat a_{o\parallel} + b_{ao} + \Delta a_{\max}\big).
$$

Per ogni stato vero in `B` e ogni storia di accelerazione che soddisfa (A4),
la separazione minima vera durante la manovra è almeno $S_{\min}(\psi_w)$.
Quindi $h(\psi_w) \ge 0$ implica che la separazione resta almeno $S_{p,0}$
fino all'arresto, entro il primo ordine del Lemma 6.1.

*Dimostrazione.* Per la Proposizione 6.3 basta considerare l'accelerazione
costante pari al limite. Per il Lemma 6.2 il minimo di `S_min` su `B` è
nell'angolo. Il Lemma 4.1 estende il risultato alla separazione 3D. ∎

**Calcolo nel 3D.** L'angolo si ottiene spostando le velocità lungo `u`
($v_r - b_{vr} u$, $v_o + b_{vo} u$, $a_o + (b_{ao} + \Delta a_{\max}) u$) e
aggiungendo $b_S$ al raggio. Gli spostamenti lungo `u` cambiano le proiezioni
esattamente di $b_i$ e lasciano invariati `u`, `D` e le componenti normali.

### 6.6 Robustezza della derivata

#### 6.6.1 Cosa si vuole garantire

La barriera usata nel QP è una funzione delle grandezze **misurate**:

$$
h_{\text{rob}}(t) = h\big(\psi_w(t)\big),
\qquad
\psi_w = \hat\psi + \delta,
\qquad
\delta = \big(-b_S,\; -b_{vr},\; +b_{vo},\; +b_{ao} + \Delta a_{\max}\big),
$$

con `T_S(q̇)` come parametro esatto. Per la Proposizione 6.4, $h_{\text{rob}} \ge 0$
garantisce la separazione per ogni stato vero compatibile con la misura. Basta
quindi che $h_{\text{rob}}$ resti non negativa nel tempo.

La formulazione continua impone
$L_f h + L_g h\,\ddot q + \gamma h \ge 0$ con derivate di Lie calcolate
sullo **stato 3D dell'angolo**: la geometria misurata ($\hat u$, $\hat D$), le
velocità spostate lungo $\hat u$ e il raggio gonfiato (§6.5).

Il tasso vero di $h_{\text{rob}}$ però non è quello calcolato. Dipende da come
si muove davvero l'ostacolo: la sua velocità, accelerazione e jerk veri, che
la misura conosce solo entro i bound. In generale

$$
\frac{d}{dt} h_{\text{rob}} = L_f h(\psi_w) + L_g h(\psi_w)\,\ddot q + E,
$$

dove l'**errore di tasso** `E` [m/s] è la differenza tra il tasso vero e
quello calcolato nell'angolo. `E` non è noto, perché dipende dallo stato vero
dell'ostacolo.

**Cosa è `β`.** È un limite inferiore di `E`, valido per ogni stato vero
compatibile con le ipotesi: un numero non negativo, calcolabile dalle sole
grandezze misurate e dai bound, tale che

$$
E \ge -\beta
\quad\Longleftrightarrow\quad
\frac{d}{dt} h_{\text{rob}} \;\ge\; L_f h(\psi_w) + L_g h(\psi_w)\,\ddot q - \beta .
$$

In altre parole, `β` è di quanto il tasso vero può risultare *peggiore* di
quello calcolato nell'angolo. Ha le unità di un tasso [m/s], non di una
distanza.

**A cosa serve.** Se il QP impone la condizione rinforzata
$L_f h + L_g h\,\ddot q + \gamma h - \beta \ge 0$, allora
$\dot h_{\text{rob}} + \gamma\, h_{\text{rob}} \ge 0$ lungo il moto vero. Per il
lemma di confronto, $h_{\text{rob}}(0) \ge 0$ implica $h_{\text{rob}}(t) \ge 0$.
Poiché `β` è un tasso, si sottrae alla condizione senza moltiplicarlo per `γ`.

I paragrafi seguenti calcolano `E` componente per componente (§6.6.3–6.6.4)
e ne ricavano `β` (§6.6.5). Nel caso di default, senza escape, `β` si riduce a
due soli contributi: il jerk dell'ostacolo e la rotazione di $\hat u$ dovuta
all'errore di velocità (§6.6.6).

#### 6.6.2 Ipotesi

- **(A3)** Gli errori sullo stato 3D dell'ostacolo sono costanti nel periodo.
  Le derivate dei segnali misurati sono quindi quelle vere:
  $\dot{\hat p}_o = v_o$, $\dot{\hat v}_o = a_o$, $\dot{\hat a}_o = j_o$, con
  $j_o$ il jerk vero.
- **(A5) Jerk limitato.** $\lvert \hat u \cdot j_o \rvert \le j_{\max}$ nel
  periodo di controllo.
- **Bound congelati.** $\delta$ è trattato come costante nel periodo, anche se
  $b_{vr}$, $b_{vo}$ e $b_{ao}$ dipendono dallo stato tramite
  $\lVert P w \rVert / D$ (§11).

Si indica con $\Delta v = \hat v_o - v_o$ e $\Delta a = \hat a_o - a_o$
l'errore di velocità e di accelerazione, con $\lVert \Delta v \rVert \le b_v$
e $\lVert \Delta a \rVert \le b_a$ per (A1), e con
$g_i = \partial h / \partial \psi_i$ il gradiente nell'angolo.

#### 6.6.3 Tasso vero e tasso calcolato

Con $\delta$ costante, $\dot\psi_w = \dot{\hat\psi}$, quindi

$$
\frac{d}{dt} h_{\text{rob}} = \sum_{i=0}^{3} g_i\, \dot{\hat\psi}_i + \frac{\partial h}{\partial T_S}\,\nabla_{\dot q} T_S^\top \ddot q .
$$

Derivando $\hat\psi$ con
$\dot{\hat u} = \hat P\,(v_r - \dot{\hat p}_o)/\hat D = \hat P\,(v_r - v_o)/\hat D$,
cioè con la velocità **vera** dell'ostacolo per (A3), si ottiene la colonna
«tasso vero». La colonna «calcolato» è $\dot\psi$ sullo stato 3D dell'angolo.
Negli spostamenti lungo $\hat u$ i termini con $\hat P$ si annullano, quindi i
vettori normali sono quelli misurati:

| `i` | Tasso vero $\dot{\hat\psi}_i$ | Calcolato nell'angolo $\dot\psi_{w,i}$ |
|---|---|---|
| 0 | $\hat u\cdot v_r - \hat u\cdot v_o$ | $(\hat v_{r\parallel} - b_{vr}) - (\hat v_{o\parallel} + b_{vo})$ |
| 1 | $\frac{\hat P(v_r - v_o)}{\hat D}\cdot v_r + \hat u\cdot a_r$ | $\frac{\hat P(v_r - \hat v_o)}{\hat D}\cdot v_r + \hat u\cdot a_r$ |
| 2 | $\frac{\hat P(v_r - v_o)}{\hat D}\cdot \hat v_o + \hat u\cdot a_o$ | $\frac{\hat P(v_r - \hat v_o)}{\hat D}\cdot \hat v_o + \hat a_{o\parallel} + b_{ao} + \Delta a_{\max}$ |
| 3 | $\frac{\hat P(v_r - v_o)}{\hat D}\cdot \hat a_o + \hat u\cdot j_o$ | $\frac{\hat P(v_r - \hat v_o)}{\hat D}\cdot \hat a_o$ |

Due fatti sono già visibili:

- **Il termine di controllo è identico.** In entrambe le colonne compare
  $\hat u \cdot a_r$, e il termine in `T_S` è esatto. `β` quindi non dipende
  da `q̈`.
- **La geometria è la stessa.** Entrambe le colonne usano $\hat u$ e $\hat D$,
  perché $h_{\text{rob}}$ è una funzione delle grandezze misurate. Non compare
  la distanza vera.

#### 6.6.4 Differenza componente per componente

L'errore $E = \tfrac{d}{dt}h_{\text{rob}} - (L_f h + L_g h\,\ddot q)$ è
$E = \sum_i g_i\,(\dot{\hat\psi}_i - \dot\psi_{w,i})$. Sottraendo le due
colonne:

| Contributo | Differenza | Intervallo |
|---|---|---|
| Velocità relativa, `i = 0` | $\Delta_0 = b_{vr} + b_{vo} + \hat u\cdot\Delta v$ | $[\,b_{vr} + b_{vo} - b_v,\; b_{vr} + b_{vo} + b_v\,] \subset [0,\; 2(b_{vr} + b_{vo})]$ |
| Accelerazione, `i = 2` | $\Delta_2 = -(b_{ao} + \Delta a_{\max}) - \hat u\cdot\Delta a$ | $[\,-(b_{ao} + \Delta a_{\max}) - b_a,\; -(b_{ao} + \Delta a_{\max}) + b_a\,] \subset [-2b_{ao} - \Delta a_{\max},\; -\Delta a_{\max}]$ |
| Jerk, `i = 3` | $\Delta_3 = \hat u\cdot j_o$ | $[-j_{\max},\; j_{\max}]$ |
| Rotazione, `i = 1, 2, 3` | $\Delta_{\text{rot}} = \dfrac{\hat P\Delta v}{\hat D}\cdot m$, con $m = g_1 \hat P v_r + g_2 \hat P \hat v_o + g_3 \hat P \hat a_o$ | $\lvert \Delta_{\text{rot}} \rvert \le b_v\,\lVert m \rVert / \hat D$ |

Gli intervalli usano $b_{vo} \ge b_v$ e $b_{ao} \ge b_a$ (Lemma 6.1). La
differenza di rotazione nasce perché $\dot{\hat u}$ dipende dalla velocità
vera dell'ostacolo, mentre l'angolo usa quella misurata; si sommano i tre
termini perché il fattore $\hat P \Delta v / \hat D$ è comune.

#### 6.6.5 Il bound

**Proposizione 6.5.** Sotto (A1)–(A5), con bound congelati,

$$
\frac{d}{dt} h_{\text{rob}} \;\ge\; L_f h(\psi_w) + L_g h(\psi_w)\,\ddot q - \beta,
$$

$$
\beta =
\Big[-\min\big(g_0\,\underline{\Delta}_0,\; g_0\,\overline{\Delta}_0\big)\Big]_+
+ \Big[-\min\big(g_2\,\underline{\Delta}_2,\; g_2\,\overline{\Delta}_2\big)\Big]_+
+ \lvert g_3 \rvert\, j_{\max}
+ \frac{b_v\,\lVert m \rVert}{\hat D},
$$

dove $\underline{\Delta}_i$ e $\overline{\Delta}_i$ sono gli estremi degli
intervalli del §6.6.4 e $[x]_+ = \max(0, x)$.

*Dimostrazione.* $E = g_0\Delta_0 + g_2\Delta_2 + g_3\Delta_3 + \Delta_{\text{rot}}$.
Ogni termine è lineare in una differenza che varia in un intervallo, quindi il
suo minimo è a un estremo: $g_0\Delta_0 \ge \min(g_0\underline\Delta_0, g_0\overline\Delta_0)$,
e lo stesso per $g_2\Delta_2$. Poi $g_3\Delta_3 \ge -\lvert g_3\rvert j_{\max}$
e $\Delta_{\text{rot}} \ge -b_v\lVert m\rVert/\hat D$. Sommando e scartando i
contributi positivi si ha $E \ge -\beta$. ∎

Scartare i contributi positivi dei primi due termini è conservativo: in
cambio `β` resta non negativo e semplice da calcolare.

#### 6.6.6 Il caso senza escape (default)

Senza termine di escape $g_0 = \partial h/\partial S = 1$,
$g_2 = -\tau^* \le 0$ e $g_3 = -\tau^{*2}/2 \le 0$. Allora:

- $g_0\,\Delta_0 \ge 0$, perché $\Delta_0 \ge 0$;
- $g_2\,\Delta_2 \ge 0$, perché $\Delta_2 \le -\Delta a_{\max} \le 0$.

L'angolo è quindi pessimista anche per il tasso: le velocità e l'accelerazione
vere fanno crescere $h_{\text{rob}}$ più di quanto prevede il calcolo
nell'angolo. Restano solo due contributi:

$$
\beta = \frac{\tau^{*2}}{2}\, j_{\max} + \frac{b_v\,\lVert m \rVert}{\hat D}.
$$

- **Jerk dell'ostacolo.** Un ostacolo che aumenta la propria accelerazione
  verso il robot fa scendere davvero la barriera: l'angolo non può
  anticiparlo, perché il jerk non è nello stato. Senza (A5) non c'è un bound.
- **Rotazione di $\hat u$.** La direzione misurata ruota con la velocità vera
  dell'ostacolo, non con quella stimata. Il termine scala con $1/\hat D$ ed è
  piccolo lontano dall'ostacolo.

Con l'escape attivo (§9), $g_0 = 1 - \varepsilon$ e
$g_2 = -(1 - \varepsilon)\tau^*$ cambiano segno per $\varepsilon > 1$, e i
primi due termini di `β` diventano positivi.

#### 6.6.7 Uso nel QP

- `β` è un tasso [m/s]: nella condizione continua si sottrae **senza** `γ`
  (§7.2, `robust_rate`).
- Non dipende da `q̈`: non serve conoscere l'accelerazione della soluzione.
- La formulazione discreta vincola $h_{\text{rob}}$ direttamente ai nodi, con
  l'ostacolo estrapolato e i bound propagati (§6.7), e non usa `β`. Il jerk
  vi compare come variazione di $\hat a_{o\parallel}$ tra un ciclo e l'altro, e
  la copre solo il margine di equilibrio (Osservazione 7.2).

**Differenza rispetto alla forma precedente.** La versione in
[robust_cbf_method.md](robust_cbf_method.md) §5.6 confrontava la derivata
nell'angolo con la dinamica dello stato vero, e ne ricavava un termine
$\lvert\sum g_i r_i\rvert\, b_S / D$ dovuto alla distanza vera. Qui il
confronto è con il tasso della barriera robusta, che dipende solo da
grandezze misurate. Quel termine sparisce, il termine di controllo diventa
esatto, e compaiono il jerk e la rotazione di $\hat u$ dovuta all'errore di
velocità.

### 6.7 Propagazione lungo l'orizzonte

Nei nodi futuri del QP l'ostacolo è estrapolato con l'accelerazione stimata
fino al tempo `t`. Gli errori si propagano e, nel caso peggiore, si sommano;
per la Proposizione 6.3 vi si aggiunge la variazione di accelerazione:

$$
b_p(t) = k\left(\sigma_p + \sigma_v t + \sigma_a \tfrac{t^2}{2}\right) + \Delta a_{\max} \tfrac{t^2}{2},
\qquad
b_v(t) = k(\sigma_v + \sigma_a t) + \Delta a_{\max}\, t,
\qquad
b_a(t) = k\,\sigma_a .
$$

### 6.8 Forma costante di `Z_d`

Per il §4.4, la somma $\sum_i \lvert \partial S_{\min}/\partial \psi_i \rvert\, k\sigma_i$
con le sole componenti dell'ostacolo vale
$k(\sigma_p + \sigma_v \tau^* + \sigma_a \tau^{*2}/2)$. È l'eq. (28)
dell'articolo di riferimento della CBF, con $\tau^*$ al posto della
sovrastima $\bar T_f$ (`cbf::perceptionMargin()`). Usata come costante
`Z_d = δ_H`, gonfia l'ostacolo per ogni stato. Con la scatola il margine
dipende dallo stato ed è piccolo quando il minimo è vicino nel tempo. Le due
forme non vanno usate insieme per lo stesso rumore.

---

## 7. Condizione CBF nel QP

### 7.1 Derivate di Lie

L'ingresso del QP è l'accelerazione articolare `q̈`, con
$a_r = J\ddot q + \dot J\dot q$. `h` dipende dal robot tramite $p_r$ (`S`,
`u`), $v_r$ (`v_r∥`) e `T_S(q̇)`:

$$
\dot h = L_f h + L_g h\,\ddot q,
\qquad
L_g h = \frac{\partial h}{\partial v_{r\parallel}}\, u^\top J + \frac{\partial h}{\partial T_S}\,\nabla_{\dot q} T_S^\top,
\qquad
L_f h = \nabla_\psi h \cdot \dot\psi_f + \frac{\partial h}{\partial v_{r\parallel}}\, u \cdot \dot J \dot q .
$$

`ψ̇_f` è la derivata di ψ a ingresso nullo. Poiché `u` ruota,
$\dot u = P(v_r - v_o)/D$:

$$
\dot\psi_{f,0} = u \cdot (v_r - v_o),
\quad
\dot\psi_{f,1} = \frac{P v_r}{D} \cdot (v_r - v_o),
\quad
\dot\psi_{f,2} = \frac{P v_o}{D} \cdot (v_r - v_o) + a_{o\parallel},
\quad
\dot\psi_{f,3} = \frac{P a_o}{D} \cdot (v_r - v_o).
$$

I termini con $P(\cdot)/D$ sono i **termini di rotazione**. Il gradiente
rispetto alla posizione del robot, usato dalla formulazione discreta, è

$$
\frac{\partial h}{\partial p_r} = \frac{\partial h}{\partial S}\, u + \sum_{i=1}^{3} \frac{\partial h}{\partial \psi_i}\,\frac{P w_i}{D},
\qquad w = (v_r, v_o, a_o).
$$

`T_S(q̇)` è il massimo su più giunti e non è differenziabile dove due giunti
sono attivi: si usa il gradiente del giunto attivo. Tutte le grandezze sono
valutate nell'angolo $\psi_w$.

### 7.2 Due formulazioni

- **Continua**, imposta all'inizio di ogni intervallo `i`:

  $$
  L_g h\,\ddot q_i + L_f h + \gamma h - [\texttt{robust\_rate}]\,\beta \ge 0 .
  $$

- **Discreta** (default), linearizzata attorno alla soluzione precedente:

  $$
  h(X_{i+1}) \ge \max(0,\, 1 - \gamma\,\Delta t_i)\, h(X_i).
  $$

I dettagli della linearizzazione sono in [api.md §2.6](api.md#26-control-barrier-function).

### 7.3 Proprietà

**Osservazione 7.1 (nessun deadlock nel safe set).** Siano robot e ostacolo
fermi, senza incertezza, con $h \ge 0$. Allora $S_{\min} = S$, tutte le
componenti di $\dot\psi_f$ sono nulle, $\dot J \dot q = 0$ e, prendendo il
minimo in $\tau = 0$, $L_g h = 0$. La condizione continua si riduce a
$\gamma h \ge 0$, vera per ogni ingresso: il robot può muoversi. Il termine di
escape non serve dentro il safe set. *(Ricavata analiticamente, da verificare
con un test.)*

**Osservazione 7.2 (margine di equilibrio).** Con incertezza che cresce nel
tempo (§6.7) o con $\Delta a_{\max} > 0$, l'ostacolo nell'angolo si avvicina
anche quando quello stimato è fermo. Un robot fermo è allora in equilibrio
solo se `γ h` compensa la crescita del caso peggiore. Nella forma continua
$\gamma h \ge \tau^* (\hat a_{o\parallel} + b_{ao} + \Delta a_{\max})$, a meno
dei termini di rotazione. Nella discreta $\gamma\,\Delta t_i\, h(X_i)$ deve
coprire la riduzione di `h` tra i nodi. È il prezzo della robustezza: il robot
si tiene a una distanza dall'ostacolo fermo che cresce con `Δa_max` e
diminuisce con `γ`.

---

## 8. Stop protettivo e ripresa

### 8.1 Ruoli

- **La CBF è la riduzione di velocità** della norma: mantiene $S \ge S_p$
  adattando il moto.
- **Lo stop protettivo è la protezione**: scatta quando la CBF non basta, per
  esempio se l'ostacolo accelera oltre (A4), il tracking si perde, il
  controllo salta un ciclo o c'è un guasto.

### 8.2 Attivazione

Lo stop scatta se esiste una coppia `(j, k)` con

$$
h_{jk}(\psi_w) < 0 \quad\Longleftrightarrow\quad S < S_p \;\text{nel caso peggiore},
$$

valutata sullo stato attuale, con la barriera **senza** termine di escape.
Durante lo stop l'MPC non è usato. Lo stop eseguito deve essere quello del
modello (A-stop): frenata sincronizzata con la durata `T_S` usata nella
barriera.

### 8.3 Esecuzione e ripresa

- **Stop:** categoria 2 di [60204], seguita da un fermo monitorato.
- **Ripresa:** con il robot fermo e $ S \ge S_p + $ `resume_margin`
  mantenuto per `resume_delay`. A robot fermo `T_S = 0` e `S_p` contiene
  ancora l'avvicinamento dell'ostacolo durante `T_r`.
- **Spazio protetto:** la ripresa automatica richiede anche che nessuna
  persona resti nello spazio protetto dall'SSM ([10218-2]). Va garantito a
  livello di impianto.

### 8.4 Sicurezza funzionale

THOR non è una funzione di sicurezza certificata. Per una cella SSM:

- il rilevamento delle persone deve usare dispositivi certificati ([61496],
  IEC/TS 62046). Sensori non standardizzati, come telecamere con tracking, non
  possono essere l'unica misura di protezione ([MN17], §2.1);
- l'attivazione dello stop deve essere eseguita da parti di controllo di
  sicurezza con il performance level richiesto ([13849]);
- lo stop di THOR è uno strato funzionale, coerente con il modello, che si
  affianca alla catena di sicurezza dell'impianto senza sostituirla.

---

## 9. Termine di escape (opzione fuori SSM)

> **Avvertenza.** Il termine di escape permette al robot di muoversi mentre la
> separazione è sotto `S_p`. Questo comportamento è **fuori dall'SSM**: la norma
> richiede lo stop sotto la soglia. È disattivato di default e va abilitato solo
> dopo un risk assessment specifico.

### 9.1 Scopo e uso

Senza escape, dentro la regione $h < 0$ la condizione CBF può non avere
soluzione: per un robot fermo $L_g h$ è nullo o piccolo, e la condizione
richiede che `h` cresca. Il termine di escape dà autorità di controllo per
uscire dalla regione.

L'uso previsto è un **allontanamento dopo lo stop**: lo stop protettivo porta
comunque il robot a fermo, e solo se l'utente l'ha abilitato il robot può poi
allontanarsi guidato dalla barriera con escape.

### 9.2 Definizione

$$
h_e(\psi) = \Delta_s \left( 1 - H(-\Delta_s)\,\frac{T_r\, v_{r\parallel}}{S_e} \right)
=
\begin{cases}
\Delta_s & \Delta_s \ge 0\\[4pt]
-(S_{p,0} - S_{\min})\,(1 - \varepsilon) & \Delta_s < 0
\end{cases}
\qquad
\varepsilon = \frac{T_r\, v_{r\parallel}}{S_e} = \frac{v_{r\parallel}}{v_e}.
$$

`S_e = T_r v_e` è la distanza che il robot percorre nel tempo di reazione alla
velocità di fuga `v_e`: è `S_r` calcolata a `v_e`. **Non entra in `S_p`**: modella
solo il recupero.

### 9.3 Proprietà

- **Safe set.** $h_e \ge 0$ se e solo se $\Delta_s \ge 0$ oppure $\varepsilon \ge 1$.
  Un robot sotto `S_p` che si allontana più veloce di `v_e` risulta «sicuro»
  per la barriera. Per questo lo stop si attiva sulla barriera senza escape
  (§8.2).
- **Pendenza.** Per $\Delta_s < 0$, $\partial h_e/\partial S_{\min} = 1 - \varepsilon$,
  negativa per $\varepsilon > 1$: una `S_min` più piccola dà una `h_e` più
  grande. In quel regime l'angolo della Proposizione 6.4 non è il minimo di
  `h_e`.
- **Accelerazione minima di fuga.** Con robot e ostacolo fermi e
  $S < S_{p,0}$, la condizione continua diventa
  $u \cdot a_r \ge \gamma S_e / T_r$. Questa soglia non dipende da quanto
  il robot è entrato nel margine. Un `S_e` troppo grande rende il QP
  infeasible.

| Regime | `h_e` sotto `S_p` | $\partial h_e/\partial S_{\min}$ |
|---|---|---|
| $\varepsilon < 1$ | negativa | positiva |
| $\varepsilon = 1$ | nulla | nulla |
| $\varepsilon > 1$ | positiva | negativa |

### 9.4 Scelta di `S_e`

$$
T_r\, v_{\text{fuga}} \;\le\; S_e \;\le\; \frac{T_r\, a_{\text{fuga}}}{\gamma},
$$

con $v_{\text{fuga}}$ la velocità di allontanamento da tenere nel regime
$\varepsilon \le 1$ e $a_{\text{fuga}}$ l'accelerazione cartesiana disponibile
per allontanarsi. Se l'intervallo è vuoto, prevale il limite superiore.

### 9.5 Rischi da valutare

Il modello non conosce l'ambiente. Il risk assessment deve considerare almeno:

- **Schiacciamento.** Allontanarsi da una parte del corpo può schiacciarne
  un'altra contro un'attrezzatura o un ostacolo non tracciato, per esempio un
  gomito tra il robot e un fermo.
- **Parti non rilevate.** Persone, parti del corpo od oggetti fuori dal
  campo dei sensori.
- **Direzione di fuga.** La barriera guarda solo la coppia più critica, non
  lo spazio verso cui il robot si muove.

---

## 10. Scelta dei parametri

### 10.1 Intrusion distance `C`

[15066] rimanda a [13855], che fissa `C` in base alla capacità di rilevamento
`d` del dispositivo e alla sua disposizione ([MN17], Tab. 5):

| Criterio | Condizione | `C` |
|---|---|---|
| Direzione di avvicinamento normale | `d ≤ 40 mm` | `max(8(d − 14), 0)` mm |
| | `40 < d ≤ 70 mm` | 850 mm |
| | fasci multipli separati | 850 mm |
| | fascio singolo | 1200 mm |
| Parallela o inclinata | – | `max(1200 − 0.4 H, 850)` mm |

[MN17] (§7) dubita che questi valori, pensati per macchine fisse, si
applichino ai robot, e propone per esempio `C = 850 mm + d`, oppure un `C`
giustificato dal risk assessment.

Con un tracking che segue braccia e mani, l'intrusione dovuta alla portata del
braccio si riduce molto. `C` deve allora coprire ciò che può sporgere oltre la
geometria tracciata: le dita oltre la sfera della mano, oggetti tenuti,
vestiti, e l'errore dell'approssimazione del corpo con sfere e capsule. Un
valore piccolo di `C` è accettabile solo se argomentato così nel risk
assessment.

### 10.2 Incertezza del sensore `Z_d` (σ e `k`)

- I σ sono quelli dello **stimatore** dell'ostacolo (posizione, velocità e
  accelerazione filtrate), non del sensore grezzo, e vanno validati su dati
  registrati. Un metodo è misurare lo scarto dalla traiettoria interpolata
  ([MN17], Fig. 5).
- La velocità stimata dipende dalla frequenza di aggiornamento e dal filtro
  ([MN17], Tab. 2): i σ valgono per una configurazione fissata.
- [MN17] (§6) propone `Z_d = σ_{T_R} · v_H`, con `σ_{T_R}` la deviazione
  standard del tempo di reazione.
- Con `k = 3` la copertura è quella discussa in (A1).

### 10.3 Incertezza del robot `Z_r`

- Va presa come valore di caso peggiore, non medio ([9283], [MN17] §6).
- Le sfere del robot devono racchiudere i link, l'utensile e il pezzo tenuto,
  con la massima deflessione possibile dell'utensile ([MN17] §8.2).
- Copre anche gli errori di calibrazione della cinematica.

### 10.4 Tempo di reazione `T_r`

`T_r` comprende tutte le latenze, nel caso peggiore: acquisizione del
sensore, elaborazione, comunicazione, ciclo di controllo e risposta della
catena di sicurezza.

- Una posizione vecchia di `Δt` equivale a un errore di `v_H Δt`: a 2 m/s, mezzo
  secondo di ritardo vale 1 m ([MN17] §5).
- Va misurato con ripetizioni e preso con margine statistico. A titolo di
  esempio, [MN17] misura 0.113 s, con +0.019 s al 99.9%, e riporta un ritardo
  di 220 ms in una cella non nativamente SSM.
- Va rivalutato periodicamente.

### 10.5 Durata della frenata `T_S`

- `T_S(q̇)` deve essere un limite superiore del tempo di arresto dello stop
  eseguito, nella configurazione e con il carico attuali.
- I limiti `q̈_max,j` dello stop vanno confrontati con i dati di arresto del
  costruttore ([10218-1], Annex B), che dipendono da velocità, carico e posa.
- Vanno rivalutati con l'usura dei freni e le variazioni del carico ([MN17] §4).

### 10.6 Variazione di accelerazione `Δa_max` e `Z_acc`

- `Δa_max` è l'accelerazione verso il robot che una persona può sviluppare
  rispetto a quella stimata, partenza da fermo compresa. Va giustificato nel
  risk assessment.
- A parità di `S_h` all'istante `T`, il default della norma `v_H` corrisponde
  a $\Delta a = 2 v_H / T$. Per esempio, `v_H = 1.6 m/s` e `T = 0.35 s` danno
  circa 9.1 m/s². È un'equivalenza puntuale, non un valore raccomandato.
- `Z_acc` copre ciò che resta oltre `Δa_max`. Se `Δa_max` è un limite fisico,
  `Z_acc = 0`. Altrimenti si calibra in modo statistico: si predice `S(τ)` sui
  dati registrati e se ne misura l'errore fino a `T_f` (quantile al 99.7%).

`j_max` serve solo alla formulazione continua con `robust_rate` (§6.6). Va
stimato dagli stessi dati registrati, come quantile del jerk dello stimatore
lungo la direzione robot–ostacolo.

### 10.7 Prestazioni del controllo `Z_ctrl`

`Z_ctrl` copre la chiusura dovuta a un ciclo di controllo perso:
$Z_{ctrl} \ge v_{\text{chiusura,max}}\, T_{\text{miss}}$. In alternativa si
aumenta `T_r` di `T_miss`, ma non entrambe le cose. A titolo di esempio, a
33 Hz robot e persona che si avvicinano a 1 m/s ciascuno chiudono 6 cm tra
due aggiornamenti ([MN17] §8.1), che raccomanda almeno 100 Hz.

### 10.8 Parametri del controllo

- `γ`: un valore più grande permette di avvicinarsi più in fretta al bordo del
  safe set e riduce il margine di equilibrio dell'Osservazione 7.2; un valore
  più piccolo rende il robot più cauto.
- `S_e`: §9.4, solo se l'escape è abilitato.

---

## 11. Ipotesi e limiti

| Limite | Effetto | Mitigazione |
|---|---|---|
| Lemma 6.1 al primo ordine in $b_p/D$ | vicino all'ostacolo la rotazione di `u` è sottostimata di termini $O((b_p/D)^2)$ | `C`; test sulla scatola |
| (A-stop): velocità del punto $v_r\varphi(\tau)$ | trascura la variazione di `J` durante la frenata e la curvatura del percorso | `Z_r`; `T_S` con margine |
| `T_S` noto solo entro un intervallo | per un robot che si allontana né la sovrastima né la sottostima di `T_S` sono conservative | dare credito all'allontanamento con il `T_S` minimo e chiudere la finestra con quello massimo |
| (A3) errori costanti nel periodo | la derivata dell'errore dello stimatore non è coperta da `β` | stimatore filtrato; periodo di controllo breve |
| (A5) jerk limitato | senza `j_max` la formulazione continua non ha un bound sul tasso; nella discreta il jerk è coperto solo dal margine di equilibrio | `j_max` dai dati; formulazione discreta |
| Bound congelati | nei gradienti e nelle derivate di Lie `b` è trattato come costante, ma dipende dallo stato tramite $\lVert P w \rVert / D$ e quindi anche da `q̈` | effetto del secondo ordine |
| (A4) accelerazione limitata | le accelerazioni oltre `Δa_max` non sono coperte | `Z_acc` calibrato |
| Distanza tra i centri piccola | `D` è limitata inferiormente a 1 mm; con i centri coincidenti `u` è un asse fisso | regime già sotto `S_p` |
| Capsule | lungo il lato i bound si applicano ai vettori proiettati sul piano normale all'asse | – |
| Copertura probabilistica | con `k = 3` il bound in norma vale con circa il 97% | scegliere `k` sul bound che interessa |
| Sicurezza funzionale | THOR non è una parte di sicurezza certificata | §8.4 |

---

## 12. Differenze rispetto al codice attuale

| # | Oggi | Modello | Dove |
|---|---|---|---|
| 1 | lo stop scatta sulla distanza attuale, `S − δ_H − kσ_p < C` | lo stop scatta su `S < S_p`, cioè sulla barriera senza escape nell'angolo (§8.2) | `thor_core.cpp` |
| 2 | frenata con decelerazione `a_s` lungo `u`; per `v_r∥ ≥ 0` nessuna frenata (`T_f = T_r`) | frenata sincronizzata con `T_S(q̇)` e profilo `φ` anche per il robot che si allontana (§4.1) | `cbf.cpp`, `thor_math.cpp` |
| 3 | `L_g h` solo tramite `v_r∥` | anche tramite `T_S(q̇)` (§7.1) | `cbf.cpp`, `thor_math.cpp` |
| 4 | termine di escape sempre attivo, con `C_e = C` di default | escape opzionale, disattivato di default, usato dopo lo stop (§9) | `cbf.cpp`, `thor_core.cpp` |
| 5 | accelerazione dell'ostacolo costante | `Δa_max` nell'angolo e nei bound lungo l'orizzonte (§6.5, §6.7) | `cbf.cpp`, `thor_math.cpp` |
| 6 | un margine unico `C` (`thor.cbf.safety_margin`) | parametri `C`, `Z_r`, `Z_acc`, `Z_ctrl` sommati in `S_p,0` | `ThorConfig`, parametri ROS |
| 7 | `C_escape` (`thor.cbf.escape_margin`) | `S_e` (`escape_distance`) | idem |
| 8 | `cbf::minimumDistance()`, `BarrierTerms::d`, `CbfDiagnostics::distance` | `minimumSeparation()`, `separation`; diagnostici con `S`, `S_min`, `S_p` | API |
| 9 | figure con la vecchia nomenclatura | figure rigenerate | `docs/figures/make_figures.py` |
| 10 | `rate_bound` $= \lvert\sum g_i r_i\rvert\, b_S/D + 2[-g_0]_+(b_{vr} + b_{vo}) + 2[g_2]_+ b_{ao}$ | `β` della Proposizione 6.5, con jerk `j_max` e rotazione di $\hat u$ da $\Delta v$ (§6.6) | `cbf.cpp`; nuovo parametro `j_max` |

Test da aggiungere:

- l'esempio del §5.4, con 0.10 m di avvicinamento;
- lo stop su `S_min` con la distanza attuale sopra `S_p,0`;
- l'Osservazione 7.1, senza deadlock con `h ≥ 0` e senza escape;
- la Proposizione 6.3, con accelerazioni variabili campionate sotto il limite;
- i test esistenti della scatola e della derivata (`CornerIsTheMinimumOverTheBox`,
  `RateBoundCoversTheTrueRate`), con la nuova manovra. Il secondo va
  riscritto: il riferimento diventa il tasso di $h_{\text{rob}}$ calcolato con
  la geometria misurata e le derivate vere dell'ostacolo (§6.6.3), campionando
  $\Delta v$, $\Delta a$ e $j_o$ nei loro bound.

Nomi proposti per i parametri ROS:

| Parametro | Simbolo |
|---|---|
| `thor.cbf.intrusion_distance` | `C` |
| `thor.cbf.robot_position_uncertainty` | `Z_r` |
| `thor.cbf.acceleration_uncertainty` | `Z_acc` |
| `thor.cbf.control_uncertainty` | `Z_ctrl` |
| `thor.cbf.distance_error` | `Z_d` costante (`δ_H`) |
| `thor.cbf.obstacle_sigma`, `thor.cbf.sigma_bound` | σ e `k` |
| `thor.cbf.max_acceleration_change` | `Δa_max` |
| `thor.cbf.max_jerk` | `j_max` |
| `thor.cbf.escape.enabled`, `thor.cbf.escape.distance` | escape, `S_e` |

Il parametro `a_s` (`thor.cbf.braking_deceleration` o equivalente) sparisce:
`T_S` si ricava dai limiti di accelerazione dello stop.

---

## 13. Corrispondenza con la nomenclatura precedente

| Precedente ([robust_cbf_method.md](robust_cbf_method.md), `cbf.h`) | Questo documento |
|---|---|
| `d` | `S` |
| `d(τ)` | `S(τ)` |
| `d_min` | `S_min` |
| `ḋ` | `Ṡ` |
| `C` (margine totale) | `S_p,0 = C + Z_r + Z_acc + Z_ctrl` |
| `C₀` | `C` (intrusion distance), in parte |
| `C_modello` | `Z_acc`, ridotto da `Δa_max` |
| `Δd_miss` | `Z_ctrl` |
| `δ_H` | `Z_d` nella forma costante |
| `C_e`, `C_escape` | `S_e` |
| `a_s` (decelerazione lungo `u`) | `T_S(q̇)` e profilo `φ` |
| `T_f = T_r − v_r∥/a_s` | `T_f = T_r + T_S` |
| `Z_S` ([15066], [MN17]) | `Z_d` ([10218-2]:2025) |
| `v_h∥`, `a_h` (articolo della CBF) | `v_o∥`, `a_o∥` |

Le incongruenze dell'articolo di riferimento della CBF rispetto al codice
(segno della frenata, eq. (7b)–(7c), e termine robusto) sono discusse in
[robust_cbf_method.md](robust_cbf_method.md) §10.
