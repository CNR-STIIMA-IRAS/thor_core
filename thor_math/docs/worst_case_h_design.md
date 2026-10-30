# Barriera nel caso peggiore: `h(ψ_worst)`

Questa nota descrive come la CBF di `ThorQP` è resa robusta al rumore di
misura su tutto lo stato mappato `ψ`, e non solo sulla distanza: il
ragionamento, le scelte fatte e i limiti che restano.

**Stato:** implementato in `thor_math` (§10). Le sezioni 8 e 11.1 correggono
un'analisi precedente, che sovrastimava il termine su `ḣ`.

**In breve.** L'articolo e `cbf_python` aggiungono `+ h_chi delta_H` alla
condizione CBF, con `h_chi = ∂h/∂d`. `thor_math` valutava già la barriera alla
distanza peggiore `d - delta_H`. Ora generalizza questo approccio a tutte le
componenti di `ψ`: la barriera, `L_f h`, `L_g h` e i gradienti si valutano
nell'angolo peggiore `ψ_worst` della scatola di `±kσ` attorno allo stato
misurato. Il margine dipende così dallo stato, e `C` può scendere verso `C₀`.

Il metodo completo, con dimostrazioni e figure, è in
[robust_cbf_method.md](robust_cbf_method.md); questa nota resta come storia
delle scelte di design.

Riferimenti: [robust_margin_deadlock.md](robust_margin_deadlock.md),
[api.md](api.md), e le eq. (6)–(16) e (28) dell'articolo.

---

## 1. Situazione di partenza

La barriera dipende da `ψ = [d, v_r∥, v_h∥, a_h]` (nel codice
`[d, v_r, v_o, a_o]`):

```
h = d_min(ψ) − C                                     se d_min ≥ C
h = d_min − C + (C − d_min)/C_e · T_r · v_r∥         se d_min < C   (termine di escape)
```

con `C_e = C` prima di questa modifica (vedi §11.2).

Nel tempo il termine robusto è cambiato più volte:

| Versione | Condizione | Problema |
|---|---|---|
| Articolo / `cbf_python` | `ḣ + γh + h_chi δ_H ≥ 0` | allenta il vincolo e somma m a m/s |
| Prima correzione | `ḣ + γ(h − h_chi δ_H) ≥ 0` | deadlock: sul nuovo bordo `L_g h = 0` |
| Raggio gonfiato | `h` valutata a `d − δ_H` | robusta solo su `d`, con `δ_H` costante |
| **Attuale** | `h` valutata all'angolo peggiore di `ψ` | vedi §11 |

Una precisazione sul nome: nel codice `h_chi` **non** è il gradiente rispetto
allo stato χ ∈ R¹⁵. È uno scalare, la sola prima componente del gradiente
rispetto a ψ ([cbf.cpp](../src/thor_math/cbf.cpp)). Il gradiente completo è ora
`BarrierTerms::dh_dpsi`.

## 2. L'eq. (28) è già una sensibilità rispetto a ψ

Per il teorema dell'inviluppo, la derivata di `d_min` rispetto a un parametro
è la derivata di `d(τ)`, eq. (6b)/(7c), valutata in `τ_min`:

```
∂d_min/∂d⁰    = 1
∂d_min/∂v_h∥  = −τ_min
∂d_min/∂a_h   = −τ_min² / 2
```

Quindi

```
Σ_k |∂d_min/∂ψ_k| · kσ_k = kσ_p + kσ_v τ_min + kσ_a τ_min² / 2
```

Con `τ_min` sostituito da `T̄_f`, questa è esattamente l'eq. (28), cioè
`cbf::perceptionMargin()`. Ne seguono due cose:

- `δ_H` contiene già la sensibilità rispetto a ψ. Moltiplicarla ancora per
  `h_chi` conta due volte la parte su `d`.
- Usare il `τ_min` vero al posto di `T̄_f` dà un margine **dipendente dallo
  stato**: piccolo quando il minimo è vicino (`τ_min` piccolo), grande solo
  quando serve. È qui il guadagno rispetto a una `C` costante.

## 3. Restringimento lungo il gradiente: forma linearizzata

Al primo ordine, il caso peggiore di `h` entro `ψ̂ ± b` vale

```
h_worst ≈ h(ψ̂) − Σ_i |∂h/∂ψ_i| b_i
```

Applicando la CBF alla barriera robusta:

```
ḣ + γ (h − Σ_i |∂h/∂ψ_i| b_i) ≥ 0
⟺   ḣ ≥ −γh + γ · Σ_i |∂h/∂ψ_i| b_i
```

**Attenzione al fattore γ.** Il termine robusto è in metri, quindi entra
moltiplicato per γ [1/s]. La forma senza γ non è equivalente e non è
dimensionalmente coerente: somma m a m/s. È lo stesso errore del
`+ h_chi δ_H` dell'articolo.

## 4. Perché non basta la forma linearizzata

La forma `γ(h − Σ|∂h/∂ψ_i| b_i)` ha lo stesso difetto della prima correzione
(vedi [robust_margin_deadlock.md](robust_margin_deadlock.md)):

- sposta il bordo del safe set dove `d_min ≥ C`;
- lì il termine di escape è spento; per un robot fermo o che si allontana
  `∂h/∂v_r∥ = 0` e quindi `L_g h = 0`;
- la riga CBF degenera in `0 · a_r ≥ −(L_f h + γ(...))`; con un ostacolo
  fermo il QP resta infeasible per sempre.

Bordo e termine di escape devono spostarsi insieme. La forma linearizzata
sposta solo il primo.

## 5. Soluzione: valutare tutto nell'angolo peggiore

Invece della linearizzazione, si valuta la barriera nell'angolo peggiore
della scatola `ψ̂ ± b`. La condizione resta quella standard, con tutti i
termini calcolati in `ψ_worst`:

```
L_f h(ψ_worst) + L_g h(ψ_worst) · a_r + γ h(ψ_worst) ≥ 0
```

È la generalizzazione a tutto ψ del raggio gonfiato di `δ_H`. Il termine di
escape si attiva quando `d_min(ψ_worst) < C`, cioè proprio sul bordo
robusto, e il deadlock non si presenta (test
`RobotAtRestNextToAnUncertainObstacleCanMoveAway`).

### L'angolo è fisso: `d_min` è monotona in tutte le componenti

Per ogni `τ` fissato, `d(τ)` è:

- crescente in `d⁰` (derivata `1`);
- non crescente in `v_h∥` (derivata `−τ`);
- non crescente in `a_h` (derivata `−τ²/2`);
- non decrescente in `v_r∥`: lo spostamento del robot verso l'ostacolo è
  `v_r τ` nella reazione e `v_r τ + a_s(τ − T_r)²/2` in frenata, entrambi
  crescenti in `v_r`. Inoltre una `v_r` più grande accorcia la finestra di
  frenata `T_f = T_r − v_r/a_s`, e il minimo su meno istanti non può
  scendere.

Il minimo su `τ` di funzioni monotone è monotono, quindi `d_min` lo è in ogni
componente e il suo minimo sulla scatola è sempre l'angolo

```
ψ_worst = (d − b_d,  v_r∥ − b_vr,  v_h∥ + b_vh,  a_h + b_ah)
```

senza bisogno del segno del gradiente. Il test
`CbfMinimumDistance.IsMonotoneInEveryComponent` lo verifica su una griglia di
casi.

Per `h` vale lo stesso fuori dal margine (`h = d_min − C`). Dentro il margine
`∂h/∂d_min = 1 − T_r v_r∥ / C_e`: l'angolo resta il minimo di `h` finché
`v_r∥ ≤ C_e / T_r` in tutta la scatola. Oltre, cioè con il robot che si
allontana velocemente dentro il margine, l'angolo non è più il minimo di `h`.
È un caso favorevole (il robot sta già scappando) e il parametro `C_escape`
(§11.2) serve proprio a spostarlo lontano dalle velocità tipiche.

### Come è calcolato nel 3D

`barrierTerms()` sposta le velocità lungo `u` e gonfia il raggio:
`v_r − b_vr u`, `v_o + b_vo u`, `a_o + b_ao u`, `radius + b_d`. Gli spostamenti
lungo `u` cambiano solo le proiezioni: `u` e le componenti normali restano
quelle misurate. Così `L_f h`, `L_g h` e `grad_p` sono esattamente quelli dello
stato all'angolo (test `VelocityBoundsMoveTheObstacleTowardsTheRobot`).

## 6. Dai bound sullo stato ai bound su ψ

Il sensore misura lo stato dell'ostacolo, non ψ. Con incertezze isotrope di
semi-ampiezza `b_p`, `b_v`, `b_a` su posizione, velocità e accelerazione
dell'ostacolo, e robot misurato dagli encoder, i bound su ψ al primo ordine
sono:

```
b_d   = b_p
b_vr  = b_p |P v_r| / D
b_vo  = b_v + b_p |P v_o| / D
b_ao  = b_a + b_p |P a_o| / D
```

con `P = I − u uᵀ` (eq. 15) e `D = |p_r − p_o|`. Sono **somme**, non radici di
varianze, perché descrivono una scatola di caso peggiore (§7):
`|Δ(w·u)| ≤ |Δw| + |w| |Δu|`, e l'errore di posizione ruota `u` di al più
`b_p / D`. Osservazioni:

- **`v_r∥` non è esatta** anche se il robot è misurato perfettamente: l'errore
  su `p_h` ruota `u`, e quindi cambia la proiezione.
- **I termini `1/D` crescono vicino all'ostacolo**, cioè proprio dove il
  margine conta.
- **Con un moto radiale** (velocità parallele a `u`) restano solo `b_d = b_p`,
  `b_vo = b_v`, `b_ao = b_a`. Con solo `b_p` il risultato coincide con il
  raggio gonfiato `δ_H = b_p` (test `RadialPositionBoundInflatesTheRadius` e
  `PositionSigmaOnAStaticSceneIsDeltaH`).

`ThorQP` costruisce i bound da `obstacle_sigma = (σ_p, σ_v, σ_a)` e
`sigma_bound = k`. Lungo l'orizzonte l'ostacolo è estrapolato ad
accelerazione costante fino al tempo `t` del nodo, e gli errori si sommano:

```
b_p(t) = k (σ_p + σ_v t + σ_a t²/2)
b_v(t) = k (σ_v + σ_a t)
b_a    = k σ_a
```

Il test `CornerIsTheMinimumOverTheBox` campiona stati veri nella scatola 3D
(posizione compresa, che ruota `u`) e verifica che nessuno dia una `h` più
bassa dell'angolo, a meno di termini del secondo ordine.

## 7. Somma o RSS

Ci sono due modi di combinare le componenti:

```
somma:   Δ = Σ_i |∂h/∂ψ_i| b_i                     (scatola, caso peggiore)
RSS:     Δ = k σ_h,   σ_h = √(∇_ψhᵀ Σ_ψ ∇_ψh)      (gaussiana, linearizzata)
```

Vale sempre `σ_h ≤ Σ_i |∂h/∂ψ_i| σ_i`, quindi la RSS è meno conservativa. La
valutazione nell'angolo corrisponde alla somma ed è coerente con l'idea di
"caso peggiore entro ±kσ". Per un argomento di sicurezza ISO 10218 è la scelta
fatta. La RSS richiederebbe errori gaussiani, la linearità di `h` e la
covarianza piena di ψ (le componenti sono correlate attraverso `u`).

## 8. Robustezza su `ḣ`

**Correzione.** Una versione precedente di questa nota stimava il termine su
`ḣ` derivando `L_f h + L_g h · a_r` rispetto a ψ, in modo simmetrico attorno
al punto misurato (circa 0.6 m/s). Con la barriera valutata nell'angolo il
ragionamento giusto è un altro, e il termine è molto più piccolo.

La barriera robusta è `h_rob(ψ̂) = h(ψ̂ − b⊙s)`, con `s` il segno dell'angolo.
Con errori di misura che non cambiano nel tempo, la sua derivata lungo il
moto vero è

```
d/dt h_rob = ∇_ψ h(ψ_worst) · ψ̇(stato vero)
```

con il gradiente **fisso all'angolo**: non entrano derivate seconde di `h`.
`L_f h + L_g h · a_r` usa invece `ψ̇(ψ_worst)`. L'errore è quindi solo
`∇_ψ h(ψ_worst) · (ψ̇(vero) − ψ̇(ψ_worst))`, e si scompone così:

- **Parte di controllo.** `(∂h/∂v_r) u · a_r` non cambia, perché gli errori non
  spostano `u` (al primo ordine). Quindi **non serve l'accelerazione del
  robot**: la scelta di congelare `a_r` al ciclo precedente non è più
  necessaria.
- **Velocità e accelerazione.** Dall'angolo, le velocità e l'accelerazione
  vere differiscono di `[0, 2b]` nella direzione favorevole: ad esempio
  `ḋ = v_r − v_o` vero è maggiore o uguale a quello dell'angolo. Contribuiscono
  solo dove `∂h/∂d < 0` o `∂h/∂v_o > 0`, cioè nel regime in cui il termine di
  escape ha pendenza negativa (`v_r > C_e / T_r`).
- **Distanza.** I termini di rotazione di `ψ̇` scalano con `1/D` e cambiano
  con l'errore di distanza `±b_d`.

Il risultato è `BarrierTerms::rate_bound`, in m/s:

```
rate_bound = |Σ_{i=1..3} ∂h/∂ψ_i · rot_i| / D · b_d
           + 2 max(0, −∂h/∂d) (b_vr + b_vo)
           + 2 max(0, ∂h/∂v_o) b_ao
```

dove `rot_i` sono i termini di rotazione di `ψ̇`. Nella formulazione continua,
con `cbf_robust_rate = true`, viene sottratto dalla condizione **senza** γ.
La formulazione discreta vincola `h` a ogni nodo e non lo usa.

Il test `RateBoundCoversTheTrueRate` verifica che `ḣ` vero non scenda sotto
`ḣ(ψ_worst) − rate_bound`, sia con campioni nella scatola sia con il solo
errore di posizione (che altrimenti sarebbe nascosto dagli errori di velocità,
sempre favorevoli).

**Cosa non copre:**

- errori di misura che cambiano nel tempo (rumore non filtrato): `ψ̂̇`
  contiene anche la derivata dell'errore, che qui è assunta nulla;
- la rotazione di `u` dovuta all'errore di posizione nei termini di `ψ̇`;
- la dipendenza dei bound dallo stato (`|P v|/D`): nei gradienti e nelle
  derivate di Lie i bound sono trattati come costanti.

## 9. Conseguenze su `C`

Se l'incertezza di percezione entra nella barriera tramite `ψ_worst`, non deve
restare anche dentro `C` né in `δ_H`. Altrimenti è contata due volte, perché
l'eq. (28) è la stessa somma del §2 con `τ_min → T̄_f`. `C` deve tornare verso
`C₀`, più quello che il modello non copre (§11.3).

## 10. Implementazione

| Cosa | Dove |
|---|---|
| Bound su ψ e valutazione nell'angolo | `cbf::PerceptionBounds`, argomento `bounds` di `cbf::barrierTerms()` e `cbf::capsuleBarrierTerms()`, `BarrierTerms::psi_bound` |
| Bound dell'errore di `ḣ` | `BarrierTerms::rate_bound` |
| Gradiente completo | `BarrierTerms::dh_dpsi` |
| Scala del termine di escape | `ThorConfig::C_escape`, argomento `C_escape` di `cbf::barrier()` |
| Deviazioni standard | `ThorInput::obstacle_sigma` (per ostacolo), `ThorConfig::obstacle_sigma` (default), `ThorConfig::sigma_bound` |
| Termine su `ḣ` nel QP | `ThorConfig::cbf_robust_rate` (solo formulazione continua) |
| Estrapolazione dei bound | `perceptionBounds()` in [thor_math.cpp](../src/thor_math/thor_math.cpp) |

Con tutte le deviazioni standard a zero il comportamento è identico a prima:
i test esistenti, compresi i riferimenti Python a 1e-12, passano invariati, e
l'esempio `cbf_mpc_example` dà gli stessi numeri.

Le diagnostiche (`CbfDiagnostics::distance`, `v_robot`, `v_obstacle`)
riportano i valori **misurati**; `h_now` e `h_min` sono all'angolo.

Test aggiunti:

- `CbfMinimumDistance.IsMonotoneInEveryComponent`
- `CbfBarrier.EscapeScaleSetsOnlyTheSlope`
- `CbfPerceptionBounds.VelocityBoundsMoveTheObstacleTowardsTheRobot`
- `CbfPerceptionBounds.RadialPositionBoundInflatesTheRadius`
- `CbfPerceptionBounds.CornerIsTheMinimumOverTheBox`
- `CbfPerceptionBounds.RateBoundCoversTheTrueRate`
- `ThorQpIntegrationTest.PositionSigmaOnAStaticSceneIsDeltaH`
- `ThorQpIntegrationTest.RobotAtRestNextToAnUncertainObstacleCanMoveAway`
- validazione degli input in `CbfConfigurationRequiresMatchingModel` e
  `CbfUsesSphereRadiiAndPerObstacleDeltaH`

## 11. Punti aperti: analisi e scelte

Le stime numeriche di questa sezione sono ordini di grandezza, calcolati con i
valori delle Tabelle II e III dell'articolo: `γ = 9.65`, `T_r = 0.15 s`,
`C = 0.25 m`, `σ_p = 0.02 m`, `σ_v = 0.085 m/s`, `σ_a = 0.39 m/s²`.

### 11.1 Il termine su `ḣ`

Superato dal §8. La stima precedente (circa 0.25 + 0.35 m/s, un terzo del
termine su `h`) derivava `ḣ` attorno al punto misurato. Con la barriera
nell'angolo, velocità e accelerazione non contribuiscono al primo ordine
(salvo il regime di escape a pendenza negativa), e il termine si riduce alla
parte dovuta all'errore di distanza nei termini di rotazione. Non serve
congelare `a_r`, e il calcolo è analitico, senza differenze finite.

### 11.2 `v_r∥` e il termine di escape — implementato

**`v_r∥` non ha incertezza trascurabile.** Vicino all'ostacolo
`b_vr = b_p |P v_r| / D` (§6). Con `D = 0.3 m`, `|P v_r| = 1 m/s` e
`b_p = 3σ_p = 0.06 m` si ha `b_vr ≈ 0.2 m/s`. Grazie alla monotonia (§5) non
serve valutare due angoli: l'angolo con `v_r∥ − b_vr` è sempre il minimo di
`d_min`.

**Il cambio di segno dell'escape dipende da `C`.** Il termine di escape cambia
segno per `v_r∥ > C_e / T_r`:

- con `C = 0.25 m` la soglia è 1.67 m/s, lontana dalle velocità tipiche;
- se `C` scende verso `C₀ = 0.06 m` (§9), la soglia diventa **0.4 m/s**, una
  velocità normale.

**Scelta:** `ThorConfig::C_escape` separa la scala dell'escape da `C`. Il
termine parte sempre da `d_min = C` (necessario contro il deadlock), ma la
pendenza è `T_r v_r / C_escape`. Con `C_escape = 0` si usa `C`, come prima.

### 11.3 Il valore di `C`: errore di misura ed errore di modello

L'eq. (28) mescola due errori di natura diversa:

- **errore di misura**: `σ_p`, `σ_v` e `σ_a` dei sensori. Con questa modifica
  entra in `ψ_worst`;
- **errore di modello**: l'accelerazione dell'umano non è costante
  sull'orizzonte. Il testo dell'articolo dice che `C` assorbe il jerk non
  modellato, ma nell'eq. (28) non c'è un termine di jerk.

Il jerk non si può coprire nel caso peggiore. Con jerk tra 100 e 1000 m/s³
[19] e `T_f = 0.3 s`, `j T³ / 6` vale tra 0.45 e 4.5 m. Va trattato in modo
statistico.

**Proposta (in attesa del dataset):** calibrare l'errore di modello sui dati
registrati. Per ogni istante del dataset umano:

1. predire `d(τ)` con il modello ad accelerazione costante, eq. (6b)/(7c);
2. confrontarla con la distanza vera registrata, per `τ ∈ [0, T̄_f]`;
3. prendere il quantile al 99.7% dell'errore come `C_modello`.

`C` diventa la somma di termini con un'origine dichiarata, utile anche per la
valutazione del rischio:

```
C = C₀ + C_modello + Δd_miss        (Δd_miss dall'eq. (23)–(24))
```

### 11.4 Incongruenze dell'articolo

Va deciso quale numero è quello vero:

- negli esperimenti `C₀` valeva 0.06 m (Tabella III) o 0.1 m (Fig. 5b)?
- "`C + Δ_H = 0.25 m`" nella Fig. 5b vuol dire che `Δ_H` era separato da `C`,
  o è un'etichetta rimasta da una versione precedente? Con l'eq. (28) si ha
  `C ≈ 0.25 m` già senza `Δ_H`.

Il commento di `perceptionMargin()` citava l'"Eq. 9" di una versione
precedente dell'articolo: ora cita l'eq. (28).

### 11.5 Decisioni

- [x] Termine su `ḣ`: analitico, nell'angolo, senza `a_r` (§8); opzionale con
      `cbf_robust_rate`.
- [x] Separare la scala dell'escape da `C` (`C_escape`, §11.2).
- [ ] Script di calibrazione di `C_modello` sul dataset umano registrato
      (§11.3): in attesa del dataset.
- [ ] Valori di `C₀` e `Δ_H` usati negli esperimenti (§11.4).
- [x] Esporre `obstacle_sigma`, `sigma_bound`, `C_escape` e `cbf_robust_rate`
      come parametri ROS in `thor_controllers` (`thor.cbf.*` e campi `sigma_*`
      di `thor_msgs/Obstacle`).
- [ ] Coprire gli errori che cambiano nel tempo (§8, *Cosa non copre*).
