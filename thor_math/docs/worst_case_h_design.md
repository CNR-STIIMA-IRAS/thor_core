# Barriera nel caso peggiore: `h(ψ_worst)`

Questa nota raccoglie le considerazioni su come rendere la CBF di `ThorQP`
robusta al rumore di misura su tutto lo stato mappato `ψ`, e non solo sulla
distanza. È un documento di design: descrive cosa cambiare e perché, ma la
proposta non è ancora implementata.

**In breve.** L'articolo e `cbf_python` aggiungono `+ h_chi delta_H` alla
condizione CBF, con `h_chi = ∂h/∂d`. Oggi `thor_math` valuta invece la barriera
alla distanza peggiore `d - delta_H`. La proposta generalizza questo approccio a
tutte le componenti di `ψ`: la barriera, `L_f h` e `L_g h` si valutano nel
punto peggiore `ψ_worst` entro `±3σ_ψ`, con `σ_ψ` propagata dalle incertezze
sullo stato χ. Il margine così dipende dallo stato, e `C` può scendere verso
`C₀`.

Riferimenti: [robust_margin_deadlock.md](robust_margin_deadlock.md),
[api.md](api.md), e le eq. (6)–(16) e (28) dell'articolo.

---

## 1. Situazione attuale

La barriera dipende da `ψ = [d, v_r∥, v_h∥, a_h]` (nel codice
`[d, v_r, v_o, a_o]`):

```
h = d_min(ψ) − C                                   se d_min ≥ C
h = d_min − C + (C − d_min)/C · T_r · v_r∥         se d_min < C   (termine di escape)
```

Nel tempo il termine robusto è cambiato tre volte:

| Versione | Condizione | Problema |
|---|---|---|
| Articolo / `cbf_python` | `ḣ + γh + h_chi δ_H ≥ 0` | allenta il vincolo e somma m a m/s |
| Prima correzione | `ḣ + γ(h − h_chi δ_H) ≥ 0` | deadlock: sul nuovo bordo `L_g h = 0` |
| Attuale | `h` valutata a `d − δ_H` (raggio gonfiato) | robusta solo su `d`, con `δ_H` costante |

Una precisazione sul nome: nel codice `h_chi` **non** è il gradiente rispetto
allo stato χ ∈ R¹⁵. È uno scalare, la sola prima componente del gradiente
rispetto a ψ ([cbf.cpp](../src/thor_math/cbf.cpp)):

```cpp
out.h_chi = dh_dpsi(0);   // ∂h/∂d
```

Il gradiente rispetto a χ, eq. (14), entra solo nel calcolo di `L_f h` e
`L_g h`.

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
  `h_chi` conta due volte la parte su `d`. Dentro il margine `h_chi > 1`, fino
  a 2.5 nei casi di riferimento.
- Usare il `τ_min` vero al posto di `T̄_f` dà un margine **dipendente dallo
  stato**: piccolo quando il minimo è vicino (`τ_min` piccolo), grande solo
  quando serve. È qui il guadagno rispetto a una `C` costante.

## 3. Restringimento lungo il gradiente: forma linearizzata

Al primo ordine, il caso peggiore di `h` entro `ψ̂ ± 3σ_ψ` vale

```
h_worst ≈ h(ψ̂) − Σ_i |∂h/∂ψ_i| 3σ_ψi = h − 3 |∇_ψ h|ᵀ σ_ψ
```

Il segno di ogni contributo dipende dal segno di `∂h/∂ψ_i`. Se è positivo, il
caso peggiore è `ψ̂_i − 3σ_i`; se è negativo, è `ψ̂_i + 3σ_i`. In entrambi i
casi la barriera si abbassa.

Applicando la CBF alla barriera robusta:

```
ḣ + γ (h − 3 |∇_ψ h|ᵀ σ_ψ) ≥ 0
⟺   ḣ ≥ −γh + γ · 3 |∇_ψ h|ᵀ σ_ψ
```

**Attenzione al fattore γ.** Il termine robusto è in metri, quindi entra
moltiplicato per γ [1/s]. La forma `ḣ ≥ −γh + 3|∇_ψ h|ᵀσ_ψ`, senza γ, non è
equivalente e non è dimensionalmente coerente: somma m a m/s. È lo stesso
errore del `+ h_chi δ_H` dell'articolo.

## 4. Perché non basta la forma linearizzata

La forma `γ(h − 3|∇_ψ h|ᵀσ_ψ)` ha lo stesso difetto della prima correzione
(vedi [robust_margin_deadlock.md](robust_margin_deadlock.md)):

- sposta il bordo del safe set a `h = 3|∇_ψ h|ᵀσ_ψ`, cioè dove `d_min ≥ C`;
- lì il termine di escape è spento; per un robot fermo o che si allontana
  `∂h/∂v_r∥ = 0` e quindi `L_g h = 0`;
- la riga CBF degenera in `0 · a_r ≥ −(L_f h + γ(...))`; con un ostacolo
  fermo il QP resta infeasible per sempre.

Bordo e termine di escape devono spostarsi insieme. La forma linearizzata
sposta solo il primo.

## 5. Proposta: valutare tutto in `ψ_worst`

Invece della linearizzazione, si valuta la barriera nel punto peggiore della
scatola `ψ̂ ± 3σ_ψ`:

```
ψ_worst = ψ̂ − 3 σ_ψ ⊙ sign(∇_ψ h)
h_rob   = h(ψ_worst)
```

Anche `L_f h` e `L_g h` si calcolano in `ψ_worst`, e la condizione resta quella
standard:

```
L_f h_rob + L_g h_rob · a_r + γ h_rob ≥ 0
```

È la generalizzazione a tutto ψ di quello che il codice fa oggi su `d`, con il
raggio gonfiato di `δ_H`. Il termine di escape si attiva quando
`d_min(ψ_worst) < C`, cioè proprio sul bordo robusto, e il deadlock non si
presenta.

### Quando l'angolo della scatola è esatto

Per tre componenti su quattro il caso peggiore non è un'approssimazione. Per
ogni `τ` fissato, `d(τ)` è:

- crescente in `d⁰` (derivata `1`);
- non crescente in `v_h∥` (derivata `−τ`);
- non crescente in `a_h` (derivata `−τ²/2`).

Il minimo su `τ` di funzioni monotone è monotono, quindi anche `d_min` lo è, e
l'angolo scelto è il vero minimo nella scatola per `d`, `v_h∥` e `a_h`.

Le eccezioni:

- **`v_r∥`**: `∂d_min/∂v_r∥` può cambiare segno, perché `v_r∥` cambia anche
  `T_f`. L'incertezza su `v_r∥` è però piccola (vedi §6).
- **Termine di escape**: `∂h/∂d_min = 1 − T_r v_r∥ / C`, che cambia segno per
  `v_r∥ > C/T_r` (circa 1.67 m/s con i parametri della Tabella II). Sotto
  questa velocità, `h` è monotona in `d_min` e l'angolo resta esatto.

In questi casi il segno del gradiente va preso in `ψ̂`, accettando
l'approssimazione, oppure si valutano i due angoli lungo quella componente e
si tiene il minimo.

## 6. Propagare `σ_ψ` dall'incertezza sullo stato

Il sensore misura lo stato χ, non ψ. Le incertezze su ψ vanno quindi propagate
con l'eq. (14):

```
Σ_ψ = (∂ψ/∂χ) Σ_χ (∂ψ/∂χ)ᵀ
```

Con incertezze isotrope sull'umano, `Σ_p = σ_p² I`, `Σ_v = σ_v² I`,
`Σ_a = σ_a² I`, e robot misurato dagli encoder (incertezza trascurabile),
le varianze marginali sono:

```
σ_d²    = σ_p²
σ_vr∥²  = σ_p² |P v_r|² / d²
σ_vh∥²  = σ_v² + σ_p² |P v_h|² / d²
σ_ah²   = σ_a² + σ_p² |P v̇_h|² / d²
```

con `P = I − u uᵀ`, eq. (15). Osservazioni:

- **`v_r∥` non è esatta** anche se il robot è misurato perfettamente: l'errore
  su `p_h` ruota `u`, e quindi cambia la proiezione.
- **I termini `1/d` crescono vicino all'umano**, cioè proprio dove il margine
  conta. Un `σ_ψ` diagonale costante li ignora.
- **Correlazioni.** Con `Σ_p` isotropa, `d` è scorrelata dalle proiezioni
  (`uᵀP = 0`), ma `v_r∥`, `v_h∥` e `a_h` sono correlate tra loro attraverso
  `P/d`. Con la scatola del §5 servono solo le σ marginali; le correlazioni
  contano solo per la variante RSS (§7).

## 7. Somma o RSS

Ci sono due modi di combinare le componenti:

```
somma:   Δ = 3 Σ_i |∂h/∂ψ_i| σ_ψi           (scatola, caso peggiore)
RSS:     Δ = 3 σ_h,   σ_h = √(∇_ψhᵀ Σ_ψ ∇_ψh)   (gaussiana, linearizzata)
```

Vale sempre `σ_h ≤ Σ_i |∂h/∂ψ_i| σ_ψi`, quindi la RSS è meno conservativa. La
somma è coerente con l'idea di "caso peggiore entro ±3σ" e con la valutazione
in `ψ_worst`. Per un argomento di sicurezza ISO 10218 è la scelta consigliata.
La RSS richiede errori gaussiani, la linearità di `h` e la covarianza piena del
§6, non solo la diagonale.

## 8. Robustezza su `ḣ`: un secondo termine

`ψ_worst` protegge `h`, ma `L_f h` e `L_g h` sono comunque calcolati su
grandezze stimate. Le CBF robuste alla misura (Dean et al. 2020, Cosner et al.
2021) aggiungono un secondo termine, che stringe la condizione sulla derivata:

```
L_f h_rob + L_g h_rob · a_r + γ h_rob − 3 |∇_ψ ḣ|ᵀ σ_ψ ≥ 0
```

Questo termine è già in m/s e va **senza** γ. Con `a_h` stimata, che è la
componente più rumorosa, è probabilmente il più grande dei due. Valutare
`L_f h` e `L_g h` in `ψ_worst` ne copre una parte, ma non in modo garantito:
l'angolo peggiore per `h` non è necessariamente quello peggiore per `ḣ`.

## 9. Conseguenze su `C`

Se l'incertezza di percezione entra nella barriera tramite `ψ_worst`, non deve
restare anche dentro `C`. Altrimenti è contata due volte, perché l'eq. (28) è
la stessa somma del §2 con `τ_min → T̄_f`. `C` deve tornare verso `C₀`, più
quello che il modello non copre (jerk non modellato, latenze), secondo la
valutazione del rischio.

## 10. Piano di implementazione

1. In `cbf.cpp`, una funzione che, dati `ψ̂` e `σ_ψ`, restituisce `ψ_worst`
   (§5). Il segno del gradiente si prende da `dh_dpsi`, già calcolato.
2. Propagazione di `σ_ψ` dalle incertezze dell'ostacolo (§6). Nuovi campi per
   ostacolo in `ThorInput`, ad esempio `obstacle_sigma_p`, `obstacle_sigma_v`,
   `obstacle_sigma_a`, al posto di `obstacle_delta_H` o accanto a esso.
3. `barrierTerms()` valuta `h`, `L_f h` e `L_g h` in `ψ_worst`. Il gonfiaggio
   del raggio con `δ_H` diventa un caso particolare (solo `σ_d`).
4. Opzionale e separato: il termine su `ḣ` del §8, da attivare con un flag.
5. Test:
   - con `σ_ψ = 0` il risultato coincide con quello attuale;
   - con solo `σ_d > 0` coincide con il raggio gonfiato;
   - non-regressione del deadlock (`RobotAtRestInsideTheRobustBandCanMoveAway`)
     con σ su tutte le componenti;
   - `h(ψ_worst)` confrontata con il minimo per forza bruta sugli angoli della
     scatola;
   - `σ_ψ` propagata confrontata con un Monte Carlo sullo stato.

## 11. Punti aperti

- Come calcolare `∇_ψ ḣ` del §8: per via analitica (servono derivate seconde
  di `d_min`) oppure con una costante di Lipschitz conservativa.
- Se trattare `v_r∥` e il cambio di segno del termine di escape con l'angolo
  in `ψ̂` o valutando due angoli (§5).
- Il valore di `C` dopo la modifica: va rifatto con la valutazione del rischio.
- L'articolo non è coerente sui numeri. La Fig. 5b riporta `C₀ = 0.1 m` e
  `C + Δ_H = 0.25 m`. La Tabella III dà `C₀ = 0.06 m`, e l'eq. (28) dà
  `C ≈ 0.25 m` senza un `Δ_H` separato. Il commento di `perceptionMargin()`
  cita ancora l'"Eq. 9" di una versione precedente.
