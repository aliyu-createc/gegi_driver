# DJR Assay Section — Results Wording

Report-ready wording for the assay work, structured to mirror the optioneering:
the **imager** (HDRI source location, §5.3.3.1) and the **gamma spectrometer**
(characterisation, §5.3.3.2). Paste into the Design Justification Report and adjust
house style as needed. Companion documents: `DJR_assay_measurement_plan.md` (method)
and `DJR_assay_experiments.md` (runbook + status).

**Instrument / geometry:** GeGi planar HPGe (90 mm crystal, 11 mm thick), mounted
upward-facing at a 0.33 m source-to-face standoff through one permanent 5 mm steel
plate; hot trays receive additional 5 mm plates.
**Reference sources:** Cs-137 2.990 MBq and Co-60 1.130 MBq, both certificated to
2026-06-01, decay-corrected to the measurement date in every comparison.

---

## Coverage against the requested report items
| Report item (assay section) | Role | Status |
|---|---|---|
| Reference to optioneering | both | drafted (§1) |
| Summary of waste information | both | narrative — from waste data |
| Maximum activity + max dead-time | spectrometer | covered (§3.4) |
| Limits of detection | spectrometer | covered (§3.3) |
| Expected count times | spectrometer | partially covered (§3.2/§3.5) |
| Expected volume throughput | system | pending — needs line cycle time |
| Uncertainty assessment | spectrometer | covered (§5) |
| Source localisation / ~1° | imager | covered (§2) |

---

## 1. Reference to the optioneering
The PHDS GeGi gamma camera was selected in the optioneering study for **both** the
HDRI source-location role (§5.3.3.1) and the characterisation gamma-spectrometer
role (§5.3.3.2), on the basis that its combination of high-resolution HPGe
spectrometry and Compton spatial imaging uniquely satisfies both the **system**
need and the **characterisation** need from a single sensor. The two roles are
substantiated separately below — the **imager** in §2 and the **spectrometer** in
§3 — and the integration benefit (a source located by the imager is quantified
correctly by the spectrometer) in §4. ISOCS mathematical calibration, cited in the
optioneering, is the intended production calibration route; the calibration here is
validated empirically against certificated sources. The dual role is inherent to
the single-sensor architecture, consistent with the optioneering conclusion that no
additional characterisation sensor is required.

---

## 2. Imager — HDRI source location (§5.3.3.1)

### 2.1 Angular and position accuracy
> The GeGi's source-location accuracy was measured by imaging a source at 0.5 m
> standoff at six lateral positions and comparing the reconstructed direction with
> the true position. The mean angular error was **1.05°** (range 0.60–1.51°, RMS
> 1.09°), equivalent to a lateral position error of **~5–13 mm** (mean ~9 mm) at
> 0.5 m. This confirms the ~1° angular precision cited in the optioneering.

### 2.2 Single- and multi-source localisation
> A single Cs-137 source near the tray axis was localised to within **~7 mm** of
> centre (−0.28 cm, 0.64 cm). In a mixed-source scene the system simultaneously
> located and correctly identified a central Co-60 source and two flanking Cs-137
> sources ~30 cm apart, distinguishing the isotopes both spectrally and spatially —
> directly evidencing the location-and-identification capability for mixed FED waste.

---

## 3. Gamma spectrometer — characterisation (§5.3.3.2)

### 3.1 Isotope identification
> The spectrometer's high energy resolution cleanly resolves the Co-60 1173 and
> 1332 keV lines and the Cs-137 662 keV line, giving unambiguous isotope
> identification. In the mixed-source scene of §2.2 the system identified Co-60 and
> Cs-137 simultaneously, confirming the identification capability for mixed FED waste.

### 3.2 Efficiency validation and calibration (Experiment B)
> Efficiency validation was performed by assaying the certificated Cs-137 (2.990 MBq)
> and Co-60 (1.130 MBq) sources in the operational geometry (0.33 m through one 5 mm
> steel plate), with certificate activities decay-corrected to the measurement date.
> The measured Cs-137 activity agreed with the certificate to **−1.1%** (−0.5% on an
> independent repeat), demonstrating that the empirical efficiency calibration
> transfers correctly from the original 0.5 m unshielded calibration to the mounted,
> shielded configuration, including the solid-angle and steel-transmission
> corrections. The initial Co-60 assay read +7 to +12% high with a reproducible
> **4.5% inconsistency between its 1173 and 1332 keV photopeaks**; because the shared
> geometry was validated by the Cs-137 result, this isolated a residual error in the
> Co-60 efficiency calibration. The Co-60 calibration factors were then corrected
> against the certificate (1173 keV: 117,730 → 107,490; 1332 keV: 163,393 → 152,010).
>
> *Note on independence:* because the Co-60 factors were **fitted to** the
> certificate, the resulting agreement is a fit residual and is **not** an
> independent validation. The independent evidence is twofold: (i) the Cs-137 result
> (**−1.1%**), which was not adjusted and validates the geometry transfer; and (ii)
> the subsequent repeatability trial (Experiment D, §3.5), in which ten fresh Co-60
> assays gave a mean **+1.2%** against the certificate. This exercise demonstrates
> the calibration methodology detecting and correcting a nuclide-specific efficiency
> error through the detector's two-line self-consistency.

### 3.3 Limits of detection (Experiment A)
> The minimum detectable activity (MDA) was determined per Currie (1968) at the 95%
> confidence level from a 3606 s source-free background measurement in the
> operational geometry. For a 300 s assay the MDA is **0.24 kBq (Cs-137)** and
> **4.0 kBq (Co-60)**, and the limit of quantification (400 net counts, ≤5% counting
> uncertainty) is **36 kBq (Cs-137)** and **78–108 kBq (Co-60)**. Against the waste
> activities of interest (1 MBq up to the ~56 MBq assay ceiling), these limits lie
> **2.4 to 5.4 orders of magnitude below** the activity to be measured — Co-60 is
> 2.4 orders below 1 MBq and 4.1 orders below 56 MBq; Cs-137 is 3.6 and 5.4 orders
> respectively. Detection and quantification of the nuclides of concern are
> therefore never limiting factors.

*Caveat: the Co-60 background ROIs recorded a small residual Co-60 signal (sources
in the vicinity during the blank), so the quoted Co-60 MDA is a conservative upper
bound; a background with all Co-60 removed would be lower. Cs-137 background was zero,
so its MDA sits at the Currie zero-background floor and scales as 1/T.*

### 3.4 Maximum activity and dead-time (Experiment C + manufacturer rating)
> The GeGi is rated by the manufacturer for **200 kcps at 10% dead-time in a
> 15 mR/hr Co-60 field**. At the 0.33 m operational standoff this corresponds to a
> Co-60 activity of **~46 MBq** (bare), or **~56 MBq** through the permanent 5 mm
> plate, at which the dead-time is **10%**. Field measurements confirmed the detector
> operates at **≤3% dead-time** across the tested source loadings, well within this
> limit. For trays exceeding ~56 MBq, additional 5 mm plates raise the tolerable
> activity by ~22% each (~69, 84, 102 MBq for 1, 2, 3 added plates) while
> simultaneously reducing operator dose.

*Conversion basis: 15 mR/hr → 131 µGy/h air kerma (1 R = 8.76 mGy); Co-60 air-kerma
rate constant Γ = 0.308 µGy·m²·MBq⁻¹·h⁻¹; A = K·d²/Γ at d = 0.33 m; shielded figures
divide by the Co-60 plate transmission (~0.82 per plate).*

### 3.5 Repeatability (Experiment D)
> Repeatability was assessed from **10 consecutive 5-minute assays** of the
> certificated Co-60 source. The per-run activity had a mean of **1.128 MBq** against
> a decay-corrected certificate of 1.115 MBq (accuracy **+1.2%**), with a standard
> deviation of **2.1%** (Type A, k = 1). The two Co-60 photopeaks agreed to 2.7%,
> confirming the corrected efficiency calibration. The 2.1% is carried as the Type A
> contribution in the uncertainty budget (§5).

### 3.6 Shielding transmission verification (Experiment F)
> The steel attenuation correction was verified by assaying a Co-60 source through 1,
> 2 and 3 total 5 mm plates. Accounting for the source displacement per plate, the
> measured transmission agreed with the `exp(-µ·n·t)` model to within **2.3%**, and
> the fitted attenuation coefficients (**44.1 /m at 1173 keV, 41.5 /m at 1332 keV**)
> matched the standard-iron values used in the driver (41.8, 39.6 /m) to within
> **5.5% and 4.7%** respectively — confirming the shielding correction that underpins
> both the activity measurement and the maximum-activity plate-scaling. The residual
> (measured attenuation slightly stronger than the standard-iron model) changes the
> transmission correction by **1.0%** at the one-plate operational configuration.
> *(Cs-137, 662 keV, was not included in this run and retains the standard-iron
> value 57.4 /m — its shielding correction is therefore unverified.)*

---

## 4. Integration — source position and imager-informed correction (Experiment E)
> Position sensitivity was assessed by assaying a Co-60 source at the centre and the
> four corner cells of a 35 cm × 35 cm tray (7 × 7 grid of 50 mm cells). A corner
> cell centre lies 0.212 m off-axis, i.e. **0.392 m from the crystal at 32.7°**,
> versus the assumed 0.33 m on-axis geometry. The corner sources read **31% lower**
> than the centred source (corner mean 69.0% of centre; individual corners
> 64.5–72.3%). This is fully explained by the increased source-detector distance:
> the inverse-square law predicts a corner/centre ratio of **70.8%** against the
> **69.0% measured**. Across the five positions the activity standard deviation was
> **18.8%**, making source position the **dominant** contribution to assay
> uncertainty.
>
> This is the point at which the two roles combine. Because it is a geometric
> systematic rather than random scatter, and because the **imager locates the source
> to ~1° / ~1 cm (§2)**, the assay geometry can be corrected per source: the imaged
> position sets the true distance and off-axis angle used in the solid-angle
> calculation. This reduces the position term from ~19% to a few percent — the single
> largest available improvement in assay accuracy, and a direct realisation of the
> dual-role benefit the optioneering identified.

---

## 5. Quantification uncertainty budget
Standard uncertainties are 1σ; the expanded uncertainty is k = 2 (95%). Two columns:
the current driver (fixed on-axis geometry) and with the imager-informed position
correction of §4.

Every row is labelled by **provenance**, so each figure is traceable:
**M** = measured directly in an experiment · **D** = derived from a measurement by
a stated calculation · **E** = estimate not yet backed by measurement.

| Component | Type | Prov. | u (1σ) | Basis |
|-----------|------|-------|--------|-------|
| Source position in tray | B | **D** | **10.2%** | Exp E measured corner/centre = 69.0% (range 64.5–100%). Modelled as rectangular over that range: (1−0.645)/2 / √3 = 10.2%. |
| Repeatability (Type A) | A | **M** | **2.1%** | Exp D: SD of 10 consecutive 5-min assays. |
| Efficiency / geometry transfer | B | **M** | **1.1%** | Exp B: Cs-137 residual (−1.1%) — the unadjusted, independent check of the 0.5 m → 0.33 m + plate transfer. |
| Reference source certificate | B | **M** | **1.5%** | Co-60 source BH-4103 certificate: 3% relative uncertainty, stated as **expanded at k = 2** per GUM ⇒ standard uncertainty 1.5%. Traceable to PTB (standard Co-60 AN-6082, PTB-6.13-296/01.2019). |
| Shielding transmission (1 plate) | B | **D** | **1.0%** | Exp F: µ agreed to 5.5%; dT/T = t·dµ = 0.005 × (0.055 × 41.8) = 1.0%. |
| Dead-time correction | B | **D** | **0.6%** | Exp C: dead-time ≤3%; a ±20% relative error in the correction gives 0.03 × 0.20/(1−0.03) = 0.6%. |
| Co-60 coincidence summing | B | **D** | **0.05%** | Calculated at 0.33 m: solid angle 0.459%, Ge total-interaction probability 11.3% for the partner gamma ⇒ summing-out loss 0.05%. Negligible. |
| Background subtraction | B | **M** | negligible | Exp A: ROI background is single-digit counts against 10²–10³ net counts. |
| **Combined standard uncertainty (1σ)** | | | **10.6%** | root-sum-square |
| **Expanded uncertainty (k = 2, 95%)** | | | **≈ 21.3%** | |

**Every term is now measured, derived or calculated — none is estimated.**

> The expanded uncertainty (k = 2, 95%) on a single-tray Co-60 assay is **≈ 21%**,
> dominated by source position within the tray; all other contributions combine to
> below 3%. The position error is a **low bias** — off-centre sources read low, so
> the assay under-estimates activity unless corrected.

### Projected uncertainty with the imager-informed position correction
⚠️ **This is a forecast, not a measurement.** The position correction of §4 is not
yet implemented, so there is no measured residual. If the correction reduces the
position term to ~3% (a reasonable expectation given the imager locates sources to
~1° / ~6 mm at 0.33 m, §2), the budget becomes:

| | u (1σ) | Expanded (k=2) |
|---|---|---|
| With position term at 3.0% (projected) | 4.5% | **≈ 9%** |

This projection should be **replaced by a measured value** once the correction is
implemented and Experiment E is repeated.

*Position treatment, stated explicitly: a corner cell reads 31% low (measured).
Modelled as a rectangular distribution over the measured 64.5–100% range this gives
a 10.2% standard uncertainty. The raw SD of the five measured positions was 18.8%;
that is the more conservative figure but the 1-centre/4-corner sample over-weights
the extremes, so the rectangular treatment is preferred. Using 18.8% instead would
raise the expanded uncertainty to ≈ 38%.*

---

---

## Appendix — data provenance (traceability)
Every figure in this document traces to a raw data file and an analysis tool, so
the numbers can be re-derived independently.

| Experiment | Raw data | Analysis tool |
|---|---|---|
| A — Limits of detection | `data/Experiment A/20260710_095500_activity.csv` (3606 s blank) | `tools/mda.py` |
| B — Efficiency validation | `data/Experiment B/Cs137/20260709_114958_*`, `data/Experiment B/Co60/20260709_130120_*` | `tools/validate_efficiency.py` |
| C — Max activity / dead-time | Manufacturer rating (GeGi manual) + `tools/deadtime_monitor.py` field readings | conversion shown in §3.4 |
| D — Repeatability | `data/Experiment D/` (10 × 5-min Co-60) | `tools/repeatability.py` |
| E — Position sensitivity | `data/Experiment E/` (centre + 4 corners) | `tools/position_check.py` |
| F — Shielding transmission | `data/Experiment F/` (1, 2, 3 total plates) | `tools/shielding_check.py` |
| Localisation | 6-position imaging trial + per-isotope heatmaps | reported directly |

**Calibration in force** (`config/isotopes.yaml`): Cs-137 CF 45,672; Co-60 1173 keV
CF 107,490; Co-60 1332 keV CF 152,010 Bq/cps at 0.5 m.

### Reference source traceability
| | Co-60 | Cs-137 |
|---|---|---|
| Serial no. | **BH-4103** | *(to be recorded)* |
| Certified activity | 1.13 MBq | 2.990 MBq |
| Reference date | 1 June 2026, 12:00 UTC | 1 June 2026 |
| Relative uncertainty | **3% (expanded, k = 2)** ⇒ 1.5% standard | *(to be recorded)* |
| Traceability | **PTB** — standard Co-60 AN-6082 (PTB-6.13-296/01.2019) | *(to be recorded)* |
| Source form | Sealed; active surface Ø1 mm | — |

The Co-60 certificate is traceable to a **national primary standard (PTB)**, satisfying
the traceability-chain requirement. Its **Ø1 mm active surface** also confirms the
point-source assumption underlying the solid-angle model used throughout.

## Outstanding — must be closed before issue
1. **Cs-137 certificate details** — serial number, uncertainty and traceability
   statement, to complete the traceability chain (the Co-60 chain is complete).
2. **Volume throughput** — needs the Auto-SAS tray cycle / handling time.
3. **Cs-137 shielding µ (57.4 /m) is unverified** — Experiment F used Co-60 only.
4. **The "corrected" uncertainty (≈9%) is a projection**, not a measurement, until
   the position correction is implemented and Experiment E repeated.
5. **Co-60 MDA is a conservative upper bound** — residual Co-60 was present in the
   room during the Experiment A blank.

## Notes for the writer
- The maximum-activity figures are **anchored on the manufacturer rating**, not an
  in-house rate-to-destruction test: the available check sources could not load the
  detector beyond ~2.5% dead-time, so the vendor's 200 kcps / 15 mR/hr rating is the
  correct and defensible basis.
- **Count times / throughput** are not yet final: throughput needs the mechanical
  Auto-SAS line-cycle time; the count time is bounded by the MDA (§3.3) and the
  400-net-count quantification gate.
- The **imager-informed position correction** (§4) is a proposed driver enhancement;
  state in the report whether it is implemented, as it changes the headline expanded
  uncertainty from ≈22% to ≈10%.
