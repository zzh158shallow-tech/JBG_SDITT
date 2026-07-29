# WRCP-Net A2R accepted-step continuity validation

## Implemented safeguards

- Previous accepted-step penetration is the only history anchor.
- All nonlinear iterations and retries within one step use the same anchor.
- Rejected/retried states never update history.
- Checkpoints save and restore accepted neural contact history.
- Ordinary teacher-envelope changes pass through unchanged.
- Changes above the traditional 99.5% rate envelope activate `alpha=0.3`
  under-relaxation and rate clipping.
- Contact patches retain identity through Hungarian assignment and branch
  hysteresis.
- Residual outputs use a 5 micrometre `tanh` bound and accepted-step slew limit.
- The runtime trace separately records base, residual, network, and final-used
  penetration values and limiter flags.
- A2R training includes 14,000 signed micron-scale perturbation pairs and a
  local difference/Jacobian consistency loss.
- Contact points are refined only inside neural candidate regions by continuous
  interpolation on the fixed profiles; no full-profile traditional fallback is
  used.

## Known discontinuity replay

| wheel/side | old maximum jump | guarded maximum jump |
| --- | ---: | ---: |
| FR-R1 contact location | 0.602 mm | 0.0063 mm |
| RR-R1 contact location | 1.152 mm | 0.0149 mm |
| FR-R1 penetration | about 49 micrometres | 1.18 micrometres |

The original 47.97 m branch reversal no longer occurs. Candidate-region
contact coordinates agree with the traditional teacher to about 0.000003 mm
in the replay window.

## 200-step coupled comparison

All runs use interval geometry, Chinese ballastless-track irregularity, seed
`20260716`, nominal `dt=1e-4 s`, and `cut_freq=50 Hz`.

| metric | traditional | A2G | A2R v1 | A2R continuity |
| --- | ---: | ---: | ---: | ---: |
| retry count | 0 | 16 | 9 | 0 |
| minimum accepted dt | 1.0e-4 | 5.0e-5 | 2.5e-5 | 1.0e-4 |
| mean iterations | 2.185 | 2.435 | 2.255 | 2.215 |
| total-force NRMSE | 0% | 36.29% | 18.56% | 1.99% |
| FF-L1 force NRMSE | 0% | 73.01% | 41.09% | 3.15% |
| FF-R1 force NRMSE | 0% | 41.66% | 37.54% | 4.63% |
| Cal wall time | 9.03 s | 48.43 s | 38.79 s | 28.40 s |

The continuity run completed all 200 Cal steps without a retry. A full-size,
full-mileage run is still required before production replacement is declared.
