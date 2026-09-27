# Fleet sizing (by hand)

* The farthest points from the center are the far corners, 1185.6 m away.
* A plain relay chain with hops of at most 90 m needs 14 hops, so 13 relays (each hop is 84.7 m, more than the 20 m separation: True).
* Surveyor lanes are 71.4 m apart (less than 2 x 40 m footprint), so 7 lanes cover one band: 7 surveyors.
* The spine has fixed relay stations and only the ones behind the column are used. At most 20 slots are needed at once, 13.6 on average.
* Each slot needs active_frac x cycle / on_station UAVs, where on_station = 1200 - t_out - t_home - 60 margin - 60 handover overlap and cycle = battery used + swap.

| slot | active frac | t_out s | t_home s | on-station s/sortie | UAVs |
|---|---|---|---|---|---|
| R0 | 1.00 | 53 | 82 | 945 | 1.34 |
| R1 | 0.79 | 66 | 97 | 917 | 1.09 |
| R2 | 0.73 | 80 | 114 | 886 | 1.04 |
| R3 | 0.67 | 95 | 132 | 853 | 0.99 |
| R4 | 0.61 | 110 | 150 | 820 | 0.94 |
| R5 | 0.55 | 125 | 169 | 786 | 0.89 |
| R6 | 0.49 | 140 | 187 | 753 | 0.82 |
| R7 | 0.43 | 156 | 205 | 719 | 0.76 |
| R8 | 0.37 | 171 | 224 | 685 | 0.68 |
| R9 | 0.31 | 186 | 242 | 652 | 0.60 |
| R10 | 0.25 | 202 | 261 | 618 | 0.51 |
| R11 | 0.20 | 217 | 279 | 584 | 0.44 |
| R12 | 0.16 | 233 | 297 | 550 | 0.37 |
| S0 | 1.00 | 154 | 203 | 723 | 1.75 |
| S1 | 1.00 | 149 | 197 | 734 | 1.73 |
| S2 | 1.00 | 146 | 194 | 740 | 1.71 |
| S3 | 1.00 | 145 | 193 | 742 | 1.71 |
| S4 | 1.00 | 147 | 195 | 739 | 1.72 |
| S5 | 1.00 | 150 | 199 | 731 | 1.73 |
| S6 | 1.00 | 155 | 205 | 720 | 1.76 |

* UAVs needed for rotation (sum): 22.6
* Plus AP-Shield shadows: 3, plus spares for faults: 2
* Recommended N = 28

This estimate assumes a steady state. In the real 45-min mission all UAVs start charged and the last flights are short, so the simulated sweep over N in results/ decides the final N.
