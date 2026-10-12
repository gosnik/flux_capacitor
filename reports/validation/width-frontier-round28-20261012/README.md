# Round28 — Last connection width feasibility diagnostic

Round27 has one physical open /TCK_TXD_A. Existing reference uses0.20mm tracks, while scaffold searches0.25mm; project minimum is0.09mm. Test0.25mm control and0.20mm candidate against identical final saved copper, unchanged clearance and0.60/0.30mm vias,50k expansions. This is an isolated width-feasibility experiment, not a production rule change or benchmark promotion.

### Result and decision
At retained Round27 copper,0.25 and0.20mm both exhaust50k expansions and leave1 open;0.20mm also exhausts200k. Removing only generated copper makes both widths accept the original coarse path directly at0 expansions. Therefore narrower width alone does not solve the dynamic interaction; do not promote a width heuristic or larger global budget.

Reversing the request at0.25mm reaches search_exhausted after479 expansions versus50k forward;0.20mm reverse exhausts after5528. It attributes a compact target-side grid pocket to dynamic owners9,90,97,98 and one static pad. This does not prove continuous infeasibility outside sampled attachment/grid states, but sharply improves repair evidence compared with a forward budget exhaustion. Next test connected repair against those actual target-side blockers before changing routing policy.

Research: [Holte et al., Bidirectional Search That Is Guaranteed to Meet in the Middle](https://ojs.aaai.org/index.php/AAAI/article/view/10436) demonstrates that search direction/frontier scheduling materially affects explored work and provides a framework for comparing bidirectional/A* searches. This experiment only reverses a bounded existing search; no MM optimality/completeness guarantee is claimed for Flux's sampled, history-bearing routing states.
