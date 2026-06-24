---
name: shooter-sotm-design-intent
description: Intentional design of the shooter/turret/SOTM firing model — do not "fix" these as bugs
metadata:
  type: feedback
---

The shooter/turret/SOTM ready-to-shoot model is deliberately gate-light. Do NOT add encoder/velocity gates as "bug fixes":

- **Turret is assumed always on target** (`Turret.alwaysAtTarget` true). They do NOT gate firing on the turret encoder reaching the target angle. `reachedTarget()`'s tolerance branch is effectively unused; leave it alone.
- **Hood angle is solved from the flywheel's CURRENT measured (smoothed) RPM**, not the LUT target RPM. The flywheel changes RPM slowly, so solving for current RPM keeps the shot valid at whatever speed the wheel has right now and lets it fire mid-spin-up. The LUT RPM is only the setpoint the wheel is driven toward. SOTM flight time / lead also use current RPM.
- **`Shooter.readyToShoot()` = `active && shotPossible`** (a valid hood solution exists). Do NOT add a "wheel must reach target velocity" check — that defeats the slow-flywheel design above.

**Why:** I once "fixed" all three (velocity-ready gate, LUT-target-RPM hood solve, turret reachedTarget abs) and was told I missed the point — they're intentional.

**How to apply:** When improving SOTM, focus on the AIM/LEAD math (pose+velocity prediction to release, decel-to-stop handling, turret servo-delay lead), not the firing gates. See [[ditto-sotm-improvements]].
