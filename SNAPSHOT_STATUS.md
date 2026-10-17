# Frozen Clean edition snapshot — 2026-10-17

This branch preserves the existing local Clean edition. It is separate from the current original-project research mainline. No new original-project physics, rigid-tip refinement or research change was adopted into Clean by this synchronization.

The segment-center phase correction was already approved in the local edition. The retained three epoch-24 policies and August evidence predate that correction; their old performance does not recertify the current controller. The P13 closed-loop evaluation matrix remains incomplete. See `PI_HANDOFF.md` and `docs/07_ORGANIZATION_AND_INTEGRITY.md`.

Every file in the existing release lock was copied and checked against that lock without regenerating it. Large generated videos/outputs and student workspace state remain local. Isaac Lab remains an external dependency; this source synchronization did not run a new simulation or certify portability on another machine.
