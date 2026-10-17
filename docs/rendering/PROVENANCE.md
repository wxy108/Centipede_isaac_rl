# Provenance

- Source project: Centipede Isaac Lab RL (`Centipede_IsaacLab_RL`).
- Notes span 2026-05-10 → 2026-06-22 (see dates in `notes_verbatim/` and script filenames).
- Organized chapters (01–05) written 2026-06-30 from a verified digest of the verbatim notes + the five scripts.
- Every quantitative claim is cited to a source file:line in the verbatim notes or the scripts.

## Open items — all RESOLVED 2026-06-30 (audit pass)
1. **Isaac Sim version → 5.0.0.** Run assets resolve to `Assets/Isaac/5.0/`; the launcher selects the 4.5 config
   only if a 4.5 install is detected (it wasn't). The env name `isaaclab_50` was correct; the "4.5 banner" was a
   red herring. Details + citations in `01_SYSTEM_AND_VERSIONS.md` §1.2.
2. **Teardown hang → ~5-6 min (RENDER jobs), watchdog force-kills at a fixed 10 s.** The "10-22 min" figure in the
   parent CLAUDE.md is a compaction error from "saved 10-22 min per launch" (a time-saved range), not a hang
   duration. See `03_PRECHECKS_AND_WATCHDOG.md` §3.6. (Recommend correcting parent CLAUDE.md:26.)
3. **Exact dependency versions** — the repo contains NO env dump, so only PyVista 0.48.4 and Python 3.11 are
   installed-confirmed; the rest are requirements pins (torch 2.7.0, gymnasium 1.2.0, Pillow 11.2.1, numpy<2) and
   trimesh/cv2/imageio/vtk have no recorded version. The authoritative fix is a one-time `pip freeze` of the
   `isaaclab_50` env — exact command in `01_SYSTEM_AND_VERSIONS.md` §1.3.

## Scripts are reference copies
The five files in `scripts/` are copies for reference. They import parent-project modules
(envs/, controllers/, IsaacLab/) and are NOT runnable standalone from this repo. To run them, place them back
in the parent project tree. They are included so the notes are self-contained and auditable.
