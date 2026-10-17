# Source synchronization — 2026-10-17

The previous GitHub tree was substantially older than local research. This update copies current controllers, environment code, selected tracked support scripts, tests, model assets, the current `mainline` source/configuration/receipts and the latest selected static figures. Hundreds of unrelated legacy diagnostic scripts, raw trajectories, runtime environments, caches and videos are not promoted into this source snapshot.

`LOCAL_SOURCE_MANIFEST.json` identifies each copied original file, its source path and SHA-256. The original workspace README is retained in `provenance/LOCAL_WORKSPACE_README.md`; the root README is a new navigation/status document. Earlier GitHub contents are preserved by the pre-sync tag and Git history, including the rendering source branch.

## Runtime boundary

This remains a research snapshot tied to the existing Isaac environment. Several frozen launchers and pin manifests refer to historical `X:/Projects/_archive/Codex`, `X:/Data` or original `C:/Users/...` inputs. The local environment and those frozen dependencies remain necessary. Source synchronization does not make every historical launcher portable or certify a fresh-machine installation. Historical instructions to continue/resume do not authorize a new run.

The old tracked support scripts are retained as compatibility code. `mainline/README.md` and the top of `PROJECT_STATE.md` determine the current scientific entry point. Large result matrices and raw pose/contact arrays remain local; the final receipt, failure accounting, analysis source and selected static figures preserve the state without pretending to be a full data backup.

## Checks performed

- All copied Python sources were compiled for syntax without importing/running the simulation.
- Source copies were hashed and checked against their originals.
- The existing final receipt accounts for 4,055 planned conditions as 3,553 complete plus 502 failed.
- Existing scientific results are reported with their original evidence limits; no new physical validation is claimed.

Per-condition expanded input/PD matrices and full native readbacks remain local and are listed in the source manifest. Some frozen launchers require these original inputs; this is a curated source/status snapshot, not a complete runtime/data distribution.
