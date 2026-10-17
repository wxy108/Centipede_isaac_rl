# How to push this repo to GitHub

Remote: **https://github.com/wxy108/Omniverse_rendering_problem.git** (already created, currently empty).
The content here is complete. The push must run on your Windows PC (it needs your GitHub credentials, which
I can't supply). Pick one option.

## Option A (recommended) — init in place, then push
Open PowerShell:
```powershell
cd C:\Users\wxy22\Documents\isaaclab-rendering-notes
git init
git branch -M main
git add -A
git commit -m "Isaac Lab rendering notes + PyVista offline renderer guide"
git remote add origin https://github.com/wxy108/Omniverse_rendering_problem.git
git push -u origin main
```

## Option B — clone from the verified bundle, then push
I wrote a verified single-file bundle: `C:\Users\wxy22\Documents\isaaclab-rendering-notes.bundle`.
```powershell
cd C:\Users\wxy22\Documents
git clone isaaclab-rendering-notes.bundle Omniverse_rendering_problem
cd Omniverse_rendering_problem
git remote set-url origin https://github.com/wxy108/Omniverse_rendering_problem.git
git push -u origin main
```

## Notes
- `.gitignore` already excludes renders/binaries (`*.mp4 *.npz *.png ...`) — this is a notes repo.
- The five files in `scripts/` are reference copies; they import parent-project modules and aren't runnable
  standalone (see `PROVENANCE.md`).
- If `git push` says the remote already has commits, run `git pull --rebase origin main` first, then push.
