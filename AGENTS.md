# Project Memory

## Python Code Location

- All existing and future Python code for this project should live under
  `python/`.
- If new Python scripts, modules, packages, or generated `.py` files are
  created, place them in `python/` unless the user explicitly asks for a
  different location.

## GitHub Push Target

- This project is a private GitHub repository.
- Default remote: `origin`
- Remote URL: `git@github.com:zzh158shallow-tech/SDITT_PY.git`
- Default branch: `main`
- Use the local SSH key at `~/.ssh/id_ed25519_github` for GitHub access.
- The repo is configured with:
  - `core.sshCommand=ssh -i ~/.ssh/id_ed25519_github -o IdentitiesOnly=yes`

## Normal Push Workflow

1. Check changes:
   - `git status -sb`
2. Commit intentionally:
   - `git add <files>`
   - `git commit -m "<message>"`
3. Push to GitHub:
   - `git push -u origin main`

Do not force-push unless the user explicitly asks for it.

## Large And Private Data

Large runtime data files are intentionally ignored and should not be committed
to GitHub unless the user explicitly asks to set up a separate storage strategy
such as Git LFS, private object storage, or an external transfer method.

Known ignored examples include:

- `SDITT-RW-FT-250728/Mat_FT_S8b.mat`
- `SDITT-RW-FT-250728/Mat_FT_CN18_T4.mat`
- `SDITT-RW-FT-250728/Pre_CRH380A_009_Face_V350_zgygPf2_*.mat`
- `SDITT-RW-FT-250728/CRH380A_009_Face_V350_zgygPf2.mat`
