# AGENTS.md

Rules for people and coding agents working in this repository.

## Owned fork contract

This repository is the user's public fork [paulnamalomba/FreeCAD](https://github.com/paulnamalomba/FreeCAD) of FreeCAD. SESKAFEA uses only the PlaneGCS sketch solver extraction, `src/Mod/Sketcher/App/planegcs/`, inside its frontend geometry worker. The worker builds those members from an exact commit of this fork. The commit is pinned with the SHA-256 of its GitHub archive in the frontend's `cmake/FrontendDependencies.json`. All intentional PlaneGCS source changes for SESKAFEA belong here as reviewed commits, never as patch files hidden in consumer build scripts. Publish a fork commit before a consumer pins it.

**Required (user, 3 November 2026):** the fork is public and independently maintained, and upstream `FreeCAD/FreeCAD` is a read/fetch reference only:

- Keep upstream fetch-only. In local clones, set its push URL to an invalid value (for example `DISABLED-upstream-is-read-only`).
- Do not push branches or tags upstream, and do not open pull requests, issues or comments there.
- GitHub records this repository as a fork, so tools may default to the upstream base. Every push, and any pull request the user asks for, must explicitly target `paulnamalomba/FreeCAD`. Create pull requests only when the user asks.

### Branches

- `native/planegcs-1.1.4` starts from upstream FreeCAD 1.1.4, commit `4fd3bf320d9566a27e60069fc8387448aaa3a094`. Its upstream archive (`https://github.com/FreeCAD/FreeCAD/archive/4fd3bf320d9566a27e60069fc8387448aaa3a094.tar.gz`) has 91,812,296 bytes and SHA-256 `d381064aede10f7d9388bd78d2073d05f07b4da53f723940a16860e0c5fe446a`. The branch carries the SESKAFEA changes as separate commits.
- A later upstream release gets a new `native/planegcs-<release>` branch. Each change is re-applied and re-qualified there; a version number never implies compatibility.
- Never rewrite or force-push a commit that a consumer has pinned.

### Change rules

- **Required:** one reviewed change per commit. The message states the defect or limit, its source location, the remedy and the evidence that tested both the raw behaviour and the remedy.
- **Required:** SESKAFEA changes touch only `src/Mod/Sketcher/App/planegcs/` and the fork's own instruction and version files. Keep each change minimal and local.
- **Required:** existing PlaneGCS entry points keep their behaviour. New behaviour is reached through new entry points or switches that default to the upstream behaviour, so FreeCAD's own callers are unaffected. Keep upstream naming and style in PlaneGCS sources.
- **Required:** a change is adopted only after it has an independent review and the consumer's acceptance gates pass on the fork commit. A zero exit is never success evidence by itself. Changes that alter numerical linear algebra are re-qualified on the full consumer corpus before adoption.
- Keep FreeCAD's version files unchanged. The internal version below is separate.

### Internal version (Proposed)

`PLANEGCS_INTERNAL_VERSION` holds the fork's four-part identity, `<breaking>.<major>.<minor>.<build>`, seeded unbuilt at `1.1.4.0`.
- A build is qualifying when the consumer's acceptance builds this fork commit from its archive and passes its required checks. The first qualifying build advances the version to `1.1.4.1`.
- Major, minor and build each range from 0 to 9, and overflow carries (`1.1.4.9 -> 1.1.5.0`), as in the MS-MPI and OCCT forks.
- Failed, configure-only and documentation-only work does not advance it.
- Notes are Markdown, grouped under `.commits/<breaking>.<major>.<minor>.md`.
- The user may re-seed or change this scheme.

### Change register

| Commit subject | Files | Evidence |
| --- | --- | --- |
| Solve with caller-supplied redundancy | `planegcs/GCS.{h,cpp}` | SESKAFEA C1.4b PS1. Raw: X2 r2 peak 176.8 MiB on `limit-budget-two-1024` from the whole-sketch dense diagnosis inside `solve`, and `Converged`-at-start on the two compatible-redundant fixtures. Tests: `X2PatchTests.py` ps1 and `X2PlaneGcsPatchUnit.cpp` ps1-* |
| Diagnose with a sparse Jacobian and sparse QR | `planegcs/GCS.{h,cpp}` | SESKAFEA C1.4b PS2. Tests: `X2PatchTests.py` ps2 (lists equal to the upstream sparse-QR path on every fixture; size rule) and `X2PlaneGcsPatchUnit.cpp` ps2-* |
| Confirm diagnosed redundancy per connected component | `planegcs/GCS.{h,cpp}` | SESKAFEA C1.4b PS3. Tests: `X2PlaneGcsPatchUnit.cpp` ps3-* (one stalled component), `X2PatchTests.py` ps3 |
| Add a sparse DogLeg Gauss-Newton step | `planegcs/GCS.{h,cpp}`, `planegcs/SubSystem.{h,cpp}` | SESKAFEA C1.4b PS4. Dense `Jx`, `Jx_new` and `fullPivLu` need at least 1 GiB at 8,192 unknowns. The step is the quasi-definite system with one `delta` and no QR path (a per-row `delta` with a QR fallback, and a `J J^T` step before it, were replaced). Tests: `X2PlaneGcsPatchUnit.cpp` ps4-shipped-step (dense reference, twelve systems), `X2StepProbe.cpp`, `X2PatchTests.py` ps4 and stress (23 fixtures, solve at most 91 MiB, zero fallback calls) |

The tests live in the SESKAFEA frontend repository, branch `x2/planegcs-requal`, folder `verification/geometry/x2-solver/`.

## Shared instruction entry points

`CLAUDE.md`, `GEMINI.md` and `ANTIGRAVITY_INSTRUCTIONS.md` reference `@AGENTS.md`; keep substantive rules here. Upstream's README and contribution rules remain in force except where this file supersedes them.
