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
- Major and minor each range from 0 to 9. **Required (user, 17 November 2026):** the build field ranges from 0 to 99 without zero padding, as in every owned repository. It counts up (`1.1.4.9 -> 1.1.4.10`) and overflow carries (`1.1.4.99 -> 1.1.5.0`, `1.1.9.99 -> 1.2.0.0`, `1.9.9.99 -> 2.0.0.0`); a build field of 100 is refused. This supersedes the earlier one-digit build field; no build was accepted under it.
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

## Agent workflow

These rules apply to the orchestrator and its subagents. The suite `AGENTS.md` holds the same common rules, and every owned repository repeats them. The default branch of this repository is `main`; the suite pins the product branch `native/planegcs-1.1.4`, which carries the SESKAFEA changes. The [owned fork contract](#owned-fork-contract) above stays in force: upstream is read-only, every pull request targets this fork explicitly, and PlaneGCS sources keep upstream naming and style.

- **Required (user, 16 November 2026), main stays current:** once a pull request merges, its work must be on the default branch. When a PR merges into an intermediate or stacked branch, a merge agent carries that branch into the default branch at once: it merges with `--no-ff`, resolves conflicts by intent, verifies, pushes, and merges the merge PR under the user's standing authorisation for this step. Agents never merge a PR the user has not merged. Open stacked PRs against the default branch where possible, because `Closes #n` links and closes issues only for PRs into the default branch.
- **Required (user, 16 November 2026), subagent models:** subagents run Claude Opus at high effort (implementation, merges, code reviews) or Claude Fable at medium effort (lean reviews, research, tracker and evidence checks). The orchestrator works as chief of staff: it hands work to background subagents through context pointers (brief files, commits, issues) and improves the agent environment before feature work, with no workarounds. The 17 November council rules below replace this rule's model assignments for PR reviews and research councils.
- **Required (user, 16 November 2026), skill routing:** use `planning-and-task-breakdown` for features and new implementations (the broad points of the implementation roadmap) before issues are written, `multi-opinion` for architecture decisions and for `research`, and `issue-and-pr-based-development` for the tracker, branch and PR flow.
- **Required (user, 17 November 2026), work per issue or PR:** each issue or PR gets an incremental build of the whole tree with no warnings, focused tests for the modules, gates and fixtures the change touches, mutation evidence that each new test catches its defect, and a council review. Then it merges. This fork has no build of its own: the incremental build and the focused tests are the consumer's (the SESKAFEA frontend geometry worker) on the fork commit. A PR runs no full suite and no build tier, and nothing is rerun after a clean merge of the default branch. This supersedes the 16 November "test scope per step" rule and the 11 November per-build tier rule, and the rebuild sentence of the merge verification reuse rule for clean merges of the default branch.
- **Required (user, 17 November 2026), build number per merged PR:** each merged PR that changes source, tests, build files, tools or dependencies gets its own build number and a build record that holds its focused evidence: the incremental build, the focused tests, the mutation evidence and the council review. A PR that changes no build-affecting file (only docs, notes or evidence) takes no build number, with the `git diff --name-only` output as evidence; documentation-only work does not advance a build number. This fork allocates the number under [Internal version](#internal-version-proposed): only a qualifying build advances the counter, and the build field runs 0–99.
- **Required (user, 17 November 2026), phase-end full run:** the consumer's full acceptance gates on the fork commit, including the full consumer corpus for a change to numerical linear algebra, and the build or acceptance run run once per batch, after the implementation of the phase is done, on the batch head. Their failures are fixed then. The record of that run lists the PR build numbers it covers. Every full run keeps its complete logs. This rule saves time only. It does not relax any numerical or acceptance check at the phase-end run.
- **Required (user, 17 November 2026), build trees:** a build tree is kept until its branch merges and is rebuilt incrementally.
- **Required (user, 17 November 2026), council review per PR:** every PR gets a council review before it merges, with scores out of 10 and a "Council review" table in the PR body. A normal PR is reviewed by one Claude Haiku 5.5 (xhigh) and one Gemini 3.8 Flash (High). A big or risky PR is reviewed by one Claude Haiku 5.5 (xhigh) and two Gemini 3.8 Flash (High); big or risky means a vendor or fork patch (any change to PlaneGCS source in this repository), a whole package, a storage-format change, or a solver or numerics change. This replaces the 11 and 12 November rule of one independent code review per PR. Issue acceptance criteria plus tests remain the contract, with no prose contract freezes. The council review is the independent review required under [Change rules](#change-rules).
- **Required (user, 17 November 2026), research council:** a research council is three Gemini 3.8 Flash (High) and one Claude Opus (high). Decision councils stay as before.
- **Required (user, 16 November 2026), merge verification reuse:** a merge that changes no build-affecting file (only verification evidence, notes or docs) reuses the verified parent's results, with the `git diff --name-only` output as evidence. Any change to source, tests, build files, tools or dependencies is rebuilt and verified.

## Shared instruction entry points

`CLAUDE.md`, `GEMINI.md` and `ANTIGRAVITY_INSTRUCTIONS.md` reference `@AGENTS.md`; keep substantive rules here. Upstream's README and contribution rules remain in force except where this file supersedes them.
