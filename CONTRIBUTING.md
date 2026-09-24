# Contributing and student workflows

This is the curated laboratory repository for the
`UR_control_python_interface` project. It is maintained under the
CSLab-UNIZAR organization for laboratory use, teaching, and student
contributions.

The maintainer's personal development repository is independent from this
repository. Stable changes may be brought here through reviewed pull requests;
changes made here are not automatically copied back to the personal repository.

## Ground rules

- Do not push directly to `main`.
- Make each change on a short-lived, clearly named branch.
- Open a pull request and describe what changed and how it was tested.
- Do not commit credentials, access tokens, private keys, personal data, or
  machine-specific secrets.
- Treat changes that can move a physical robot as safety-critical. Test them in
  simulation or in the approved lab environment and describe the test in the
  pull request.
- Keep generated files, build products, logs, recordings, and local workspaces
  out of commits unless they are intentionally part of the project.

## Students contributing changes back to this repository

Use the normal fork-and-pull-request workflow:

1. Fork this repository into your GitHub account.
2. Clone your fork and register this repository as `upstream`:

   ```bash
   git clone https://github.com/YOUR-USER/UR_control_python_interface.git
   cd UR_control_python_interface
   git remote add upstream https://github.com/CSLab-UNIZAR/UR_control_python_interface.git
   ```

3. Create a branch from an up-to-date `main`:

   ```bash
   git fetch upstream
   git switch main
   git merge --ff-only upstream/main
   git switch -c feature/short-description
   ```

4. Commit and push the branch to your fork:

   ```bash
   git push -u origin feature/short-description
   ```

5. Open a pull request from your fork into
   `CSLab-UNIZAR/UR_control_python_interface:main`.

## Students starting an independent project

If the new project should not be represented by GitHub as a fork, first create
an empty repository in your personal GitHub account. Then:

```bash
git clone https://github.com/CSLab-UNIZAR/UR_control_python_interface.git
cd UR_control_python_interface
git remote rename origin upstream
git remote add origin https://github.com/YOUR-USER/YOUR-PROJECT.git
git push -u origin main
```

This creates an independent GitHub repository while preserving compatible Git
history. The `upstream` remote remains available if you later choose to review
or merge lab updates:

```bash
git fetch upstream
git log --oneline --left-right main...upstream/main
```

Alternatively, use GitHub's **Use this template** button for a standalone copy
with new, unrelated history. A template-based project cannot directly merge
branches with this repository without first reconciling the unrelated
histories.

Before redistributing a variation, check the project-level license and the
licenses of all included third-party components.

## Maintainer workflow: publishing from the personal repository

The project owner maintains a clone with two remotes:

```text
personal  https://github.com/nachocz/UR10_contro_python_interface_campero.git
lab       https://github.com/CSLab-UNIZAR/UR_control_python_interface.git
```

Stable personal changes are published by creating a branch from the current
lab `main`, then merging personal `main` into that branch:

```bash
git fetch personal
git fetch lab
git switch -c publish/YYYY-MM-DD-short-description lab/main
git merge --no-ff personal/main
git push -u lab publish/YYYY-MM-DD-short-description
```

The owner then opens and reviews a pull request from the publication branch
into lab `main`. Starting from `lab/main` preserves student or lab-only changes
without copying those changes into personal `main`.

Do not maintain this repository with a recurring mirror push. A mirror can
overwrite or delete lab-only references.

## Maintainer workflow: lab-specific changes

Changes that belong only to the lab repository should be developed directly on
a lab branch:

```bash
git fetch lab
git switch -c lab/short-description lab/main
# edit, test, and commit
git push -u lab lab/short-description
```

Open a pull request into lab `main`. Lab-specific changes are copied to the
personal repository only after an explicit decision by the owner.

## Pull-request checklist

- The change has a focused purpose and an explanatory description.
- Relevant tests or manual robot/simulation checks are documented.
- No credentials, large generated artifacts, or unrelated changes are present.
- Documentation is updated when behavior or setup changes.
- Existing third-party notices and license files are preserved.
