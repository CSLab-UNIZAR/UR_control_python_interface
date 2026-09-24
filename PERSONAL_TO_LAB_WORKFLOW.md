# Personal-to-lab publication workflow

This repository is the personal development repository for the project. It is
where experimental work, work in progress, and changes intended for later
publication are developed.

The complementary lab repository is:

- <https://github.com/CSLab-UNIZAR/UR_control_python_interface>

The two GitHub repositories are independent repositories, not GitHub forks.
They share Git history so that selected personal changes can be merged into the
lab repository cleanly. Changes made in the lab repository are not
automatically merged back into this personal repository.

## Repository roles

- `personal`: the normal development remote and the default push destination.
- `lab`: the curated CSLab-UNIZAR repository used by the laboratory and
  students.
- `main`: the personal development branch. It tracks `personal/main`.

Check the configuration with:

```bash
git remote -v
git config --get remote.pushDefault
git branch -vv
```

The expected remote URLs are:

```text
personal  https://github.com/nachocz/UR10_contro_python_interface_campero.git
lab       https://github.com/CSLab-UNIZAR/UR_control_python_interface.git
```

## Configure a fresh personal clone

A fresh clone normally calls its only remote `origin`. Configure it as follows:

```bash
git remote rename origin personal
git remote add lab https://github.com/CSLab-UNIZAR/UR_control_python_interface.git
git config remote.pushDefault personal
git branch --set-upstream-to=personal/main main
```

## Normal personal development

Develop and test changes on branches in this repository. Merge the changes
that belong on personal `main`, then push normally:

```bash
git switch main
git pull --ff-only personal main
git push
```

Because `remote.pushDefault` is `personal`, an unqualified `git push` does not
publish anything to the lab repository.

## Publish personal changes to the lab

Publish through a temporary branch based on the current lab `main`. This keeps
lab-only work in the lab repository and avoids merging it into personal
`main`.

First make sure all intended personal work is committed and pushed. Then:

```bash
git fetch personal
git fetch lab
git switch -c publish/YYYY-MM-DD-short-description lab/main
git merge --no-ff personal/main
```

Resolve any conflicts and run the relevant tests on the publication branch.
Push that branch to the lab repository:

```bash
git push -u lab publish/YYYY-MM-DD-short-description
```

Open a pull request in the lab repository from the publication branch into
`main`, review the resulting diff, wait for required checks, and merge it there.
Do not push directly to lab `main`.

After the lab pull request has been merged:

```bash
git switch main
git branch -d publish/YYYY-MM-DD-short-description
```

The remote publication branch can be deleted from the GitHub pull-request page
after merge.

## Direction of synchronization

The intended flow is:

```text
personal/main -> lab publication branch -> pull request -> lab/main
```

Do not run `git merge lab/main` or `git pull lab main` while on personal
`main`. Lab changes should move back to the personal repository only after an
explicit decision that they belong in both projects.

Do not use a recurring `git push --mirror`: it could overwrite or remove
lab-only branches and references.
