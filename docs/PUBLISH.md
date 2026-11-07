# Publish this source tree on GitHub

The local export is prepared separately from the full handover archive. Publish only this repository directory. The preparation step does not create a GitHub repository, choose an account, upload data or configure a remote.

## Review locally

```bash
python tools/audit_release.py
python tools/validate_scenarios.py
python -m unittest discover -s tests -v
git status --short
```

Check the MIT license and third-party notices, project title and README. `.env`, `runs/`, `build/`, ULogs and ROS build products must stay untracked. The historical CSV/figure selection is suitable for the source tree; larger raw datasets can be distributed separately with their own manifest and release notes. Do not copy the full handover folder into Git.

## First commit and upload

Inside this repository, set your own Git author identity if needed. No identity is invented by the preparation script.

```bash
git add .
git diff --cached --stat
git commit -m "Prepare UAV farm coverage source release"
```

After you have authenticated GitHub CLI, one option is:

```bash
gh repo create uav-farm-coverage --public --source=. --remote=origin --push
```

Alternatively create an empty repository in the GitHub website, copy its exact URL, then use `git remote add origin YOUR_REPOSITORY_URL` and `git push -u origin main`. Replace the placeholder with your own repository URL; do not paste credentials into commands or committed files.

Once public, inspect the offline GitHub Actions run. The workflow validates Python/reference data; it does not run a full ROS/PX4/Gazebo fleet in GitHub Actions. Add the actual repository URL to `CITATION.cff` and release notes after publication. No DOI is assigned by this project preparation.

GitHub's [large-file guidance](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github) explains its regular Git file limit. This export checks for files at 50 MiB, below the 100 MiB regular-Git block threshold; no Git LFS is required for the selected contents.
