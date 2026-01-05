# Branch Migration: master → main

This repository has migrated from using `master` as the default branch to using `main`.

## What happened

- All content from the `master` branch has been merged into the `main` branch
- The `main` branch now contains all driver code, configuration files, and documentation
- The `master` branch should be deleted on GitHub to complete the migration

## For contributors

If you have local clones of this repository with the old `master` branch:

```bash
# Fetch the latest changes
git fetch origin

# Switch to main branch
git checkout main

# Update your local main branch
git pull origin main

# Optional: Delete your local master branch
git branch -d master
```

## For repository maintainers

To complete the migration on GitHub:

1. Go to repository Settings → Branches
2. Change the default branch to `main`
3. Delete the `master` branch from the repository

This ensures that:
- All new clones will use `main` by default
- Pull requests will target `main` by default
- The old `master` branch no longer exists
