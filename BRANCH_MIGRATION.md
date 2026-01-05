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

To complete the migration on GitHub (requires admin permissions):

### Step 1: Merge this PR into main
1. Review and merge this pull request into the `main` branch
2. This will add all content from `master` into `main`

### Step 2: Update default branch
1. Go to repository Settings → Branches
2. Change the default branch from `master` to `main`
3. GitHub will show a warning - confirm the change

### Step 3: Delete master branch
1. Go to the repository's branches page
2. Find the `master` branch and click the delete (trash) icon
3. Confirm the deletion

This ensures that:
- All new clones will use `main` by default
- Pull requests will target `main` by default
- The old `master` branch no longer exists

### Verification
After completing these steps, verify:
- `main` is the default branch in Settings → Branches
- `master` branch no longer appears in the branch list
- All code and history from `master` is present in `main`
