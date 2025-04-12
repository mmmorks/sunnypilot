#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Configuration
LOCAL_BRANCH="staging-merged"  # Your local branch name
UPSTREAM_REMOTE="upstream"    # The name of your upstream remote
UPSTREAM_BRANCH="staging-c3-new"  # The upstream branch to track
ORIGIN_REMOTE="origin"        # Your fork remote
TEMP_BRANCH="temp-save-changes"  # Temporary branch to store your changes

# Print status messages in color
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting update process for $LOCAL_BRANCH based on $UPSTREAM_REMOTE/$UPSTREAM_BRANCH${NC}"

# Check if we're on the correct branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT_BRANCH" != "$LOCAL_BRANCH" ]; then
    echo -e "${YELLOW}Currently on branch $CURRENT_BRANCH, switching to $LOCAL_BRANCH${NC}"
    git checkout $LOCAL_BRANCH
fi

# Fetch the latest changes from upstream
echo -e "${YELLOW}Fetching latest changes from $UPSTREAM_REMOTE...${NC}"
git fetch $UPSTREAM_REMOTE

# Store the current HEAD of the local branch
echo -e "${YELLOW}Saving reference to current HEAD...${NC}"
LOCAL_HEAD=$(git rev-parse HEAD)

# Create a temporary branch with our current changes
echo -e "${YELLOW}Creating temporary branch $TEMP_BRANCH to save current changes...${NC}"
git branch -D $TEMP_BRANCH 2>/dev/null || true
git checkout -b $TEMP_BRANCH

# Create a new branch based on the latest upstream
echo -e "${YELLOW}Creating new branch based on $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"
git branch -D new-base 2>/dev/null || true
git checkout -b new-base $UPSTREAM_REMOTE/$UPSTREAM_BRANCH

# Cherry-pick our custom commits
echo -e "${YELLOW}Identifying and applying custom commits on top of new base...${NC}"

# Get a list of commits that are in our branch but not in upstream
# Skip the first commit which is known to conflict
echo -e "${YELLOW}Skipping the first commit (which conflicts) and applying the rest...${NC}"

# Get all custom commits
CUSTOM_COMMITS=$(git log --format="%H" $TEMP_BRANCH --not --remotes=$UPSTREAM_REMOTE)

# Convert to array and skip the first commit (which is the oldest when reversed)
COMMIT_ARRAY=($(echo "$CUSTOM_COMMITS" | tac))
COMMITS_TO_APPLY=("${COMMIT_ARRAY[@]:1}")  # Skip the first element

# Check if we have any commits to apply
if [ ${#COMMITS_TO_APPLY[@]} -eq 0 ]; then
    echo -e "${YELLOW}No custom commits found to apply (after skipping the first one).${NC}"
    echo -e "${YELLOW}Using the upstream branch directly.${NC}"
    
    # Replace the old branch with our new one
    git checkout new-base
    git branch -D $LOCAL_BRANCH
    git branch -m new-base $LOCAL_BRANCH
else
    # Apply our custom commits in order (oldest first, but skipping the first commit)
    for COMMIT in "${COMMITS_TO_APPLY[@]}"; do
        echo -e "${YELLOW}Applying commit: $(git log -1 --pretty=format:"%s" $COMMIT)${NC}"
        if ! git cherry-pick $COMMIT; then
            echo -e "${RED}Cherry-pick failed for commit $COMMIT${NC}"
            echo -e "${YELLOW}You'll need to resolve conflicts manually.${NC}"
            echo -e "${YELLOW}After resolving conflicts, run:${NC}"
            echo -e "  git cherry-pick --continue"
            echo -e "  git branch -D $LOCAL_BRANCH"
            echo -e "  git branch -m new-base $LOCAL_BRANCH"
            echo -e "  git branch --set-upstream-to=$ORIGIN_REMOTE/$LOCAL_BRANCH $LOCAL_BRANCH"
            echo -e "  git branch -D $TEMP_BRANCH"
            echo -e "  git push $ORIGIN_REMOTE $LOCAL_BRANCH --force"
            exit 1
        fi
    done
    
    # Replace the old branch with our new one
    git branch -D $LOCAL_BRANCH
    git branch -m new-base $LOCAL_BRANCH
    
    # Set up proper tracking with origin
    echo -e "${YELLOW}Setting up tracking with $ORIGIN_REMOTE/$LOCAL_BRANCH...${NC}"
    git branch --set-upstream-to=$ORIGIN_REMOTE/$LOCAL_BRANCH $LOCAL_BRANCH || {
        echo -e "${YELLOW}Remote branch $ORIGIN_REMOTE/$LOCAL_BRANCH doesn't exist yet.${NC}"
        echo -e "${YELLOW}After pushing, you can set tracking with:${NC}"
        echo -e "  git branch --set-upstream-to=$ORIGIN_REMOTE/$LOCAL_BRANCH $LOCAL_BRANCH"
    }
fi

# Clean up
echo -e "${YELLOW}Cleaning up temporary branch...${NC}"
git branch -D $TEMP_BRANCH 2>/dev/null || true

echo -e "${GREEN}Update complete! Your $LOCAL_BRANCH branch now contains your changes on top of the latest $UPSTREAM_REMOTE/$UPSTREAM_BRANCH.${NC}"
echo -e "${GREEN}The first commit from your original branch was skipped to avoid conflicts.${NC}"

# Provide instructions for pushing to origin
echo -e "${YELLOW}To push these changes to your fork, run:${NC}"
echo -e "  git push $ORIGIN_REMOTE $LOCAL_BRANCH --force"
echo -e "${YELLOW}Note: --force is required because the branch history has been rewritten.${NC}"
