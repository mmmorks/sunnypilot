#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Configuration
LOCAL_BRANCH="staging-merged"  # Your local branch name
UPSTREAM_REMOTE="upstream"    # The name of your upstream remote
UPSTREAM_BRANCH="staging-c3-new"  # The upstream branch to track
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

# Find the common base commit between our branch and upstream
# This might be empty if histories are completely unrelated
BASE_COMMIT=$(git merge-base $LOCAL_BRANCH $UPSTREAM_REMOTE/$UPSTREAM_BRANCH 2>/dev/null || echo "")

# Create a temporary branch with our current changes
echo -e "${YELLOW}Creating temporary branch $TEMP_BRANCH to save current changes...${NC}"
git branch -D $TEMP_BRANCH 2>/dev/null || true
git checkout -b $TEMP_BRANCH

# Identify our custom commits
if [ -z "$BASE_COMMIT" ]; then
    # If histories are unrelated, we need to identify our custom commits differently
    echo -e "${YELLOW}Unrelated histories detected. Identifying custom commits...${NC}"
    
    # Create a new branch based on the latest upstream
    echo -e "${YELLOW}Creating new branch based on $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"
    git checkout -b new-base $UPSTREAM_REMOTE/$UPSTREAM_BRANCH
    
    # Cherry-pick our custom commits
    echo -e "${YELLOW}Applying custom commits on top of new base...${NC}"
    
    # Get a list of commits that are in our branch but not in upstream
    # This approach works even with unrelated histories
    CUSTOM_COMMITS=$(git log --format="%H" $TEMP_BRANCH --not --remotes=$UPSTREAM_REMOTE)
    
    # Apply our custom commits in order (oldest first)
    for COMMIT in $(echo "$CUSTOM_COMMITS" | tac); do
        if ! git cherry-pick $COMMIT; then
            echo -e "${RED}Cherry-pick failed for commit $COMMIT${NC}"
            echo -e "${YELLOW}You'll need to resolve conflicts manually.${NC}"
            echo -e "${YELLOW}After resolving conflicts, run:${NC}"
            echo -e "  git cherry-pick --continue"
            echo -e "  git branch -D $LOCAL_BRANCH"
            echo -e "  git branch -m new-base $LOCAL_BRANCH"
            echo -e "  git branch -D $TEMP_BRANCH"
            exit 1
        fi
    done
    
    # Replace the old branch with our new one
    git branch -D $LOCAL_BRANCH
    git branch -m new-base $LOCAL_BRANCH
else
    # If histories are related, we can use rebase --onto
    echo -e "${YELLOW}Resetting $LOCAL_BRANCH to match $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"
    git checkout $LOCAL_BRANCH
    git reset --hard $UPSTREAM_REMOTE/$UPSTREAM_BRANCH
    
    # Rebase our changes on top of the updated branch
    echo -e "${YELLOW}Rebasing your changes on top...${NC}"
    if ! git rebase --onto $LOCAL_BRANCH $BASE_COMMIT $TEMP_BRANCH; then
        echo -e "${RED}Rebase failed. You'll need to resolve conflicts manually.${NC}"
        echo -e "${YELLOW}After resolving conflicts, run:${NC}"
        echo -e "  git rebase --continue"
        echo -e "  git checkout $LOCAL_BRANCH"
        echo -e "  git reset --hard $TEMP_BRANCH"
        echo -e "  git branch -D $TEMP_BRANCH"
        exit 1
    fi
    
    # Move our branch pointer to the rebased changes
    git checkout $LOCAL_BRANCH
    git reset --hard $TEMP_BRANCH
fi

# Clean up
echo -e "${YELLOW}Cleaning up temporary branch...${NC}"
git branch -D $TEMP_BRANCH 2>/dev/null || true

echo -e "${GREEN}Update complete! Your $LOCAL_BRANCH branch now contains your changes on top of the latest $UPSTREAM_REMOTE/$UPSTREAM_BRANCH.${NC}"
