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

# Create a temporary branch with our current changes
echo -e "${YELLOW}Creating temporary branch $TEMP_BRANCH to save current changes...${NC}"
git branch -D $TEMP_BRANCH 2>/dev/null || true
git checkout -b $TEMP_BRANCH

# Switch back to our main branch and reset it to match upstream
echo -e "${YELLOW}Resetting $LOCAL_BRANCH to match $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"
git checkout $LOCAL_BRANCH
git reset --hard $UPSTREAM_REMOTE/$UPSTREAM_BRANCH

# Merge our changes back on top
echo -e "${YELLOW}Merging your changes back on top...${NC}"
if git merge $TEMP_BRANCH --no-commit; then
    echo -e "${GREEN}Successfully merged changes from $TEMP_BRANCH${NC}"
    
    # Check if there are any conflicts
    if git diff --name-only --diff-filter=U | grep -q .; then
        echo -e "${RED}There are merge conflicts. Please resolve them manually.${NC}"
        exit 1
    fi
    
    # Commit the merge
    git commit -m "Merge local changes on top of latest $UPSTREAM_REMOTE/$UPSTREAM_BRANCH"
    echo -e "${GREEN}Successfully updated $LOCAL_BRANCH with latest upstream changes${NC}"
else
    echo -e "${RED}Merge failed. You'll need to resolve conflicts manually.${NC}"
    echo -e "${YELLOW}Your original changes are preserved in the $TEMP_BRANCH branch.${NC}"
    exit 1
fi

# Clean up
echo -e "${YELLOW}Cleaning up temporary branch...${NC}"
git branch -D $TEMP_BRANCH

echo -e "${GREEN}Update complete! Your $LOCAL_BRANCH branch now contains your changes on top of the latest $UPSTREAM_REMOTE/$UPSTREAM_BRANCH.${NC}"
