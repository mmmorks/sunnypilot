#!/bin/bash
set -e  # Exit immediately if a command exits with a non-zero status

# Configuration
LOCAL_BRANCH="staging-merged"  # Your local branch name
UPSTREAM_REMOTE="upstream"    # The name of your upstream remote
UPSTREAM_BRANCH="staging"  # The upstream branch to track
OLD_UPSTREAM_BRANCH="staging-c3-new"  # Previous upstream branch (for identifying custom commits)
ORIGIN_REMOTE="origin"        # Your fork remote
TEMP_BRANCH="temp-save-changes"  # Temporary branch to store your changes

# Print status messages in color
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting update process for $LOCAL_BRANCH based on $UPSTREAM_REMOTE/$UPSTREAM_BRANCH${NC}"

# Check if we're in the middle of a cherry-pick operation
if [ -d ".git/sequencer" ] || git status --porcelain | grep -q "^UU\|^AA\|^DD"; then
    echo -e "${YELLOW}Detected ongoing cherry-pick operation. Checking status...${NC}"
    
    # Check if there are unresolved conflicts
    if git status --porcelain | grep -q "^UU\|^AA\|^DD"; then
        echo -e "${RED}There are still unresolved conflicts. Please resolve them first.${NC}"
        echo -e "${YELLOW}After resolving conflicts, run: git add . && git cherry-pick --continue${NC}"
        echo -e "${YELLOW}Then run this script again to continue with remaining commits.${NC}"
        exit 1
    else
        # No conflicts, continue the cherry-pick
        echo -e "${YELLOW}Continuing cherry-pick operation...${NC}"
        if git cherry-pick --continue; then
            echo -e "${GREEN}Cherry-pick continued successfully. Continuing with remaining commits...${NC}"
            # Continue with the rest of the script to process remaining commits
        else
            echo -e "${RED}Failed to continue cherry-pick. Please resolve manually.${NC}"
            exit 1
        fi
    fi
fi

# Check if we're on the correct branch (unless we're on new-base from a previous run)
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT_BRANCH" != "$LOCAL_BRANCH" ] && [ "$CURRENT_BRANCH" != "new-base" ]; then
    echo -e "${YELLOW}Currently on branch $CURRENT_BRANCH, switching to $LOCAL_BRANCH${NC}"
    git checkout $LOCAL_BRANCH
elif [ "$CURRENT_BRANCH" == "new-base" ]; then
    echo -e "${YELLOW}Resuming on existing new-base branch...${NC}"
    # Skip the branch creation steps and go directly to cherry-picking remaining commits
    RESUMING_CHERRYPICK=true
fi

# Fetch the latest changes from upstream (unless resuming)
if [ "$RESUMING_CHERRYPICK" != "true" ]; then
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
else
    # We're resuming, so we need to get the temp branch reference
    if git show-ref --verify --quiet refs/heads/$TEMP_BRANCH; then
        echo -e "${YELLOW}Using existing temporary branch $TEMP_BRANCH...${NC}"
    else
        echo -e "${RED}Error: Cannot find temporary branch $TEMP_BRANCH for resuming.${NC}"
        exit 1
    fi
fi

# Cherry-pick our custom commits
echo -e "${YELLOW}Identifying and applying custom commits on top of new base...${NC}"

# Get a list of commits that are in our branch but not in the old upstream branch
# This ensures we only get our custom commits, not upstream commits that may differ between branches
echo -e "${YELLOW}Identifying custom commits to apply...${NC}"

# Get all custom commits by comparing against the old upstream branch
CUSTOM_COMMITS=$(git log --format="%H" $TEMP_BRANCH --not $UPSTREAM_REMOTE/$OLD_UPSTREAM_BRANCH)

# Convert to array (oldest first)
COMMIT_ARRAY=($(echo "$CUSTOM_COMMITS" | tac))
COMMITS_TO_APPLY=("${COMMIT_ARRAY[@]}")  # Apply all commits

# Filter out commits that have already been applied (by checking commit messages)
REMAINING_COMMITS=()
for COMMIT in "${COMMITS_TO_APPLY[@]}"; do
    COMMIT_MSG=$(git log -1 --pretty=format:"%s" $COMMIT)
    if ! git log --oneline --grep="$COMMIT_MSG" HEAD | grep -q .; then
        REMAINING_COMMITS+=("$COMMIT")
    else
        echo -e "${GREEN}Commit already applied: $COMMIT_MSG${NC}"
    fi
done

COMMITS_TO_APPLY=("${REMAINING_COMMITS[@]}")

# Check if we have any commits to apply
if [ ${#COMMITS_TO_APPLY[@]} -eq 0 ]; then
    echo -e "${YELLOW}No custom commits found to apply.${NC}"
    echo -e "${YELLOW}Using the upstream branch directly.${NC}"
    
    # Replace the old branch with our new one
    git checkout new-base
    git branch -D $LOCAL_BRANCH
    git branch -m new-base $LOCAL_BRANCH
else
    # Apply our custom commits in order (oldest first)
    for COMMIT in "${COMMITS_TO_APPLY[@]}"; do
        echo -e "${YELLOW}Applying commit: $(git log -1 --pretty=format:"%s" $COMMIT)${NC}"
        if ! git cherry-pick $COMMIT; then
            echo -e "${RED}Cherry-pick failed for commit $COMMIT${NC}"
            echo -e "${YELLOW}Resolve conflicts manually, then run this script again to continue.${NC}"
            echo -e "${YELLOW}The script will detect the resolved state and continue with remaining commits.${NC}"
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

# Provide instructions for pushing to origin
echo -e "${YELLOW}To push these changes to your fork, run:${NC}"
echo -e "  git push $ORIGIN_REMOTE $LOCAL_BRANCH --force"
echo -e "${YELLOW}Note: --force is required because the branch history has been rewritten.${NC}"
