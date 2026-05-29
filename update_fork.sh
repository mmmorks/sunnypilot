#!/usr/bin/env bash
set -e

# Configuration
LOCAL_BRANCH="master"
UPSTREAM_REMOTE="upstream"
UPSTREAM_BRANCH="master"
ORIGIN_REMOTE="origin"

# Print status messages in color
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${YELLOW}Updating $LOCAL_BRANCH from $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"

# If a rebase is already in progress, tell the user to finish it
if [ -d ".git/rebase-merge" ] || [ -d ".git/rebase-apply" ]; then
    echo -e "${RED}A rebase is already in progress.${NC}"
    echo -e "${YELLOW}Resolve conflicts, then: git add <files> && git rebase --continue${NC}"
    echo -e "${YELLOW}Or abort with: git rebase --abort${NC}"
    exit 1
fi

# Stash uncommitted changes
if ! git diff --quiet || ! git diff --cached --quiet; then
    echo -e "${YELLOW}Stashing uncommitted changes...${NC}"
    git stash push -m "update_fork: auto-stash"
    STASHED=true
fi

# Ensure we're on the right branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [ "$CURRENT_BRANCH" != "$LOCAL_BRANCH" ]; then
    echo -e "${YELLOW}Switching to $LOCAL_BRANCH...${NC}"
    git checkout "$LOCAL_BRANCH"
fi

# Fetch latest upstream
echo -e "${YELLOW}Fetching $UPSTREAM_REMOTE...${NC}"
git fetch "$UPSTREAM_REMOTE"

# Find where custom commits begin
FORK_POINT=$(git merge-base "$LOCAL_BRANCH" "$UPSTREAM_REMOTE/$UPSTREAM_BRANCH")
UPSTREAM_HEAD=$(git rev-parse "$UPSTREAM_REMOTE/$UPSTREAM_BRANCH")

if [ "$FORK_POINT" == "$UPSTREAM_HEAD" ]; then
    echo -e "${GREEN}Already up to date with $UPSTREAM_REMOTE/$UPSTREAM_BRANCH.${NC}"
    if [ "$STASHED" == "true" ]; then
        git stash pop
    fi
    exit 0
fi

CUSTOM_COUNT=$(git rev-list --count "$FORK_POINT"..HEAD)
echo -e "${YELLOW}Rebasing $CUSTOM_COUNT custom commit(s) onto $UPSTREAM_REMOTE/$UPSTREAM_BRANCH...${NC}"
echo -e "${YELLOW}Fork point: $(git log -1 --oneline "$FORK_POINT")${NC}"

if git rebase --onto "$UPSTREAM_REMOTE/$UPSTREAM_BRANCH" "$FORK_POINT" "$LOCAL_BRANCH"; then
    echo -e "${GREEN}Rebase completed successfully!${NC}"
else
    echo -e "${RED}Rebase encountered conflicts.${NC}"
    echo -e "${YELLOW}Resolve, then: git add <files> && git rebase --continue${NC}"
    echo -e "${YELLOW}Or abort with: git rebase --abort${NC}"
    exit 1
fi

# Restore stash
if [ "$STASHED" == "true" ]; then
    echo -e "${YELLOW}Restoring stashed changes...${NC}"
    git stash pop || echo -e "${YELLOW}Warning: stash pop conflict, check 'git stash list'${NC}"
fi

echo -e "${GREEN}Done! $CUSTOM_COUNT custom commit(s) on top of $UPSTREAM_REMOTE/$UPSTREAM_BRANCH.${NC}"
echo -e "${YELLOW}To push: git push $ORIGIN_REMOTE $LOCAL_BRANCH --force${NC}"
