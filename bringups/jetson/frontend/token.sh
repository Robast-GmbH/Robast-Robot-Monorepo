#!/bin/bash

# Check if GITHUB_TOKEN is set
if [ -z "$GITHUB_TOKEN" ]; then
    echo "Error: GITHUB_TOKEN is not set."
    exit 1
fi

# API URL to get user info
API_URL="https://api.github.com/user"

# Make a request to the GitHub API
RESPONSE=$(curl -s -H "Authorization: token $GITHUB_TOKEN" $API_URL)

# Check if the response contains user information
if echo "$RESPONSE" | grep -q '"login":'; then
    echo "The GitHub token is valid."
    exit 0
else
    # If the token is invalid, check for error messages
    echo "$RESPONSE" | grep -q 'Bad credentials' && echo "The GitHub token is invalid."
    echo "$RESPONSE" | grep -q 'authentication failed' && echo "The GitHub token is invalid or expired."
    exit 1
fi
