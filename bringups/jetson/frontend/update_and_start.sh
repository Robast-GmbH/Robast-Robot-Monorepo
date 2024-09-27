#!/bin/bash

./token.sh
IS_TOKEN_VALID=$?
if [ $IS_TOKEN_VALID -ne 0 ]; then
    exit $IS_TOKEN_VALID
fi

REPO_OWNER="Robast-GmbH"
REPO_NAME="Monorepo"
WORKFLOW_NAME="flutter_build_linux.yml"
OUTPUT_DIR="robot_frontend"
VERSION_FILE="version.txt"

function start_frontend(){
    echo "Starting frontend..."
    if [ -f "$OUTPUT_DIR/robot_frontend" ]; then
        $OUTPUT_DIR/robot_frontend &
        sleep 1
        wmctrl -r robot_frontend -b add,fullscreen
    else
        echo "No frontend executable found in $OUTPUT_DIR"
    fi
}

if [ -f "$VERSION_FILE" ] && [ -d "$OUTPUT_DIR" ]; then
  LAST_RUN_TIME=$(cat "$VERSION_FILE")
else
  echo "No last run time found. Assuming fresh install."
  LAST_RUN_TIME="1970-01-01T00:00:00Z"  # Epoch time for initial comparison
fi

LATEST_RUN_INFO=$(curl -s -H "Authorization: token $GITHUB_TOKEN" \
  "https://api.github.com/repos/$REPO_OWNER/$REPO_NAME/actions/workflows/$WORKFLOW_NAME/runs?status=success&per_page=1")

LATEST_RUN_TIMESTAMP=$(echo "$LATEST_RUN_INFO" | grep '"updated_at":' | head -n 1 | sed 's/.*"\(.*\)".*/\1/')
LATEST_RUN_ID=$(echo "$LATEST_RUN_INFO" | grep '"id":' | head -n 1 | sed 's/[^0-9]*//g')

# Check if a new successful run was found
if [[ -z "$LATEST_RUN_TIMESTAMP" ]]; then
  echo "No successful workflow run found."
  exit 1
fi

# Compare the timestamps
if [[ "$LATEST_RUN_TIMESTAMP" == "$LAST_RUN_TIME" ]]; then
  echo "You already have the latest artifact from the last run at $LAST_RUN_TIME."
  start_frontend
  exit 0
fi

# Fetch the list of artifacts for the latest workflow run
ARTIFACT_URL=$(curl -s -H "Authorization: token $GITHUB_TOKEN" \
    "https://api.github.com/repos/$REPO_OWNER/$REPO_NAME/actions/runs/$LATEST_RUN_ID/artifacts" \
    | grep '"archive_download_url":' | head -n 1 | cut -d '"' -f 4)

if [ -z "$ARTIFACT_URL" ]; then
  echo "No artifacts found for run ID $LATEST_RUN_ID"
  exit 1
fi

echo "A new artifact is available from the latest run at $LATEST_RUN_TIMESTAMP (last run time: $LAST_RUN_TIME)."

rm -r $OUTPUT_DIR

mkdir -p $OUTPUT_DIR

# Download the artifact zip file
curl -L -H "Authorization: token $GITHUB_TOKEN" \
    -o "$OUTPUT_DIR/artifact.zip" "$ARTIFACT_URL"

unzip "$OUTPUT_DIR/artifact.zip" -d $OUTPUT_DIR

# Cleanup
rm "$OUTPUT_DIR/artifact.zip"

chmod +x "$OUTPUT_DIR/robot_frontend"

echo "$LATEST_RUN_TIMESTAMP" > "$VERSION_FILE"
echo "Last run time updated to $LATEST_RUN_TIMESTAMP."

echo "Downloaded and extracted artifact to $OUTPUT_DIR"

start_frontend
