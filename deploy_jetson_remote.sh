#!/usr/bin/env bash
set -euo pipefail

# =========================================================================== #
# Jetson Nano remote deploy — archive → SCP → SSH → Docker build & start
# =========================================================================== #
# Run from your dev machine. Requires:
#   - sshpass (optional, only if JETSON_PASSWORD is set)
#   - tar + pigz (optional, fallback gzip)
#
# Usage:
#   ./deploy_jetson_remote.sh [--yes] [--no-build] [--env .env.jetson]
#
# Env (set in your env file or export):
#   JETSON_HOST        Jetson IP or hostname
#   JETSON_USER        SSH user (default: root)
#   JETSON_PORT        SSH port (default: 22)
#   JETSON_PASSWORD    SSH password (blank = use key auth)
#   JETSON_DEST_DIR    Target dir on Jetson (default: /opt/car-calib-jetson)
# =========================================================================== #

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROJECT_NAME="${PROJECT_NAME:-car-calib-jetson}"
VERSION="$(date -u +%Y%m%d-%H%M%S)-$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"

AUTO_CONFIRM=false
SKIP_BUILD=false

usage() {
    cat <<'EOF'
Usage:
  ./deploy_jetson_remote.sh [--yes] [--no-build] [--env <path>]

Options:
  --yes         Skip confirmation prompt
  --no-build    Skip docker build on remote (just upload + restart)
  --env <path>  Custom env file (default: .env.jetson)

Setup:
  1. Create .env.jetson.remote with lines:
       JETSON_HOST=192.168.x.x
       JETSON_USER=root
       JETSON_PASSWORD=yourpass   (or leave blank for key auth)
  2. Run: ./deploy_jetson_remote.sh
EOF
}

# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
log_step()  { echo -e "${BLUE}$1${NC}"; }
log_ok()    { echo -e "${GREEN}$1${NC}"; }
log_warn()  { echo -e "${YELLOW}$1${NC}"; }
log_err()   { echo -e "${RED}$1${NC}"; }

trim() {
    local v="$1"
    v="${v#"${v%%[![:space:]]*}"}"
    v="${v%"${v##*[![:space:]]}"}"
    printf '%s' "$v"
}

shell_quote() {
    printf "'%s'" "${1//\'/\'\\\'\'}"
}

# --------------------------------------------------------------------------- #
# Parse args
# --------------------------------------------------------------------------- #
ENV_FILE="${ENV_FILE:-${SCRIPT_DIR}/.env.jetson}"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --yes)       AUTO_CONFIRM=true; shift ;;
        --no-build)  SKIP_BUILD=true; shift ;;
        --env)       ENV_FILE="$2"; shift 2 ;;
        -h|--help)   usage; exit 0 ;;
        *)           log_err "Unknown: $1"; usage; exit 1 ;;
    esac
done

if [[ ! -f "$ENV_FILE" ]]; then
    log_err "Env file not found: $ENV_FILE"
    log_err "Copy .env.jetson to $ENV_FILE and add JETSON_HOST, JETSON_USER, JETSON_PASSWORD"
    exit 1
fi

# Source env (these override anything in .env.jetson)
export JETSON_HOST JETSON_USER JETSON_PORT JETSON_PASSWORD JETSON_DEST_DIR
export JETSON_HOST JETSON_USER JETSON_PORT JETSON_PASSWORD JETSON_DEST_DIR

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

# Defaults
JETSON_HOST="${JETSON_HOST:-}"
JETSON_USER="${JETSON_USER:-root}"
JETSON_PORT="${JETSON_PORT:-22}"
JETSON_PASSWORD="$(trim "${JETSON_PASSWORD:-}")"
JETSON_DEST_DIR="$(trim "${JETSON_DEST_DIR:-/opt/car-calib-jetson}")"
COMPOSE_FILE="docker-compose.jetson.yml"

if [[ -z "$JETSON_HOST" ]]; then
    log_err "JETSON_HOST not set. Add it to $ENV_FILE"
    exit 1
fi

# --------------------------------------------------------------------------- #
# SSH helper
# --------------------------------------------------------------------------- #
ssh_cmd() {
    local password="$1"
    shift
    if [[ -n "$password" ]]; then
        local sp
        sp="$(command -v sshpass 2>/dev/null || true)"
        if [[ -n "$sp" ]]; then
            "$sp" -p "$password" ssh -p "$JETSON_PORT" \
                -o StrictHostKeyChecking=accept-new \
                -o ConnectTimeout=10 \
                "$@" 2>/dev/null
            return
        fi
        log_err "JETSON_PASSWORD is set but sshpass is not installed."
        log_err "Install: sudo apt-get install -y sshpass"
        exit 1
    fi
    ssh -p "$JETSON_PORT" \
        -o BatchMode=yes \
        -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout=10 \
        "$@"
}

scp_cmd() {
    local password="$1"
    shift
    if [[ -n "$password" ]]; then
        local sp
        sp="$(command -v sshpass 2>/dev/null || true)"
        if [[ -n "$sp" ]]; then
            "$sp" -p "$password" scp -P "$JETSON_PORT" \
                -o StrictHostKeyChecking=accept-new \
                -o ConnectTimeout=10 \
                "$@"
            return
        fi
        log_err "sshpass required for password-based SCP."
        exit 1
    fi
    scp -P "$JETSON_PORT" \
        -o BatchMode=yes \
        -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout=10 \
        "$@"
}

# --------------------------------------------------------------------------- #
# Confirm
# --------------------------------------------------------------------------- #
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   Jetson Nano Remote Deploy${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "  Target:    ${BLUE}${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT}${NC}"
echo -e "  Dest:      ${BLUE}${JETSON_DEST_DIR}${NC}"
echo -e "  Version:   ${BLUE}${VERSION}${NC}"
echo ""

if [[ "$AUTO_CONFIRM" != "true" ]]; then
    read -r -p "Deploy to Jetson Nano? [y/N]: " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        log_warn "Cancelled"
        exit 0
    fi
fi

# --------------------------------------------------------------------------- #
# Step 1: Check SSH
# --------------------------------------------------------------------------- #
log_step "[1/4] Checking SSH connectivity..."
if ! ssh_cmd "$JETSON_PASSWORD" "${JETSON_USER}@${JETSON_HOST}" exit 2>/dev/null; then
    log_err "Cannot SSH to ${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT}"
    log_err "Check JETSON_HOST, JETSON_PASSWORD, or SSH key access."
    exit 1
fi
log_ok "SSH OK"

# --------------------------------------------------------------------------- #
# Step 2: Build archive
# --------------------------------------------------------------------------- #
log_step "[2/4] Building source archive..."

ARCHIVE_PATH="/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"
remote_env="/tmp/${PROJECT_NAME}-${VERSION}.env"

# Create archive (exclude bulky files)
if command -v pigz &>/dev/null; then
    tar --exclude='.git' --exclude='.pytest_cache' --exclude='__pycache__' \
        --exclude='.venv' --exclude='venv' --exclude='*.pyc' \
        --exclude='*.log' --exclude='logs' --exclude='.env' \
        -I pigz -cf "$ARCHIVE_PATH" -C "$SCRIPT_DIR" .
else
    tar --exclude='.git' --exclude='.pytest_cache' --exclude='__pycache__' \
        --exclude='.venv' --exclude='venv' --exclude='*.pyc' \
        --exclude='*.log' --exclude='logs' --exclude='.env' \
        -czf "$ARCHIVE_PATH" -C "$SCRIPT_DIR" .
fi

log_ok "Archive: $ARCHIVE_PATH ($(du -h "$ARCHIVE_PATH" | cut -f1))"

# --------------------------------------------------------------------------- #
# Step 3: Upload
# --------------------------------------------------------------------------- #
log_step "[3/4] Uploading to Jetson..."

ssh_cmd "$JETSON_PASSWORD" "${JETSON_USER}@${JETSON_HOST}" "mkdir -p ${JETSON_DEST_DIR}/releases" 2>/dev/null || true

scp_cmd "$JETSON_PASSWORD" "$ARCHIVE_PATH" "${JETSON_USER}@${JETSON_HOST}:/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"
scp_cmd "$JETSON_PASSWORD" "$ENV_FILE" "${JETSON_USER}@${JETSON_HOST}:${remote_env}"

log_ok "Upload complete"

# --------------------------------------------------------------------------- #
# Step 4: Deploy remote
# --------------------------------------------------------------------------- #
log_step "[4/4] Deploying on Jetson..."

ssh_cmd "$JETSON_PASSWORD" "${JETSON_USER}@${JETSON_HOST}" \
    DEST_DIR="$(shell_quote "$JETSON_DEST_DIR")" \
    VERSION="$(shell_quote "$VERSION")" \
    PROJECT_NAME="$(shell_quote "$PROJECT_NAME")" \
    COMPOSE_FILE="$(shell_quote "$COMPOSE_FILE")" \
    SKIP_BUILD="$(shell_quote "$SKIP_BUILD")" \
    REMOTE_ENV="$(shell_quote "$remote_env")" \
    bash -s <<'REMOTE_EOF'
set -euo pipefail

root_dir="${DEST_DIR%/}"
release_dir="${root_dir}/releases/${VERSION}"
current_dir="${root_dir}/current"
remote_archive="/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"

echo "[remote] Extracting release..."
mkdir -p "$release_dir"
tar -xzf "$remote_archive" -C "$release_dir"
cp "$REMOTE_ENV" "$release_dir/.env"
ln -sfn "$release_dir" "$current_dir"
rm -f "$remote_archive" "$REMOTE_ENV"

echo "[remote] Docker compose..."
cd "$current_dir"

if ! command -v docker &>/dev/null; then
    echo "[remote] Docker not installed. Install and re-run."
    exit 1
fi

COMPOSE_CMD=""
if docker compose version &>/dev/null 2>&1; then
    COMPOSE_CMD="docker compose"
elif command -v docker-compose &>/dev/null; then
    COMPOSE_CMD="docker-compose"
else
    echo "[remote] docker compose not found"
    exit 1
fi

# Stop old
$COMPOSE_CMD -f "$COMPOSE_FILE" down 2>/dev/null || true

# Build & start
if [[ "$SKIP_BUILD" == "true" ]]; then
    $COMPOSE_CMD -f "$COMPOSE_FILE" up -d --remove-orphans
else
    $COMPOSE_CMD -f "$COMPOSE_FILE" up --build -d --remove-orphans
fi

# Status
sleep 2
$COMPOSE_CMD -f "$COMPOSE_FILE" ps

# Clean old releases (keep 3)
cd "${root_dir}/releases"
ls -1dt */ 2>/dev/null | tail -n +4 | xargs -r rm -rf -- || true

echo "[remote] Deploy complete"
REMOTE_EOF

# --------------------------------------------------------------------------- #
# Done
# --------------------------------------------------------------------------- #
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Remote Deploy Complete${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
echo -e "  Dashboard:  ${BLUE}http://${JETSON_HOST}:${DASHBOARD_PORT}${NC}"
echo -e "  Logs:       ${BLUE}ssh ${JETSON_USER}@${JETSON_HOST} 'docker logs -f car-calib-jetson'${NC}"
echo ""

rm -f "$ARCHIVE_PATH"
