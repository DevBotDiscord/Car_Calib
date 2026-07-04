#!/usr/bin/env bash
set -uo pipefail

# Remote deploy for Raspberry Pi control-direct runtime.
# Archives local source, uploads through SSH/SCP, builds + starts Docker on Pi.

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PROJECT_NAME="${PROJECT_NAME:-car-calib-control-direct}"
VERSION="$(date -u +%Y%m%d-%H%M%S)-$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
ENV_FILE="${ENV_FILE:-${SCRIPT_DIR}/.env.control-direct}"
COMPOSE_FILE="docker-compose.control-direct.yml"
CONTAINER_NAME="${CONTROL_DIRECT_CONTAINER_NAME:-car-calib-control-direct}"
AUTO_CONFIRM=false
SKIP_BUILD=false

usage() {
    cat <<'EOF'
Usage:
  ./deploy_control_direct_remote.sh [--yes] [--no-build] [--env <path>]

Env in --env file:
  RPI_HOST          Raspberry Pi IP/hostname
  RPI_USER          SSH user (default: root)
  RPI_PORT          SSH port (default: 22)
  RPI_PASSWORD      SSH password (blank = SSH key auth)
  RPI_DEST_DIR      Target dir (default: /opt/car-calib-control-direct)

Runtime env in same file:
  SERVO_PIN, BASE_OUT1/2/3, RELAY_PIN, POWER_RELAY_PIN, DASHBOARD_PORT, ...
EOF
}

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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --yes) AUTO_CONFIRM=true; shift ;;
        --no-build) SKIP_BUILD=true; shift ;;
        --env) ENV_FILE="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) log_err "Unknown: $1"; usage; exit 1 ;;
    esac
done

if [[ ! -f "$ENV_FILE" ]]; then
    log_err "Env file not found: $ENV_FILE"
    exit 1
fi

export RPI_HOST RPI_USER RPI_PORT RPI_PASSWORD RPI_DEST_DIR DASHBOARD_PORT
set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

RPI_HOST="${RPI_HOST:-}"
RPI_USER="${RPI_USER:-root}"
RPI_PORT="${RPI_PORT:-22}"
RPI_PASSWORD="$(trim "${RPI_PASSWORD:-}")"
RPI_DEST_DIR="$(trim "${RPI_DEST_DIR:-/opt/car-calib-control-direct}")"

if [[ -z "$RPI_HOST" ]]; then
    log_err "RPI_HOST not set. Add it to $ENV_FILE"
    exit 1
fi

ssh_cmd() {
    local password="$1"
    shift
    if [[ -n "$password" ]]; then
        local sp
        sp="$(command -v sshpass 2>/dev/null || true)"
        if [[ -z "$sp" ]]; then
            log_err "RPI_PASSWORD set but sshpass missing. Install: sudo apt-get install -y sshpass"
            exit 1
        fi
        "$sp" -p "$password" ssh -p "$RPI_PORT" \
            -o StrictHostKeyChecking=accept-new \
            -o ConnectTimeout=10 \
            "$@" 2>/dev/null
        return
    fi
    ssh -p "$RPI_PORT" \
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
        if [[ -z "$sp" ]]; then
            log_err "sshpass required for password-based SCP."
            exit 1
        fi
        "$sp" -p "$password" scp -P "$RPI_PORT" \
            -o StrictHostKeyChecking=accept-new \
            -o ConnectTimeout=10 \
            "$@"
        return
    fi
    scp -P "$RPI_PORT" \
        -o BatchMode=yes \
        -o StrictHostKeyChecking=accept-new \
        -o ConnectTimeout=10 \
        "$@"
}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   Control Direct Remote Deploy${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "  Target:  ${BLUE}${RPI_USER}@${RPI_HOST}:${RPI_PORT}${NC}"
echo -e "  Dest:    ${BLUE}${RPI_DEST_DIR}${NC}"
echo -e "  Version: ${BLUE}${VERSION}${NC}"
echo ""

if [[ "$AUTO_CONFIRM" != "true" ]]; then
    read -r -p "Deploy control-direct to Raspberry Pi? [y/N]: " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        log_warn "Cancelled"
        exit 0
    fi
fi

log_step "[1/4] Checking SSH connectivity..."
if ! ssh_cmd "$RPI_PASSWORD" "${RPI_USER}@${RPI_HOST}" exit 2>/dev/null; then
    log_err "Cannot SSH to ${RPI_USER}@${RPI_HOST}:${RPI_PORT}"
    exit 1
fi
log_ok "SSH OK"

log_step "[2/4] Building source archive..."
ARCHIVE_PATH="/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"
remote_env="/tmp/${PROJECT_NAME}-${VERSION}.env"

tar_args=(
    --exclude='.git'
    --exclude='.pytest_cache'
    --exclude='__pycache__'
    --exclude='.venv'
    --exclude='venv'
    --exclude='*.pyc'
    --exclude='*.log'
    --exclude='logs'
    --exclude='routes'
    --exclude='.env'
)
if command -v pigz &>/dev/null; then
    tar "${tar_args[@]}" -I pigz -cf "$ARCHIVE_PATH" -C "$SCRIPT_DIR" .
else
    tar "${tar_args[@]}" -czf "$ARCHIVE_PATH" -C "$SCRIPT_DIR" .
fi
log_ok "Archive: $ARCHIVE_PATH ($(du -h "$ARCHIVE_PATH" | cut -f1))"

log_step "[3/4] Uploading to Raspberry Pi..."
ssh_cmd "$RPI_PASSWORD" "${RPI_USER}@${RPI_HOST}" "mkdir -p ${RPI_DEST_DIR}/releases" 2>/dev/null || true
scp_cmd "$RPI_PASSWORD" "$ARCHIVE_PATH" "${RPI_USER}@${RPI_HOST}:/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"
scp_cmd "$RPI_PASSWORD" "$ENV_FILE" "${RPI_USER}@${RPI_HOST}:${remote_env}"
log_ok "Upload complete"

log_step "[4/4] Deploying on Raspberry Pi..."
ssh_cmd "$RPI_PASSWORD" "${RPI_USER}@${RPI_HOST}" \
    DEST_DIR="$(shell_quote "$RPI_DEST_DIR")" \
    VERSION="$(shell_quote "$VERSION")" \
    PROJECT_NAME="$(shell_quote "$PROJECT_NAME")" \
    COMPOSE_FILE="$(shell_quote "$COMPOSE_FILE")" \
    CONTAINER_NAME="$(shell_quote "$CONTAINER_NAME")" \
    SKIP_BUILD="$(shell_quote "$SKIP_BUILD")" \
    REMOTE_ENV="$(shell_quote "$remote_env")" \
    bash -s <<'REMOTE_EOF'
echo "[remote] START deploy version=$VERSION"
set -uo pipefail

root_dir="${DEST_DIR%/}"
release_dir="${root_dir}/releases/${VERSION}"
current_dir="${root_dir}/current"
remote_archive="/tmp/${PROJECT_NAME}-${VERSION}.tar.gz"

echo "[remote] Extracting release..."
mkdir -p "$release_dir"
tar -xzf "$remote_archive" -C "$release_dir"
cp "$REMOTE_ENV" "$release_dir/.env.control-direct"
ln -sfn "$release_dir" "$current_dir"
rm -f "$remote_archive" "$REMOTE_ENV"

cd "$current_dir"
mkdir -p "$root_dir/data/logs" "$root_dir/data/routes"
export RPI_DEST_DIR="$root_dir"

if ! command -v docker &>/dev/null; then
    echo "[remote] ERROR: Docker not installed."
    exit 1
fi

DOCKER_BIN="docker"
if ! docker ps &>/dev/null 2>&1; then
    if sudo -n docker ps &>/dev/null 2>&1; then
        DOCKER_BIN="sudo docker"
        echo "[remote] Using sudo docker"
    else
        echo "[remote] ERROR: docker not accessible"
        exit 1
    fi
fi

if $DOCKER_BIN compose version &>/dev/null 2>&1; then
    COMPOSE_CMD="$DOCKER_BIN compose"
elif command -v docker-compose &>/dev/null; then
    COMPOSE_CMD="docker-compose"
else
    echo "[remote] ERROR: docker compose not found"
    exit 1
fi

echo "[remote] Compose: $COMPOSE_CMD -f $COMPOSE_FILE"
$COMPOSE_CMD -f "$COMPOSE_FILE" down 2>/dev/null || true

BUILD_LOG="/tmp/car-calib-control-direct-build-${VERSION}.log"
echo "[remote] Build/start log: $BUILD_LOG"
if [[ "$SKIP_BUILD" == "true" ]]; then
    $COMPOSE_CMD -f "$COMPOSE_FILE" up -d --remove-orphans > "$BUILD_LOG" 2>&1 &
else
    $COMPOSE_CMD -f "$COMPOSE_FILE" up --build -d --remove-orphans > "$BUILD_LOG" 2>&1 &
fi
BUILD_PID=$!

DEADLINE=$((SECONDS + 180))
while kill -0 $BUILD_PID 2>/dev/null && (( SECONDS < DEADLINE )); do
    if [[ -s "$BUILD_LOG" ]]; then
        tail -5 "$BUILD_LOG" 2>/dev/null
    fi
    sleep 3
done

if kill -0 $BUILD_PID 2>/dev/null; then
    echo "[remote] Build still running PID=$BUILD_PID"
    echo "[remote] Check: tail -f $BUILD_LOG"
else
    wait $BUILD_PID || { tail -80 "$BUILD_LOG" 2>/dev/null || true; exit 1; }
    echo "[remote] Build/start done"
fi

$COMPOSE_CMD -f "$COMPOSE_FILE" ps 2>/dev/null || true
docker logs --tail 30 "$CONTAINER_NAME" 2>/dev/null || true

cd "${root_dir}/releases"
ls -1dt */ 2>/dev/null | tail -n +4 | xargs -r rm -rf -- || true
echo "[remote] Deploy complete"
REMOTE_EOF

rm -f "$ARCHIVE_PATH"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   Control Direct Deploy Complete${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  Dashboard: ${BLUE}http://${RPI_HOST}:${DASHBOARD_PORT:-8080}${NC}"
echo -e "  Logs:      ${BLUE}ssh ${RPI_USER}@${RPI_HOST} 'docker logs -f ${CONTAINER_NAME}'${NC}"
