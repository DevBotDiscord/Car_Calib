#!/usr/bin/env bash

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-${SCRIPT_DIR}/.env.production}"
TARGET="${1:-all}"

usage() {
    cat <<'EOF'
Usage:
  ./deploy_production.sh [all|minipc|ras]

Targets:
  all     Deploy both vision (MiniPC) and Raspberry Pi bridge
  minipc  Deploy only the vision stack
  ras     Deploy only the Raspberry Pi MQTT bridge
EOF
}

print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}   Car_Calib Production Deployment${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

log_step() {
    echo -e "${BLUE}$1${NC}"
}

log_ok() {
    echo -e "${GREEN}$1${NC}"
}

log_warn() {
    echo -e "${YELLOW}$1${NC}"
}

log_err() {
    echo -e "${RED}$1${NC}"
}

trim_whitespace() {
    local value="$1"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s' "$value"
}

find_command() {
    local candidate

    for candidate in "$@"; do
        if command -v "$candidate" >/dev/null 2>&1; then
            command -v "$candidate"
            return 0
        fi
    done

    return 1
}

require_file() {
    local file_path="$1"
    if [[ ! -f "$file_path" ]]; then
        log_err "Missing required file: $file_path"
        exit 1
    fi
}

require_vars() {
    local missing=()
    local name

    for name in "$@"; do
        if [[ -z "${!name:-}" ]]; then
            missing+=("$name")
        fi
    done

    if (( ${#missing[@]} > 0 )); then
        log_err "Missing required environment variables in ${ENV_FILE}: ${missing[*]}"
        exit 1
    fi
}

ssh_base_cmd() {
    local password="$1"
    local port="$2"
    local sshpass_bin=""
    local plink_bin=""
    shift 2

    password="$(trim_whitespace "$password")"

    if [[ -n "$password" ]]; then
        if sshpass_bin="$(find_command sshpass)"; then
            "$sshpass_bin" -p "$password" ssh -p "$port" -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
                -o StrictHostKeyChecking=accept-new "$@"
            return
        fi

        if plink_bin="$(find_command plink.exe plink)"; then
            "$plink_bin" -P "$port" -pw "$password" -batch "$@"
            return
        fi

        log_err "Password-based SSH requested, but neither sshpass nor plink.exe is available."
        log_err "Install sshpass or PuTTY (plink/pscp), or leave *_PASSWORD empty and use SSH keys."
        exit 1
    fi

    ssh -p "$port" \
        -o BatchMode=yes \
        -o NumberOfPasswordPrompts=0 \
        -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
        -o StrictHostKeyChecking=accept-new \
        "$@"
}

scp_base_cmd() {
    local password="$1"
    local port="$2"
    local sshpass_bin=""
    local pscp_bin=""
    shift 2

    password="$(trim_whitespace "$password")"

    if [[ -n "$password" ]]; then
        if sshpass_bin="$(find_command sshpass)"; then
            "$sshpass_bin" -p "$password" scp -P "$port" -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
                -o StrictHostKeyChecking=accept-new "$@"
            return
        fi

        if pscp_bin="$(find_command pscp.exe pscp)"; then
            "$pscp_bin" -P "$port" -pw "$password" -batch "$@"
            return
        fi

        log_err "Password-based SCP requested, but neither sshpass nor pscp.exe is available."
        log_err "Install sshpass or PuTTY (plink/pscp), or leave *_PASSWORD empty and use SSH keys."
        exit 1
    fi

    scp -P "$port" \
        -o BatchMode=yes \
        -o NumberOfPasswordPrompts=0 \
        -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
        -o StrictHostKeyChecking=accept-new \
        "$@"
}

create_archive() {
    local archive_path="$1"

    tar \
        --exclude='.git' \
        --exclude='.pytest_cache' \
        --exclude='__pycache__' \
        --exclude='.venv' \
        --exclude='venv' \
        --exclude='.env' \
        --exclude='.env.production' \
        --exclude='.env.backup' \
        --exclude='*.pyc' \
        --exclude='*.pyo' \
        --exclude='*.log' \
        --exclude='.last_production_version' \
        --exclude='.last_production_deploy' \
        -czf "$archive_path" \
        -C "$SCRIPT_DIR" .
}

deploy_target() {
    local label="$1"
    local host="$2"
    local user="$3"
    local port="$4"
    local password="$5"
    local dest_dir="$6"
    local compose_file="$7"
    local use_sudo_docker="$8"

    local remote_archive="/tmp/${PROJECT_NAME}-${VERSION}-${label}.tar.gz"
    local remote_env="/tmp/${PROJECT_NAME}-${VERSION}-${label}.env"

    log_step "[${label}] Checking SSH connectivity..."
    if ! ssh_base_cmd "$password" "$port" "${user}@${host}" exit >/dev/null 2>&1; then
        log_err "[${label}] Cannot connect to ${user}@${host}:${port}"
        log_err "[${label}] Check SSH key access or verify ${label^^}_PASSWORD and sshpass/plink availability."
        exit 1
    fi
    log_ok "[${label}] SSH connection OK"

    log_step "[${label}] Uploading release bundle..."
    ssh_base_cmd "$password" "$port" "${user}@${host}" "mkdir -p '${dest_dir}/releases'"
    scp_base_cmd "$password" "$port" "$ARCHIVE_PATH" "${user}@${host}:${remote_archive}"
    scp_base_cmd "$password" "$port" "$ENV_FILE" "${user}@${host}:${remote_env}"
    log_ok "[${label}] Upload complete"

    log_step "[${label}] Deploying release ${VERSION}..."
    ssh_base_cmd "$password" "$port" "${user}@${host}" bash -s -- \
        "$dest_dir" \
        "$compose_file" \
        "$VERSION" \
        "$use_sudo_docker" \
        "$remote_archive" \
        "$remote_env" \
        "$DEPLOY_KEEP_RELEASES" <<'EOF'
set -euo pipefail

dest_dir="$1"
compose_file="$2"
version="$3"
use_sudo_docker="$4"
remote_archive="$5"
remote_env="$6"
keep_releases="$7"

root_dir="${dest_dir%/}"
release_dir="${root_dir}/releases/${version}"
current_dir="${root_dir}/current"

mkdir -p "${root_dir}/releases"
rm -rf "${release_dir}"
mkdir -p "${release_dir}"
tar -xzf "${remote_archive}" -C "${release_dir}"
cp "${remote_env}" "${release_dir}/.env"
cp "${remote_env}" "${release_dir}/.env.production"
ln -sfn "${release_dir}" "${current_dir}"
rm -f "${remote_archive}" "${remote_env}"

if [[ "${use_sudo_docker}" == "true" ]]; then
    docker_cmd=(sudo docker)
else
    docker_cmd=(docker)
fi

cd "${current_dir}"
"${docker_cmd[@]}" compose -f "${compose_file}" up --build -d
"${docker_cmd[@]}" compose -f "${compose_file}" ps

if [[ "${keep_releases}" =~ ^[0-9]+$ ]] && (( keep_releases > 0 )); then
    cd "${root_dir}/releases"
    mapfile -t old_releases < <(ls -1dt */ 2>/dev/null | tail -n +$((keep_releases + 1)) || true)
    if (( ${#old_releases[@]} > 0 )); then
        rm -rf -- "${old_releases[@]}"
    fi
fi
EOF

    log_ok "[${label}] Deployment complete"
}

print_summary() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}   Production Deployment Complete${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${BLUE}Version:${NC} ${VERSION}"
    echo -e "${BLUE}Targets:${NC} ${TARGET}"
    echo -e "${BLUE}Archive:${NC} ${ARCHIVE_PATH}"
    echo ""
    echo -e "${BLUE}Monitoring:${NC}"
    if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
        echo "  MiniPC logs: ssh -p ${MINIPC_SSH_PORT} ${MINIPC_USER}@${MINIPC_HOST} 'cd ${MINIPC_DEST_DIR}/current && ${MINIPC_DOCKER_LOG_CMD}'"
    fi
    if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
        echo "  Raspberry Pi logs: ssh -p ${RPI_SSH_PORT} ${RPI_USER}@${RPI_HOST} 'cd ${RPI_DEST_DIR}/current && ${RPI_DOCKER_LOG_CMD}'"
    fi
    echo ""
}

case "$TARGET" in
    all|minipc|ras)
        ;;
    *)
        usage
        exit 1
        ;;
esac

print_header
require_file "$ENV_FILE"

echo -e "${YELLOW}WARNING: this will deploy to production targets${NC}"
echo -e "${YELLOW}Config file: ${ENV_FILE}${NC}"
echo ""
read -r -p "Continue with production deployment? [y/N]: " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    log_warn "Deployment cancelled"
    exit 0
fi

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

MINIPC_PASSWORD="$(trim_whitespace "${MINIPC_PASSWORD:-}")"
RPI_PASSWORD="$(trim_whitespace "${RPI_PASSWORD:-}")"

PROJECT_NAME="${PROJECT_NAME:-car-calib}"
SSH_CONNECT_TIMEOUT_S="${SSH_CONNECT_TIMEOUT_S:-10}"
DEPLOY_KEEP_RELEASES="${DEPLOY_KEEP_RELEASES:-3}"
MINIPC_SSH_PORT="${MINIPC_SSH_PORT:-22}"
RPI_SSH_PORT="${RPI_SSH_PORT:-22}"
MINIPC_DEST_DIR="${MINIPC_DEST_DIR:-/opt/${PROJECT_NAME}/vision}"
RPI_DEST_DIR="${RPI_DEST_DIR:-/opt/${PROJECT_NAME}/rpi}"
MINIPC_COMPOSE_FILE="${MINIPC_COMPOSE_FILE:-docker-compose.vision.yml}"
RPI_COMPOSE_FILE="${RPI_COMPOSE_FILE:-docker-compose.rpi.yml}"
MINIPC_USE_SUDO_DOCKER="${MINIPC_USE_SUDO_DOCKER:-false}"
RPI_USE_SUDO_DOCKER="${RPI_USE_SUDO_DOCKER:-true}"
MINIPC_DOCKER_LOG_CMD="docker compose -f ${MINIPC_COMPOSE_FILE} logs -f"
RPI_DOCKER_LOG_CMD="docker compose -f ${RPI_COMPOSE_FILE} logs -f"
if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
    MINIPC_DOCKER_LOG_CMD="sudo ${MINIPC_DOCKER_LOG_CMD}"
fi
if [[ "${RPI_USE_SUDO_DOCKER}" == "true" ]]; then
    RPI_DOCKER_LOG_CMD="sudo ${RPI_DOCKER_LOG_CMD}"
fi

if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    require_vars MINIPC_HOST MINIPC_USER
fi
if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    require_vars RPI_HOST RPI_USER
fi

VERSION="$(date +%Y%m%d-%H%M%S)"
if git -C "$SCRIPT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    GIT_SHA="$(git -C "$SCRIPT_DIR" rev-parse --short HEAD)"
    VERSION="${VERSION}-${GIT_SHA}"
fi

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
ARCHIVE_PATH="${TMP_DIR}/${PROJECT_NAME}-${VERSION}.tar.gz"

log_step "[1/4] Creating deployment archive..."
create_archive "$ARCHIVE_PATH"
log_ok "Archive ready: ${ARCHIVE_PATH}"

echo ""
echo -e "${BLUE}Production Configuration:${NC}"
echo "  Version: ${VERSION}"
echo "  Target: ${TARGET}"
if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    echo "  MiniPC: ${MINIPC_USER}@${MINIPC_HOST}:${MINIPC_SSH_PORT} -> ${MINIPC_DEST_DIR}"
    if [[ -n "${MINIPC_PASSWORD}" ]]; then
        echo "  MiniPC auth: password"
    else
        echo "  MiniPC auth: ssh-key"
    fi
fi
if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    echo "  Raspberry Pi: ${RPI_USER}@${RPI_HOST}:${RPI_SSH_PORT} -> ${RPI_DEST_DIR}"
    if [[ -n "${RPI_PASSWORD}" ]]; then
        echo "  Raspberry Pi auth: password"
    else
        echo "  Raspberry Pi auth: ssh-key"
    fi
fi
echo ""

step_index=2
if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    log_step "[${step_index}/4] Deploying vision stack to MiniPC..."
    deploy_target "minipc" \
        "$MINIPC_HOST" \
        "$MINIPC_USER" \
        "$MINIPC_SSH_PORT" \
        "${MINIPC_PASSWORD:-}" \
        "$MINIPC_DEST_DIR" \
        "$MINIPC_COMPOSE_FILE" \
        "$MINIPC_USE_SUDO_DOCKER"
    step_index=$((step_index + 1))
fi

if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    log_step "[${step_index}/4] Deploying MQTT bridge to Raspberry Pi..."
    deploy_target "ras" \
        "$RPI_HOST" \
        "$RPI_USER" \
        "$RPI_SSH_PORT" \
        "${RPI_PASSWORD:-}" \
        "$RPI_DEST_DIR" \
        "$RPI_COMPOSE_FILE" \
        "$RPI_USE_SUDO_DOCKER"
fi

echo "${VERSION}" > "${SCRIPT_DIR}/.last_production_version"
date -Iseconds > "${SCRIPT_DIR}/.last_production_deploy"

print_summary
