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
AUTO_CONFIRM=false

usage() {
    cat <<'EOF'
Usage:
  ./deploy_production.sh [all|minipc|ras] [--yes] [--env <path>] [--no-build] [--env-only]

Targets:
  all     Deploy both vision (MiniPC) and Raspberry Pi bridge
  minipc  Deploy only the vision stack
  ras     Deploy only the Raspberry Pi MQTT bridge

Options:
  --yes         Skip interactive confirmation prompt
  --env <path>  Use custom environment file (default: .env.production)
  --no-build    Skip image build step and reuse existing local images on target
  --env-only    Upload only env file and recreate containers from current release
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

shell_quote() {
    local value="$1"
    printf "'%s'" "${value//\'/\'\\\'\'}"
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

validate_password_auth_support() {
    local label="$1"
    local password="$2"
    local has_sshpass=1
    local has_plink=1
    local has_pscp=1

    password="$(trim_whitespace "$password")"
    if [[ -z "$password" ]]; then
        return 0
    fi

    if find_command sshpass >/dev/null 2>&1; then
        return 0
    fi

    if find_command plink.exe plink >/dev/null 2>&1; then
        has_plink=0
    fi
    if find_command pscp.exe pscp >/dev/null 2>&1; then
        has_pscp=0
    fi

    if (( has_plink == 0 && has_pscp == 0 )); then
        return 0
    fi

    log_err "[${label}] Password auth is configured, but no supported helper tools were found."
    log_err "[${label}] Install sshpass, or install PuTTY and ensure both plink.exe and pscp.exe are in PATH."
    log_err "[${label}] Otherwise leave ${label^^}_PASSWORD blank and use SSH keys."
    exit 1
}

ssh_base_cmd() {
    local password="$1"
    local port="$2"
    local host_key="$3"
    local sshpass_bin=""
    local plink_bin=""
    shift 3

    password="$(trim_whitespace "$password")"
    host_key="$(trim_whitespace "$host_key")"

    if [[ -n "$password" ]]; then
        if sshpass_bin="$(find_command sshpass)"; then
            "$sshpass_bin" -p "$password" ssh -p "$port" -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
                -o StrictHostKeyChecking=accept-new "$@"
            return
        fi

        if plink_bin="$(find_command plink.exe plink)"; then
            if [[ -n "$host_key" ]]; then
                "$plink_bin" -P "$port" -pw "$password" -batch -hostkey "$host_key" "$@"
            else
                "$plink_bin" -P "$port" -pw "$password" -batch "$@"
            fi
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
    local host_key="$3"
    local sshpass_bin=""
    local pscp_bin=""
    shift 3

    password="$(trim_whitespace "$password")"
    host_key="$(trim_whitespace "$host_key")"

    if [[ -n "$password" ]]; then
        if sshpass_bin="$(find_command sshpass)"; then
            "$sshpass_bin" -p "$password" scp -P "$port" -o ConnectTimeout="${SSH_CONNECT_TIMEOUT_S}" \
                -o StrictHostKeyChecking=accept-new "$@"
            return
        fi

        if pscp_bin="$(find_command pscp.exe pscp)"; then
            if [[ -n "$host_key" ]]; then
                "$pscp_bin" -P "$port" -pw "$password" -batch -hostkey "$host_key" "$@"
            else
                "$pscp_bin" -P "$port" -pw "$password" -batch "$@"
            fi
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
    local gzip_bin=""
    local common_args=(
        --exclude='.git'
        --exclude='.pytest_cache'
        --exclude='__pycache__'
        --exclude='.venv'
        --exclude='venv'
        --exclude='.env'
        --exclude='.env.production'
        --exclude='.env.backup'
        --exclude='*.pyc'
        --exclude='*.pyo'
        --exclude='*.log'
        --exclude='.last_production_version'
        --exclude='.last_production_deploy'
        --exclude='videos'
        --exclude='videos/*'
    )

    if gzip_bin="$(find_command pigz)"; then
        tar "${common_args[@]}" -I "$gzip_bin" -cf "$archive_path" -C "$SCRIPT_DIR" .
    else
        tar "${common_args[@]}" -czf "$archive_path" -C "$SCRIPT_DIR" .
    fi
}

deploy_target() {
    local label="$1"
    local host="$2"
    local user="$3"
    local port="$4"
    local password="$5"
    local dest_dir="$6"
    local compose_file="$7"
    local compose_project_name="$8"
    local use_sudo_docker="$9"
    local use_sudo_remote="${10}"
    local host_key="${11}"
    local sudo_password="${12}"
    local skip_build="${13}"
    local env_only="${14}"

    local remote_archive="/tmp/${PROJECT_NAME}-${VERSION}-${label}.tar.gz"
    local remote_env="/tmp/${PROJECT_NAME}-${VERSION}-${label}.env"
    local compose_cmd=()
    local compose_project_name_safe=""
    local remote_bootstrap=""
    local ssh_check_output=""

    validate_password_auth_support "$label" "$password"
    host_key="$(trim_whitespace "$host_key")"
    sudo_password="$(trim_whitespace "$sudo_password")"
    compose_project_name_safe="$(trim_whitespace "$compose_project_name")"
    if [[ -z "$sudo_password" ]]; then
        sudo_password="$password"
    fi
    if [[ -z "$compose_project_name_safe" ]]; then
        compose_project_name_safe="${PROJECT_NAME}-${label}"
    fi

    log_step "[${label}] Checking SSH connectivity..."
    if ! ssh_check_output="$(ssh_base_cmd "$password" "$port" "$host_key" "${user}@${host}" exit 2>&1)"; then
        log_err "[${label}] Cannot connect to ${user}@${host}:${port}"
        log_err "[${label}] Check SSH key access or verify ${label^^}_PASSWORD and helper-tool availability."
        if [[ -n "$password" && -n "$host_key" ]]; then
            log_err "[${label}] Using pinned host key from ${label^^}_SSH_HOST_KEY."
        elif [[ -n "$password" ]]; then
            log_err "[${label}] For PuTTY/plink password mode, set ${label^^}_SSH_HOST_KEY=SHA256:... to trust the host non-interactively."
        fi
        if [[ -n "$ssh_check_output" ]]; then
            while IFS= read -r line; do
                [[ -n "$line" ]] && log_err "[${label}] ${line}"
            done <<< "$ssh_check_output"
        fi
        exit 1
    fi
    log_ok "[${label}] SSH connection OK"

    if [[ "${env_only}" == "true" ]]; then
        log_step "[${label}] Uploading env file only..."
    else
        log_step "[${label}] Uploading release bundle..."
        ssh_base_cmd "$password" "$port" "$host_key" "${user}@${host}" "mkdir -p '${dest_dir}/releases'"
        scp_base_cmd "$password" "$port" "$host_key" "$ARCHIVE_PATH" "${user}@${host}:${remote_archive}"
    fi
    scp_base_cmd "$password" "$port" "$host_key" "$ENV_FILE" "${user}@${host}:${remote_env}"
    log_ok "[${label}] Upload complete"

    log_step "[${label}] Deploying release ${VERSION}..."
    remote_bootstrap=$(
        printf '%s' \
            "DEST_DIR=$(shell_quote "$dest_dir") " \
            "COMPOSE_FILE=$(shell_quote "$compose_file") " \
            "COMPOSE_PROJECT_NAME=$(shell_quote "$compose_project_name_safe") " \
            "VERSION=$(shell_quote "$VERSION") " \
            "USE_SUDO_DOCKER=$(shell_quote "$use_sudo_docker") " \
            "USE_SUDO_REMOTE=$(shell_quote "$use_sudo_remote") " \
            "SUDO_PASSWORD=$(shell_quote "$sudo_password") " \
            "REMOTE_ARCHIVE=$(shell_quote "$remote_archive") " \
            "REMOTE_ENV=$(shell_quote "$remote_env") " \
            "KEEP_RELEASES=$(shell_quote "$DEPLOY_KEEP_RELEASES") " \
            "SKIP_BUILD=$(shell_quote "$skip_build") " \
            "ENV_ONLY=$(shell_quote "$env_only") " \
            "bash -s"
    )
    ssh_base_cmd "$password" "$port" "$host_key" "${user}@${host}" "$remote_bootstrap" <<'EOF'
set -euo pipefail

root_dir="${DEST_DIR%/}"
release_dir="${root_dir}/releases/${VERSION}"
current_dir="${root_dir}/current"

if [[ "${USE_SUDO_REMOTE}" == "true" || "${USE_SUDO_DOCKER}" == "true" ]]; then
    if [[ -n "${SUDO_PASSWORD}" ]]; then
        printf '%s\n' "${SUDO_PASSWORD}" | sudo -S -p '' true >/dev/null 2>&1 || {
            echo "Remote sudo authentication failed" >&2
            exit 1
        }
    else
        sudo -n true >/dev/null 2>&1 || {
            echo "Remote sudo access is required but no passwordless sudo is available" >&2
            exit 1
        }
    fi
fi

if [[ "${USE_SUDO_REMOTE}" == "true" ]]; then
    file_cmd=(sudo -n)
else
    file_cmd=()
fi

if [[ "${ENV_ONLY}" == "true" ]]; then
    if [[ ! -d "${current_dir}" ]]; then
        echo "Current release not found at ${current_dir}; cannot run --env-only." >&2
        exit 1
    fi
    "${file_cmd[@]}" cp "${REMOTE_ENV}" "${current_dir}/.env"
    "${file_cmd[@]}" cp "${REMOTE_ENV}" "${current_dir}/.env.production"
    "${file_cmd[@]}" rm -f "${REMOTE_ENV}"
else
    "${file_cmd[@]}" mkdir -p "${root_dir}/releases"
    "${file_cmd[@]}" rm -rf "${release_dir}"
    "${file_cmd[@]}" mkdir -p "${release_dir}"
    "${file_cmd[@]}" tar -xzf "${REMOTE_ARCHIVE}" -C "${release_dir}"
    "${file_cmd[@]}" cp "${REMOTE_ENV}" "${release_dir}/.env"
    "${file_cmd[@]}" cp "${REMOTE_ENV}" "${release_dir}/.env.production"
    "${file_cmd[@]}" ln -sfn "${release_dir}" "${current_dir}"
    "${file_cmd[@]}" rm -f "${REMOTE_ARCHIVE}" "${REMOTE_ENV}"
fi

if [[ "${USE_SUDO_DOCKER}" == "true" ]]; then
    docker_cmd=(sudo -n docker)
else
    docker_cmd=(docker)
fi

cd "${current_dir}"
compose_cmd=("${docker_cmd[@]}" compose -p "${COMPOSE_PROJECT_NAME}" -f "${COMPOSE_FILE}")
mapfile -t legacy_projects < <("${docker_cmd[@]}" ps --format '{{.Names}}' | sed -n "s/^\([0-9]\{8\}-[0-9]\{6\}-[0-9a-f]\{7\}\)-.*$/\1/p" | sort -u)
for legacy in "${legacy_projects[@]}"; do
    if [[ -z "${legacy}" || "${legacy}" == "${COMPOSE_PROJECT_NAME}" ]]; then
        continue
    fi
    "${docker_cmd[@]}" compose -p "${legacy}" -f "${COMPOSE_FILE}" down --remove-orphans || true
done
if [[ "${SKIP_BUILD}" == "true" ]]; then
    "${compose_cmd[@]}" up -d --remove-orphans
elif [[ "${ENV_ONLY}" == "true" ]]; then
    "${compose_cmd[@]}" up -d --force-recreate --remove-orphans
else
    "${compose_cmd[@]}" up --build -d --remove-orphans
fi
"${compose_cmd[@]}" ps

if [[ "${KEEP_RELEASES}" =~ ^[0-9]+$ ]] && (( KEEP_RELEASES > 0 )); then
    if [[ "${ENV_ONLY}" == "true" ]]; then
        exit 0
    fi
    cd "${root_dir}/releases"
    mapfile -t old_releases < <(ls -1dt */ 2>/dev/null | tail -n +$((KEEP_RELEASES + 1)) || true)
    if (( ${#old_releases[@]} > 0 )); then
        "${file_cmd[@]}" rm -rf -- "${old_releases[@]}"
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

while (( "$#" > 0 )); do
    case "$1" in
        all|minipc|ras)
            TARGET="$1"
            shift
            ;;
        --yes)
            AUTO_CONFIRM=true
            shift
            ;;
        --env)
            if [[ -z "${2:-}" ]]; then
                log_err "--env requires a file path"
                exit 1
            fi
            ENV_FILE="$2"
            shift 2
            ;;
        --no-build)
            DEPLOY_SKIP_BUILD=true
            shift
            ;;
        --env-only)
            DEPLOY_ENV_ONLY=true
            DEPLOY_SKIP_BUILD=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_err "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

case "$TARGET" in
    all|minipc|ras) ;;
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
if [[ "$AUTO_CONFIRM" != "true" ]]; then
    read -r -p "Continue with production deployment? [y/N]: " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
        log_warn "Deployment cancelled"
        exit 0
    fi
else
    log_warn "Auto-confirm enabled (--yes), continuing without prompt."
fi

set -a
# shellcheck disable=SC1090
source "$ENV_FILE"
set +a

MINIPC_PASSWORD="$(trim_whitespace "${MINIPC_PASSWORD:-}")"
RPI_PASSWORD="$(trim_whitespace "${RPI_PASSWORD:-}")"
MINIPC_SSH_HOST_KEY="$(trim_whitespace "${MINIPC_SSH_HOST_KEY:-}")"
RPI_SSH_HOST_KEY="$(trim_whitespace "${RPI_SSH_HOST_KEY:-}")"
MINIPC_SUDO_PASSWORD="$(trim_whitespace "${MINIPC_SUDO_PASSWORD:-}")"
RPI_SUDO_PASSWORD="$(trim_whitespace "${RPI_SUDO_PASSWORD:-}")"

PROJECT_NAME="${PROJECT_NAME:-car-calib}"
SSH_CONNECT_TIMEOUT_S="${SSH_CONNECT_TIMEOUT_S:-10}"
DEPLOY_KEEP_RELEASES="${DEPLOY_KEEP_RELEASES:-3}"
DEPLOY_SKIP_BUILD="${DEPLOY_SKIP_BUILD:-false}"
DEPLOY_ENV_ONLY="${DEPLOY_ENV_ONLY:-false}"
MINIPC_SSH_PORT="${MINIPC_SSH_PORT:-22}"
RPI_SSH_PORT="${RPI_SSH_PORT:-22}"
MINIPC_DEST_DIR="${MINIPC_DEST_DIR:-/opt/${PROJECT_NAME}/vision}"
RPI_DEST_DIR="${RPI_DEST_DIR:-/opt/${PROJECT_NAME}/rpi}"
MINIPC_COMPOSE_FILE="${MINIPC_COMPOSE_FILE:-docker-compose.vision.yml}"
RPI_COMPOSE_FILE="${RPI_COMPOSE_FILE:-docker-compose.rpi.yml}"
MINIPC_COMPOSE_PROJECT_NAME="${MINIPC_COMPOSE_PROJECT_NAME:-${PROJECT_NAME}-minipc}"
RPI_COMPOSE_PROJECT_NAME="${RPI_COMPOSE_PROJECT_NAME:-${PROJECT_NAME}-ras}"
MINIPC_USE_SUDO_DOCKER="${MINIPC_USE_SUDO_DOCKER:-false}"
RPI_USE_SUDO_DOCKER="${RPI_USE_SUDO_DOCKER:-true}"
MINIPC_USE_SUDO_REMOTE="${MINIPC_USE_SUDO_REMOTE:-true}"
RPI_USE_SUDO_REMOTE="${RPI_USE_SUDO_REMOTE:-true}"
MINIPC_DOCKER_LOG_CMD="docker compose -p ${MINIPC_COMPOSE_PROJECT_NAME} -f ${MINIPC_COMPOSE_FILE} logs -f"
RPI_DOCKER_LOG_CMD="docker compose -p ${RPI_COMPOSE_PROJECT_NAME} -f ${RPI_COMPOSE_FILE} logs -f"
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

if [[ "${DEPLOY_ENV_ONLY}" == "true" ]]; then
    log_step "[1/3] Env-only mode: skipping release archive build."
else
    log_step "[1/4] Creating deployment archive..."
    create_archive "$ARCHIVE_PATH"
    log_ok "Archive ready: ${ARCHIVE_PATH}"
fi

echo ""
echo -e "${BLUE}Production Configuration:${NC}"
echo "  Version: ${VERSION}"
echo "  Target: ${TARGET}"
echo "  Build images: $([[ "${DEPLOY_SKIP_BUILD}" == "true" ]] && echo "no (reuse existing)" || echo "yes")"
echo "  Env only: ${DEPLOY_ENV_ONLY}"
if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    echo "  MiniPC: ${MINIPC_USER}@${MINIPC_HOST}:${MINIPC_SSH_PORT} -> ${MINIPC_DEST_DIR}"
    if [[ -n "${MINIPC_PASSWORD}" ]]; then
        echo "  MiniPC auth: password"
    else
        echo "  MiniPC auth: ssh-key"
    fi
    echo "  MiniPC remote sudo: ${MINIPC_USE_SUDO_REMOTE}"
    if [[ -n "${MINIPC_SUDO_PASSWORD}" ]]; then
        echo "  MiniPC sudo auth: password"
    elif [[ -n "${MINIPC_PASSWORD}" ]]; then
        echo "  MiniPC sudo auth: ssh-password fallback"
    else
        echo "  MiniPC sudo auth: passwordless sudo required"
    fi
    if [[ -n "${MINIPC_SSH_HOST_KEY}" ]]; then
        echo "  MiniPC host key: pinned"
    fi
fi
if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    echo "  Raspberry Pi: ${RPI_USER}@${RPI_HOST}:${RPI_SSH_PORT} -> ${RPI_DEST_DIR}"
    if [[ -n "${RPI_PASSWORD}" ]]; then
        echo "  Raspberry Pi auth: password"
    else
        echo "  Raspberry Pi auth: ssh-key"
    fi
    echo "  Raspberry Pi remote sudo: ${RPI_USE_SUDO_REMOTE}"
    if [[ -n "${RPI_SUDO_PASSWORD}" ]]; then
        echo "  Raspberry Pi sudo auth: password"
    elif [[ -n "${RPI_PASSWORD}" ]]; then
        echo "  Raspberry Pi sudo auth: ssh-password fallback"
    else
        echo "  Raspberry Pi sudo auth: passwordless sudo required"
    fi
    if [[ -n "${RPI_SSH_HOST_KEY}" ]]; then
        echo "  Raspberry Pi host key: pinned"
    fi
fi
echo ""

deploy_steps=0
if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    deploy_steps=$((deploy_steps + 1))
fi
if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    deploy_steps=$((deploy_steps + 1))
fi

step_index=2
if [[ "${DEPLOY_ENV_ONLY}" == "true" ]]; then
    total_steps=$((deploy_steps + 1))
else
    total_steps=$((deploy_steps + 2))
fi
if [[ "$TARGET" == "all" || "$TARGET" == "minipc" ]]; then
    log_step "[${step_index}/${total_steps}] Deploying vision stack to MiniPC..."
    deploy_target "minipc" \
        "$MINIPC_HOST" \
        "$MINIPC_USER" \
        "$MINIPC_SSH_PORT" \
        "${MINIPC_PASSWORD:-}" \
        "$MINIPC_DEST_DIR" \
        "$MINIPC_COMPOSE_FILE" \
        "$MINIPC_COMPOSE_PROJECT_NAME" \
        "$MINIPC_USE_SUDO_DOCKER" \
        "$MINIPC_USE_SUDO_REMOTE" \
        "${MINIPC_SSH_HOST_KEY:-}" \
        "${MINIPC_SUDO_PASSWORD:-}" \
        "$DEPLOY_SKIP_BUILD" \
        "$DEPLOY_ENV_ONLY"
    step_index=$((step_index + 1))
fi

if [[ "$TARGET" == "all" || "$TARGET" == "ras" ]]; then
    log_step "[${step_index}/${total_steps}] Deploying MQTT bridge to Raspberry Pi..."
    deploy_target "ras" \
        "$RPI_HOST" \
        "$RPI_USER" \
        "$RPI_SSH_PORT" \
        "${RPI_PASSWORD:-}" \
        "$RPI_DEST_DIR" \
        "$RPI_COMPOSE_FILE" \
        "$RPI_COMPOSE_PROJECT_NAME" \
        "$RPI_USE_SUDO_DOCKER" \
        "$RPI_USE_SUDO_REMOTE" \
        "${RPI_SSH_HOST_KEY:-}" \
        "${RPI_SUDO_PASSWORD:-}" \
        "$DEPLOY_SKIP_BUILD" \
        "$DEPLOY_ENV_ONLY"
fi

echo "${VERSION}" > "${SCRIPT_DIR}/.last_production_version"
date -Iseconds > "${SCRIPT_DIR}/.last_production_deploy"

print_summary
