#!/usr/bin/env bash

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-${SCRIPT_DIR}/.env.production}"
DEST="${1:-}"
ROUTE_ROOT_OVERRIDE="${2:-}"

usage() {
    cat <<'EOF'
Usage:
  ./sync_routes.sh [local_destination] [remote_route_root]

Examples:
  ./sync_routes.sh ./route
  ./sync_routes.sh /mnt/data/routes
  ./sync_routes.sh ./route /data/routes
  ./sync_routes.sh   # read defaults from .env.production

Behavior:
  - List routes from MiniPC over SSH
  - Arrow-key select one route
  - Pull selected route from MiniPC to local machine
EOF
}

log_step() { echo -e "${BLUE}$1${NC}"; }
log_ok() { echo -e "${GREEN}$1${NC}"; }
log_warn() { echo -e "${YELLOW}$1${NC}"; }
log_err() { echo -e "${RED}$1${NC}"; }

trim_whitespace() {
    local value="$1"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"
    printf '%s' "$value"
}

load_env_var_from_file() {
    local key="$1"
    local value=""
    if [[ -f "$ENV_FILE" ]]; then
        value="$(sed -n "s/^[[:space:]]*${key}[[:space:]]*=[[:space:]]*//p" "$ENV_FILE" | tail -n 1)"
        value="$(trim_whitespace "$value")"
        value="${value%\"}"; value="${value#\"}"
        value="${value%\'}"; value="${value#\'}"
    fi
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

ssh_base_cmd() {
    local password="$1"
    local port="$2"
    shift 2

    if [[ -n "$password" ]]; then
        local sshpass_bin=""
        if sshpass_bin="$(find_command sshpass)"; then
            "$sshpass_bin" -p "$password" ssh -p "$port" -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$@"
            return
        fi
    fi
    ssh -p "$port" -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$@"
}

load_dest_from_env() {
    local value
    value="$(load_env_var_from_file "ROUTE_SYNC_DEST")"
    if [[ -n "$value" ]]; then
        printf '%s' "$value"
        return
    fi
    value="$(load_env_var_from_file "ROUTE_EXPORT_DEST")"
    if [[ -n "$value" ]]; then
        printf '%s' "$value"
        return
    fi
    printf '%s' "${SCRIPT_DIR}/route"
}

draw_menu() {
    local selected="$1"
    shift
    local items=("$@")
    local i

    printf '\033[H\033[2J'
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}   Route Pull From MiniPC (Arrow)${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "MiniPC:     ${YELLOW}${MINIPC_USER}@${MINIPC_HOST}:${MINIPC_SSH_PORT}${NC}"
    echo -e "Remote root:${YELLOW}${ROUTE_ROOT}${NC}"
    echo -e "Local dest: ${YELLOW}${DEST}${NC}"
    echo ""
    echo "Select one route:"
    echo ""

    for i in "${!items[@]}"; do
        if [[ "$i" -eq "$selected" ]]; then
            echo -e "  ${GREEN}> ${items[$i]}${NC}"
        else
            echo "    ${items[$i]}"
        fi
    done

    echo ""
    echo "Up/Down: move  Enter: select  q: quit"
}

select_route_with_arrows() {
    local items=("$@")
    local selected=0
    local key=""

    while true; do
        draw_menu "$selected" "${items[@]}"
        IFS= read -rsn1 key

        if [[ "$key" == $'\x1b' ]]; then
            IFS= read -rsn2 key || true
            case "$key" in
                "[A")
                    ((selected--))
                    if (( selected < 0 )); then selected=$((${#items[@]} - 1)); fi
                    ;;
                "[B")
                    ((selected++))
                    if (( selected >= ${#items[@]} )); then selected=0; fi
                    ;;
            esac
        elif [[ -z "$key" ]]; then
            printf '%s\n' "${items[$selected]}"
            return 0
        elif [[ "$key" == "q" || "$key" == "Q" ]]; then
            return 1
        fi
    done
}

list_remote_routes() {
    ssh_base_cmd "$MINIPC_PASSWORD" "$MINIPC_SSH_PORT" "${MINIPC_USER}@${MINIPC_HOST}" \
        "if [ -d '$ROUTE_ROOT' ]; then find '$ROUTE_ROOT' -mindepth 1 -maxdepth 1 -type d -name 'route-*' -printf '%f\n' | sort -r; fi"
}

sync_remote_route() {
    local route_name="$1"
    local remote_src="${MINIPC_USER}@${MINIPC_HOST}:${ROUTE_ROOT}/${route_name}/"
    local local_dst="${DEST}/${route_name}/"

    mkdir -p "$DEST"
    if command -v rsync >/dev/null 2>&1; then
        if [[ -n "$MINIPC_PASSWORD" ]] && command -v sshpass >/dev/null 2>&1; then
            sshpass -p "$MINIPC_PASSWORD" rsync -az --progress -e "ssh -p ${MINIPC_SSH_PORT} -o StrictHostKeyChecking=accept-new" "$remote_src" "$local_dst"
        else
            rsync -az --progress -e "ssh -p ${MINIPC_SSH_PORT} -o StrictHostKeyChecking=accept-new" "$remote_src" "$local_dst"
        fi
    else
        scp -r -P "$MINIPC_SSH_PORT" "$remote_src" "$local_dst"
    fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

MINIPC_HOST="$(load_env_var_from_file "MINIPC_HOST")"
MINIPC_USER="$(load_env_var_from_file "MINIPC_USER")"
MINIPC_SSH_PORT="$(load_env_var_from_file "MINIPC_SSH_PORT")"
MINIPC_PASSWORD="$(load_env_var_from_file "MINIPC_PASSWORD")"

if [[ -z "$MINIPC_SSH_PORT" ]]; then MINIPC_SSH_PORT=22; fi
if [[ -z "$MINIPC_HOST" || -z "$MINIPC_USER" ]]; then
    log_err "Missing MINIPC_HOST or MINIPC_USER in ${ENV_FILE}."
    exit 1
fi

if [[ -z "$DEST" ]]; then
    DEST="$(load_dest_from_env)"
fi
if [[ "$DEST" != /* ]]; then
    DEST="${SCRIPT_DIR}/${DEST}"
fi

if [[ -n "$ROUTE_ROOT_OVERRIDE" ]]; then
    ROUTE_ROOT="$ROUTE_ROOT_OVERRIDE"
else
    ROUTE_ROOT="$(load_env_var_from_file "ROUTE_LOG_ROOT")"
fi
if [[ -z "$ROUTE_ROOT" ]]; then
    ROUTE_ROOT="/data/routes"
fi

log_step "Checking MiniPC route root..."
if ! ssh_base_cmd "$MINIPC_PASSWORD" "$MINIPC_SSH_PORT" "${MINIPC_USER}@${MINIPC_HOST}" "test -d '$ROUTE_ROOT'"; then
    log_err "Remote route root not found on MiniPC: $ROUTE_ROOT"
    exit 1
fi

mapfile -t ROUTES < <(list_remote_routes)
if (( ${#ROUTES[@]} == 0 )); then
    log_warn "No route directories found on MiniPC in: $ROUTE_ROOT"
    exit 0
fi

if ! SELECTED_ROUTE="$(select_route_with_arrows "${ROUTES[@]}")"; then
    log_warn "Selection cancelled."
    exit 0
fi

echo ""
echo -e "${BLUE}Selected route:${NC} ${SELECTED_ROUTE}"
read -r -p "Pull this route from MiniPC to '${DEST}'? [y/N]: " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    log_warn "Sync cancelled."
    exit 0
fi

log_step "Pulling route ${SELECTED_ROUTE} from MiniPC..."
sync_remote_route "$SELECTED_ROUTE"
log_ok "Done: ${DEST}/${SELECTED_ROUTE}"
