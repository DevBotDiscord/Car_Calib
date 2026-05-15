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
CONTAINER_NAME_OVERRIDE="${3:-}"

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
  - List routes from MiniPC Docker container over SSH
  - Arrow-key select one route (shows route duration)
  - Action per route: sync / delete single / delete all
EOF
}

log_step() { echo -e "${BLUE}$1${NC}" >&2; }
log_ok() { echo -e "${GREEN}$1${NC}" >&2; }
log_warn() { echo -e "${YELLOW}$1${NC}" >&2; }
log_err() { echo -e "${RED}$1${NC}" >&2; }

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

remote_cmd() {
    ssh_base_cmd "$MINIPC_PASSWORD" "$MINIPC_SSH_PORT" "${MINIPC_USER}@${MINIPC_HOST}" "$1"
}

resolve_remote_container() {
    local container="${CONTAINER_NAME_OVERRIDE:-}"
    local project="$MINIPC_COMPOSE_PROJECT_NAME"
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi

    if [[ -n "$container" ]]; then
        printf '%s' "$container"
        return
    fi

    container="$(remote_cmd "${docker_prefix} ps --format '{{.Names}}' | sed -n '1p'")"
    if [[ -n "$project" ]]; then
        container="$(remote_cmd "${docker_prefix} ps --format '{{.Names}}' | grep -E '^${project}-vision-[0-9]+\$' | head -n 1 || true")"
    fi
    if [[ -z "$container" ]]; then
        container="$(remote_cmd "${docker_prefix} ps --format '{{.Names}}' | grep -E 'vision' | head -n 1 || true")"
    fi
    printf '%s' "$container"
}

draw_menu() {
    local selected="$1"
    shift
    local items=("$@")
    local i

    printf '\033[H\033[2J' >&2
    echo -e "${BLUE}========================================${NC}" >&2
    echo -e "${BLUE}   Route Pull From MiniPC (Arrow)${NC}" >&2
    echo -e "${BLUE}========================================${NC}" >&2
    echo "" >&2
    echo -e "MiniPC:     ${YELLOW}${MINIPC_USER}@${MINIPC_HOST}:${MINIPC_SSH_PORT}${NC}" >&2
    echo -e "Remote root:${YELLOW}${ROUTE_ROOT}${NC}" >&2
    echo -e "Local dest: ${YELLOW}${DEST}${NC}" >&2
    echo "" >&2
    echo "Select one route:" >&2
    echo "" >&2

    for i in "${!items[@]}"; do
        if [[ "$i" -eq "$selected" ]]; then
            echo -e "  ${GREEN}> ${items[$i]}${NC}" >&2
        else
            echo "    ${items[$i]}" >&2
        fi
    done

    echo "" >&2
    echo "Up/Down: move  Enter: select  q: quit" >&2
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
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi
    remote_cmd "${docker_prefix} exec '${REMOTE_CONTAINER}' sh -lc \"if [ -d '$ROUTE_ROOT' ]; then find '$ROUTE_ROOT' -mindepth 1 -maxdepth 1 -type d -name 'route-*' -printf '%f\\n' | sort -r; fi\""
}

list_remote_route_rows() {
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi
    remote_cmd "${docker_prefix} exec '${REMOTE_CONTAINER}' sh -lc \"python3 - <<'PY'
import json, os, glob
root = '${ROUTE_ROOT}'
for route in sorted(glob.glob(os.path.join(root, 'route-*')), reverse=True):
    name = os.path.basename(route)
    seconds = ''
    summary = os.path.join(route, 'route_summary.json')
    if os.path.isfile(summary):
        try:
            with open(summary, 'r', encoding='utf-8') as f:
                payload = json.load(f)
            total = payload.get('total_elapsed_seconds')
            if isinstance(total, (int, float)):
                seconds = f'{float(total):.1f}'
        except Exception:
            pass
    print(f'{name}|{seconds}')
PY\""
}

sync_remote_route() {
    local route_name="$1"
    mkdir -p "$DEST"
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi
    remote_cmd "${docker_prefix} exec '${REMOTE_CONTAINER}' sh -lc \"set -e; snap_root=/tmp/route-sync-snapshots; snap_dir=\\\"\\\${snap_root}/${route_name}\\\"; rm -rf \\\"\\\${snap_dir}\\\"; mkdir -p \\\"\\\${snap_root}\\\"; cp -a '$ROUTE_ROOT/${route_name}' \\\"\\\${snap_dir}\\\"; tar -C \\\"\\\${snap_root}\\\" -cf - '${route_name}'; rm -rf \\\"\\\${snap_dir}\\\"\"" | tar -x -C "$DEST"
}

delete_remote_route() {
    local route_name="$1"
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi
    remote_cmd "${docker_prefix} exec '${REMOTE_CONTAINER}' sh -lc \"rm -rf '$ROUTE_ROOT/$route_name'\""
}

delete_all_remote_routes() {
    local docker_prefix="docker"
    if [[ "${MINIPC_USE_SUDO_DOCKER}" == "true" ]]; then
        docker_prefix="sudo -n docker"
    fi
    remote_cmd "${docker_prefix} exec '${REMOTE_CONTAINER}' sh -lc \"find '$ROUTE_ROOT' -mindepth 1 -maxdepth 1 -type d -name 'route-*' -exec rm -rf {} +\""
}

format_duration() {
    local seconds="$1"
    if [[ -z "$seconds" ]]; then
        printf '%s' "n/a"
        return
    fi
    awk -v s="$seconds" 'BEGIN {
        sec = int(s + 0.5);
        h = int(sec / 3600);
        m = int((sec % 3600) / 60);
        r = sec % 60;
        if (h > 0) printf "%dh%02dm%02ds", h, m, r;
        else if (m > 0) printf "%dm%02ds", m, r;
        else printf "%ds", r;
    }'
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

MINIPC_HOST="$(load_env_var_from_file "MINIPC_HOST")"
MINIPC_USER="$(load_env_var_from_file "MINIPC_USER")"
MINIPC_SSH_PORT="$(load_env_var_from_file "MINIPC_SSH_PORT")"
MINIPC_PASSWORD="$(load_env_var_from_file "MINIPC_PASSWORD")"
MINIPC_COMPOSE_PROJECT_NAME="$(load_env_var_from_file "MINIPC_COMPOSE_PROJECT_NAME")"
MINIPC_USE_SUDO_DOCKER="$(load_env_var_from_file "MINIPC_USE_SUDO_DOCKER")"

if [[ -z "$MINIPC_SSH_PORT" ]]; then MINIPC_SSH_PORT=22; fi
if [[ -z "$MINIPC_USE_SUDO_DOCKER" ]]; then MINIPC_USE_SUDO_DOCKER=false; fi
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
REMOTE_CONTAINER="$(resolve_remote_container)"
if [[ -z "$REMOTE_CONTAINER" ]]; then
    log_err "No running container found on MiniPC (expected vision container)."
    exit 1
fi

if ! list_remote_routes >/dev/null 2>&1; then
    log_err "Cannot access route root in container '${REMOTE_CONTAINER}': $ROUTE_ROOT"
    exit 1
fi

mapfile -t ROUTE_ROWS < <(list_remote_route_rows)
if (( ${#ROUTE_ROWS[@]} == 0 )); then
    log_warn "No route directories found on MiniPC in: $ROUTE_ROOT"
    exit 0
fi

ROUTES=()
ROUTE_DURATIONS=()
ROUTE_MENU_ITEMS=()
for row in "${ROUTE_ROWS[@]}"; do
    route_name="${row%%|*}"
    route_seconds="${row#*|}"
    route_duration="$(format_duration "$route_seconds")"
    ROUTES+=("$route_name")
    ROUTE_DURATIONS+=("$route_duration")
    ROUTE_MENU_ITEMS+=("${route_name}  [duration: ${route_duration}]")
done

if ! SELECTED_LABEL="$(select_route_with_arrows "${ROUTE_MENU_ITEMS[@]}")"; then
    log_warn "Selection cancelled."
    exit 0
fi

SELECTED_INDEX=-1
for i in "${!ROUTE_MENU_ITEMS[@]}"; do
    if [[ "${ROUTE_MENU_ITEMS[$i]}" == "$SELECTED_LABEL" ]]; then
        SELECTED_INDEX="$i"
        break
    fi
done
if (( SELECTED_INDEX < 0 )); then
    log_err "Failed to resolve selected route index."
    exit 1
fi
SELECTED_ROUTE="${ROUTES[$SELECTED_INDEX]}"
SELECTED_DURATION="${ROUTE_DURATIONS[$SELECTED_INDEX]}"

echo "" >&2
echo -e "${BLUE}Selected route:${NC} ${SELECTED_ROUTE} (duration=${SELECTED_DURATION})" >&2
echo "Choose action:" >&2
echo "  1) Sync route to local destination" >&2
echo "  2) Delete this route on MiniPC container" >&2
echo "  3) Delete ALL routes on MiniPC container" >&2
echo "  q) Cancel" >&2
read -r -p "Action [1/2/3/q]: " action

case "$action" in
    1)
        read -r -p "Pull this route from MiniPC to '${DEST}'? [y/N]: " confirm_sync
        if [[ ! "$confirm_sync" =~ ^[Yy]$ ]]; then
            log_warn "Sync cancelled."
            exit 0
        fi
        log_step "Pulling route ${SELECTED_ROUTE} from MiniPC..."
        sync_remote_route "$SELECTED_ROUTE"
        log_ok "Done: ${DEST}/${SELECTED_ROUTE}"
        ;;
    2)
        read -r -p "Delete route '${SELECTED_ROUTE}' on MiniPC? [y/N]: " confirm_del
        if [[ ! "$confirm_del" =~ ^[Yy]$ ]]; then
            log_warn "Delete cancelled."
            exit 0
        fi
        log_step "Deleting route ${SELECTED_ROUTE} on MiniPC..."
        delete_remote_route "$SELECTED_ROUTE"
        log_ok "Deleted: ${SELECTED_ROUTE}"
        ;;
    3)
        read -r -p "Delete ALL route-* under '${ROUTE_ROOT}' on MiniPC? [y/N]: " confirm_all
        if [[ ! "$confirm_all" =~ ^[Yy]$ ]]; then
            log_warn "Delete-all cancelled."
            exit 0
        fi
        log_step "Deleting all routes on MiniPC..."
        delete_all_remote_routes
        log_ok "Deleted all route-* in ${ROUTE_ROOT}"
        ;;
    q|Q)
        log_warn "Cancelled."
        exit 0
        ;;
    *)
        log_warn "Unknown action, cancelled."
        exit 0
        ;;
esac
