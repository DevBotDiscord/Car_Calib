#!/usr/bin/env bash

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="${ENV_FILE:-${SCRIPT_DIR}/.env.production}"
ROUTE_ROOT_DEFAULT="${SCRIPT_DIR}/logs/routes"
ROUTE_ROOT_RUNTIME_DEFAULT="/data/routes"
DEST="${1:-}"
ROUTE_ROOT_OVERRIDE="${2:-}"

usage() {
    cat <<'EOF'
Usage:
  ./sync_routes.sh [destination] [route_root]

Examples:
  ./sync_routes.sh /mnt/usb/routes_export
  ./sync_routes.sh user@10.0.0.20:/data/routes
  ./sync_routes.sh /mnt/usb/routes_export /data/routes
  ./sync_routes.sh                  # read destination from .env.production

Controls:
  Arrow Up/Down: Move selection
  Enter: Select route to sync
  q: Quit
EOF
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

load_env_var_from_file() {
    local key="$1"
    local value=""
    if [[ -f "$ENV_FILE" ]]; then
        value="$(sed -n "s/^[[:space:]]*${key}[[:space:]]*=[[:space:]]*//p" "$ENV_FILE" | tail -n 1)"
        value="$(trim_whitespace "$value")"
        value="${value%\"}"
        value="${value#\"}"
        value="${value%\'}"
        value="${value#\'}"
    fi
    printf '%s' "$value"
}

load_route_root_from_env() {
    load_env_var_from_file "ROUTE_LOG_ROOT"
}

load_dest_from_env() {
    local value
    value="$(load_env_var_from_file "ROUTE_SYNC_DEST")"
    if [[ -n "$value" ]]; then
        printf '%s' "$value"
        return
    fi
    value="$(load_env_var_from_file "ROUTE_EXPORT_DEST")"
    printf '%s' "$value"
}

resolve_route_root() {
    local root=""

    if [[ -n "$ROUTE_ROOT_OVERRIDE" ]]; then
        root="$ROUTE_ROOT_OVERRIDE"
    else
        root="$(load_route_root_from_env)"
    fi

    if [[ -n "$root" ]]; then
        if [[ "$root" != /* ]]; then
            root="${SCRIPT_DIR}/${root}"
        fi
        printf '%s' "$root"
        return
    fi

    # Fallback order:
    # 1) runtime default path used by main.py in container (/data/routes)
    # 2) repo-local legacy path (./logs/routes)
    if [[ -d "$ROUTE_ROOT_RUNTIME_DEFAULT" ]]; then
        printf '%s' "$ROUTE_ROOT_RUNTIME_DEFAULT"
        return
    fi

    printf '%s' "$ROUTE_ROOT_DEFAULT"
}

resolve_existing_route_root() {
    local preferred_root="$1"
    local candidate=""
    local candidates=()

    if [[ -n "$preferred_root" ]]; then
        candidates+=("$preferred_root")
    fi
    candidates+=("$ROUTE_ROOT_RUNTIME_DEFAULT")
    candidates+=("$ROUTE_ROOT_DEFAULT")
    candidates+=("${SCRIPT_DIR}/route")
    candidates+=("${SCRIPT_DIR}/logs/routes")

    for candidate in "${candidates[@]}"; do
        if [[ -d "$candidate" ]]; then
            printf '%s' "$candidate"
            return 0
        fi
    done
    return 1
}

list_routes() {
    local route_root="$1"
    find "$route_root" -mindepth 1 -maxdepth 1 -type d -name 'route-*' -printf '%P\n' | sort -r
}

draw_menu() {
    local selected="$1"
    shift
    local items=("$@")

    printf '\033[H\033[2J'
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}      Route Sync Selector (Arrow)${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "Destination: ${YELLOW}${DEST}${NC}"
    echo -e "Route root:  ${YELLOW}${ROUTE_ROOT}${NC}"
    echo ""
    echo "Select one route:"
    echo ""

    local i
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
                    if (( selected < 0 )); then
                        selected=$((${#items[@]} - 1))
                    fi
                    ;;
                "[B")
                    ((selected++))
                    if (( selected >= ${#items[@]} )); then
                        selected=0
                    fi
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

sync_route() {
    local route_root="$1"
    local route_name="$2"
    local src_dir="${route_root}/${route_name}"
    local rsync_cmd=""

    if ! command -v rsync >/dev/null 2>&1; then
        if [[ "$DEST" == *:* ]]; then
            log_err "rsync is required for remote destination sync."
            exit 1
        fi
        log_warn "rsync not found. Falling back to cp -a for local destination."
        mkdir -p "${DEST}/${route_name}"
        cp -a "${src_dir}/." "${DEST}/${route_name}/"
        log_ok "Route synced to ${DEST}/${route_name}"
        return
    fi

    if [[ "$DEST" == *:* ]]; then
        rsync_cmd="rsync -az --progress --mkpath"
        log_step "Syncing remote route ${route_name} ..."
        # shellcheck disable=SC2086
        $rsync_cmd "${src_dir}/" "${DEST}/${route_name}/"
        log_ok "Remote sync done: ${DEST}/${route_name}"
    else
        mkdir -p "$DEST"
        rsync_cmd="rsync -a --progress"
        log_step "Syncing local route ${route_name} ..."
        # shellcheck disable=SC2086
        $rsync_cmd "${src_dir}/" "${DEST}/${route_name}/"
        log_ok "Local sync done: ${DEST}/${route_name}"
    fi
}

if [[ "$DEST" == "-h" || "$DEST" == "--help" ]]; then
    usage
    exit 0
fi

if [[ -z "$DEST" ]]; then
    DEST="$(load_dest_from_env)"
fi

if [[ -z "$DEST" ]]; then
    log_err "Missing destination."
    log_err "Provide destination as CLI arg or set ROUTE_SYNC_DEST in ${ENV_FILE}."
    usage
    exit 1
fi

ROUTE_ROOT="$(resolve_route_root)"
if [[ ! -d "$ROUTE_ROOT" ]]; then
    original_root="$ROUTE_ROOT"
    if ROUTE_ROOT="$(resolve_existing_route_root "$ROUTE_ROOT")"; then
        log_warn "Route root not found: $original_root"
        log_warn "Falling back to existing route root: $ROUTE_ROOT"
    else
        log_err "Route root not found: $original_root"
        log_err "No fallback route root exists. Checked: /data/routes, ./logs/routes, ./route"
        exit 1
    fi
fi

mapfile -t ROUTES < <(list_routes "$ROUTE_ROOT")
if (( ${#ROUTES[@]} == 0 )); then
    log_warn "No route directories found in: $ROUTE_ROOT"
    log_warn "Tip: set ROUTE_LOG_ROOT in ${ENV_FILE} or pass route root explicitly:"
    log_warn "  ./sync_routes.sh <dest> <route_root>"
    exit 0
fi

if ! SELECTED_ROUTE="$(select_route_with_arrows "${ROUTES[@]}")"; then
    log_warn "Selection cancelled."
    exit 0
fi

echo ""
echo -e "${BLUE}Selected route:${NC} ${SELECTED_ROUTE}"
read -r -p "Sync this route to '${DEST}'? [y/N]: " confirm
if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
    log_warn "Sync cancelled."
    exit 0
fi

sync_route "$ROUTE_ROOT" "$SELECTED_ROUTE"
