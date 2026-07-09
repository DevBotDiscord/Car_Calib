param(
    [switch]$Yes,
    [switch]$NoBuild,
    [string]$Env = "",
    [switch]$Help
)

$ErrorActionPreference = "Stop"

function Log-Step($Message) { Write-Host $Message -ForegroundColor Blue }
function Log-Ok($Message) { Write-Host $Message -ForegroundColor Green }
function Log-Warn($Message) { Write-Host $Message -ForegroundColor Yellow }
function Log-Err($Message) { Write-Host $Message -ForegroundColor Red }

function Read-DotEnv($Path) {
    $result = @{}
    foreach ($line in Get-Content -Encoding utf8 $Path) {
        $trimmed = $line.Trim()
        if ($trimmed -eq "" -or $trimmed.StartsWith("#") -or -not $trimmed.Contains("=")) { continue }
        $parts = $trimmed.Split("=", 2)
        $name = $parts[0].Trim()
        $value = $parts[1].Trim()
        if (($value.StartsWith('"') -and $value.EndsWith('"')) -or ($value.StartsWith("'") -and $value.EndsWith("'"))) {
            $value = $value.Substring(1, $value.Length - 2)
        }
        $result[$name] = $value
    }
    return $result
}

function Need-Cmd($Name) {
    $cmd = Get-Command $Name -ErrorAction SilentlyContinue
    if (-not $cmd) { throw "$Name not found in PATH" }
    return $cmd.Source
}

function Shell-Quote($Value) {
    return "'" + ($Value -replace "'", "'\''") + "'"
}

function Env-Value($Map, $Name, $Default) {
    if ($Map.ContainsKey($Name) -and $null -ne $Map[$Name]) {
        return [string]$Map[$Name]
    }
    return $Default
}

function Show-Usage {
    @"
Usage:
  .\deploy_production.bat [--yes] [--no-build] [--env <path>]

Windows behavior:
  - Uses native PowerShell + Windows OpenSSH by default.
  - If RPI_PASSWORD is set and PuTTY plink.exe/pscp.exe are installed, password upload is automatic.
  - Without PuTTY, OpenSSH may prompt for password. For no prompts, use SSH key auth and clear RPI_PASSWORD.

Env file:
  RPI_HOST, RPI_USER, RPI_PORT, RPI_PASSWORD, RPI_DEST_DIR
"@
}

if ($Help -or $args -contains "--help" -or $args -contains "-h") {
    Show-Usage
    exit 0
}

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir

$EnvFile = if ($Env) { $Env } else { Join-Path $ScriptDir ".env.control-direct" }
if (-not (Test-Path -LiteralPath $EnvFile)) {
    Log-Err "Env file not found: $EnvFile"
    exit 1
}

$cfg = Read-DotEnv $EnvFile
$ProjectName = if ($env:PROJECT_NAME) { $env:PROJECT_NAME } else { "car-calib-control-direct" }
$GitHash = "unknown"
try { $GitHash = (& git rev-parse --short HEAD 2>$null).Trim() } catch {}
if (-not $GitHash) { $GitHash = "unknown" }
$Version = (Get-Date).ToUniversalTime().ToString("yyyyMMdd-HHmmss") + "-" + $GitHash
$ComposeFile = "docker-compose.control-direct.yml"
$ContainerName = if ($env:CONTROL_DIRECT_CONTAINER_NAME) { $env:CONTROL_DIRECT_CONTAINER_NAME } else { "car-calib-control-direct" }

$RpiHost = (Env-Value $cfg "RPI_HOST" "").Trim()
$RpiUser = (Env-Value $cfg "RPI_USER" "root").Trim()
$RpiPort = (Env-Value $cfg "RPI_PORT" "22").Trim()
$RpiPassword = (Env-Value $cfg "RPI_PASSWORD" "").Trim()
$RpiDestDir = (Env-Value $cfg "RPI_DEST_DIR" "/opt/car-calib-control-direct").Trim()
$DashboardPort = (Env-Value $cfg "DASHBOARD_PORT" "8080").Trim()

if (-not $RpiHost) {
    Log-Err "RPI_HOST not set. Add it to $EnvFile"
    exit 1
}

$SshExe = Need-Cmd "ssh.exe"
$ScpExe = Need-Cmd "scp.exe"
$TarExe = Need-Cmd "tar.exe"
$Plink = Get-Command "plink.exe" -ErrorAction SilentlyContinue
$Pscp = Get-Command "pscp.exe" -ErrorAction SilentlyContinue
$UsePutty = $RpiPassword -and $Plink -and $Pscp

if ($RpiPassword -and -not $UsePutty) {
    Log-Warn "RPI_PASSWORD is set, but plink.exe/pscp.exe not found."
    Log-Warn "Using Windows OpenSSH. It may ask for password on each SSH/SCP call."
    Log-Warn "For password automation install PuTTY, or use SSH key auth and clear RPI_PASSWORD."
}

function Invoke-Remote($Command) {
    if ($UsePutty) {
        & $Plink.Source -ssh -P $RpiPort -pw $RpiPassword "$RpiUser@$RpiHost" $Command
    } else {
        & $SshExe -p $RpiPort -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 "$RpiUser@$RpiHost" $Command
    }
    if ($LASTEXITCODE -ne 0) { throw "remote command failed: $Command" }
}

function Copy-ToRemote($Local, $Remote) {
    if ($UsePutty) {
        & $Pscp.Source -P $RpiPort -pw $RpiPassword $Local "$RpiUser@${RpiHost}:$Remote"
    } else {
        & $ScpExe -P $RpiPort -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 $Local "$RpiUser@${RpiHost}:$Remote"
    }
    if ($LASTEXITCODE -ne 0) { throw "upload failed: $Local -> $Remote" }
}

Write-Host "========================================" -ForegroundColor Blue
Write-Host "   Control Direct Remote Deploy" -ForegroundColor Blue
Write-Host "========================================" -ForegroundColor Blue
Write-Host ""
Write-Host "  Target:  " -NoNewline; Write-Host "$RpiUser@${RpiHost}:$RpiPort" -ForegroundColor Blue
Write-Host "  Dest:    " -NoNewline; Write-Host $RpiDestDir -ForegroundColor Blue
Write-Host "  Version: " -NoNewline; Write-Host $Version -ForegroundColor Blue
Write-Host ""

if (-not $Yes) {
    $confirm = Read-Host "Deploy control-direct to Raspberry Pi? [y/N]"
    if ($confirm -notmatch "^[Yy]$") {
        Log-Warn "Cancelled"
        exit 0
    }
}

try {
    Log-Step "[1/4] Checking SSH connectivity..."
    Invoke-Remote "exit"
    Log-Ok "SSH OK"

    Log-Step "[2/4] Building source archive..."
    $ArchivePath = Join-Path $env:TEMP "$ProjectName-$Version.tar.gz"
    if (Test-Path -LiteralPath $ArchivePath) { Remove-Item -LiteralPath $ArchivePath -Force }

    $tarArgs = @(
        "--exclude=.git",
        "--exclude=.pytest_cache",
        "--exclude=__pycache__",
        "--exclude=.venv",
        "--exclude=venv",
        "--exclude=*.pyc",
        "--exclude=*.log",
        "--exclude=logs",
        "--exclude=routes",
        "--exclude=.env",
        "-czf",
        $ArchivePath,
        "-C",
        $ScriptDir,
        "."
    )
    & $TarExe @tarArgs
    if ($LASTEXITCODE -ne 0) { throw "tar failed" }
    $sizeMb = [math]::Round((Get-Item -LiteralPath $ArchivePath).Length / 1MB, 2)
    Log-Ok "Archive: $ArchivePath (${sizeMb} MB)"

    Log-Step "[3/4] Uploading to Raspberry Pi..."
    $RemoteEnv = "/tmp/$ProjectName-$Version.env"
    Invoke-Remote "mkdir -p $(Shell-Quote "$RpiDestDir/releases")"
    Copy-ToRemote $ArchivePath "/tmp/$ProjectName-$Version.tar.gz"
    Copy-ToRemote $EnvFile $RemoteEnv
    Log-Ok "Upload complete"

    Log-Step "[4/4] Deploying on Raspberry Pi..."
    $RemoteScript = @'
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
mkdir -p "$root_dir/data/logs" "$root_dir/data/routes" "$root_dir/data/models"
export RPI_DEST_DIR="$root_dir"

if ! command -v docker >/dev/null 2>&1; then
    echo "[remote] ERROR: Docker not installed."
    exit 1
fi

DOCKER_BIN="docker"
if ! docker ps >/dev/null 2>&1; then
    if sudo -n docker ps >/dev/null 2>&1; then
        DOCKER_BIN="sudo docker"
        echo "[remote] Using sudo docker"
    else
        echo "[remote] ERROR: docker not accessible"
        exit 1
    fi
fi

if $DOCKER_BIN compose version >/dev/null 2>&1; then
    COMPOSE_CMD="$DOCKER_BIN compose"
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE_CMD="docker-compose"
else
    echo "[remote] ERROR: docker compose not found"
    exit 1
fi

echo "[remote] Compose: $COMPOSE_CMD -f $COMPOSE_FILE"
$COMPOSE_CMD -f "$COMPOSE_FILE" down 2>/dev/null || true

BUILD_LOG="/tmp/car-calib-control-direct-build-${VERSION}.log"
echo "[remote] Build/start log: $BUILD_LOG"
if [ "$SKIP_BUILD" = "true" ]; then
    $COMPOSE_CMD -f "$COMPOSE_FILE" up -d --remove-orphans > "$BUILD_LOG" 2>&1 &
else
    $COMPOSE_CMD -f "$COMPOSE_FILE" up --build -d --remove-orphans > "$BUILD_LOG" 2>&1 &
fi
BUILD_PID=$!

DEADLINE=$((SECONDS + 180))
while kill -0 $BUILD_PID 2>/dev/null && [ "$SECONDS" -lt "$DEADLINE" ]; do
    if [ -s "$BUILD_LOG" ]; then
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
'@
    $RemoteScriptPath = Join-Path $env:TEMP "$ProjectName-$Version-remote.sh"
    [System.IO.File]::WriteAllText($RemoteScriptPath, $RemoteScript, [System.Text.Encoding]::ASCII)
    $RemoteScriptTarget = "/tmp/$ProjectName-$Version-remote.sh"
    Copy-ToRemote $RemoteScriptPath $RemoteScriptTarget

    $skip = if ($NoBuild) { "true" } else { "false" }
    $remoteCommand = "DEST_DIR=$(Shell-Quote $RpiDestDir) VERSION=$(Shell-Quote $Version) PROJECT_NAME=$(Shell-Quote $ProjectName) COMPOSE_FILE=$(Shell-Quote $ComposeFile) CONTAINER_NAME=$(Shell-Quote $ContainerName) SKIP_BUILD=$(Shell-Quote $skip) REMOTE_ENV=$(Shell-Quote $RemoteEnv) bash $RemoteScriptTarget; rc=`$?; rm -f $RemoteScriptTarget; exit `$rc"
    Invoke-Remote $remoteCommand

    Remove-Item -LiteralPath $ArchivePath -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $RemoteScriptPath -Force -ErrorAction SilentlyContinue

    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "   Control Direct Deploy Complete" -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "  Dashboard: " -NoNewline; Write-Host "http://${RpiHost}:$DashboardPort" -ForegroundColor Blue
    Write-Host "  Logs:      " -NoNewline; Write-Host "ssh $RpiUser@$RpiHost 'docker logs -f $ContainerName'" -ForegroundColor Blue
} catch {
    Log-Err $_.Exception.Message
    exit 1
}
