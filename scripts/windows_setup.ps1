$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
Set-Location $Root

if (-not (Test-Path ".venv")) {
    py -3.11 -m venv .venv
}
& ".venv\Scripts\python.exe" -m pip install --upgrade pip
& ".venv\Scripts\python.exe" -m pip install -e ".[realsense,calibration]"

if (-not (Test-Path "config\local.toml")) {
    & ".venv\Scripts\python.exe" -m robot_teleop.cli init --example vision_only
}

$PackagedRuntime = Test-Path "RobotTeleopVision.exe"
if ($PackagedRuntime) {
    $Config = Get-Content "config\local.toml" -Raw
    $Config = $Config -replace 'executable = ""', 'executable = "RobotTeleopVision.exe"'
    $Config = $Config -replace 'packaged_runtime = false', 'packaged_runtime = true'
    Set-Content "config\local.toml" $Config -Encoding UTF8
}

Write-Host "Setup complete."
if (-not (Get-Command cloudflared -ErrorAction SilentlyContinue)) {
    Write-Warning "cloudflared was not found. Install it for the default free public URL, or set stack.public_mode = 'off' for local-only use."
}
Write-Host "Next: .\.venv\Scripts\robot-teleop.exe doctor"
