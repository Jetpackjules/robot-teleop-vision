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

if (-not $PackagedRuntime) {
    $ExtensionManifest = "res://native/realsense_shared_memory/realsense_shared_memory.gdextension"
    $ExtensionList = Join-Path $Root ".godot\extension_list.cfg"
    $NeedsGodotImport = -not (Test-Path $ExtensionList)
    if (-not $NeedsGodotImport) {
        $NeedsGodotImport = -not (Get-Content $ExtensionList -Raw).Contains($ExtensionManifest)
    }

    if ($NeedsGodotImport) {
        $GodotExecutable = [string](& ".venv\Scripts\robot-teleop.exe" godot-path)
        if ($LASTEXITCODE -ne 0) {
            throw "Godot 4.6+ was not found. Add Godot to PATH or set godot.executable in config\local.toml, then rerun this script."
        }
        $GodotExecutable = $GodotExecutable.Trim()
        if (-not $GodotExecutable) {
            throw "Godot 4.6+ was not found. Add Godot to PATH or set godot.executable in config\local.toml, then rerun this script."
        }

        $GodotImportExecutable = $GodotExecutable
        if ([IO.Path]::GetExtension($GodotExecutable) -ieq ".exe" -and -not [IO.Path]::GetFileNameWithoutExtension($GodotExecutable).EndsWith("_console", [StringComparison]::OrdinalIgnoreCase)) {
            $ConsoleCandidate = Join-Path ([IO.Path]::GetDirectoryName($GodotExecutable)) ([IO.Path]::GetFileNameWithoutExtension($GodotExecutable) + "_console.exe")
            if (Test-Path -LiteralPath $ConsoleCandidate) {
                $GodotImportExecutable = $ConsoleCandidate
            }
        }

        Write-Host "Importing the Godot project and registering native extensions..."
        & $GodotImportExecutable --headless --editor --path $Root --quit
        if ($LASTEXITCODE -ne 0) {
            throw "Godot project import failed. Review the output above before starting the runtime."
        }
    }

    if (-not (Test-Path $ExtensionList)) {
        throw "Godot did not create .godot\extension_list.cfg. The native RealSense extension is not registered."
    }
    if (-not (Get-Content $ExtensionList -Raw).Contains($ExtensionManifest)) {
        throw "Godot did not register the RealSense GDExtension. Rerun setup after resolving the Godot import errors above."
    }
    Write-Host "Verified Godot native-extension registration."
}

Write-Host "Setup complete."
if (-not (Get-Command cloudflared -ErrorAction SilentlyContinue)) {
    Write-Warning "cloudflared was not found. Install it for the default free public URL, or set stack.public_mode = 'off' for local-only use."
}
Write-Host "Next: .\.venv\Scripts\robot-teleop.exe doctor"
