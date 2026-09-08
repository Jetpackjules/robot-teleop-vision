<#
.SYNOPSIS
Starts the isolated alignment scene and its local simulation controller.
.DESCRIPTION
Uses only AlignmentDemo.tscn and the standard-library simulation relay. It
does not start the production scene, cameras, follower, or tunnel. Webcam
and leader input remain opt-in actions in the controller page.
.PARAMETER GodotPath
Optional path to a Godot executable. Otherwise searches PATH, then the newest
Godot*_console.exe version in the current user's Downloads directory.
.PARAMETER NoBrowser
Starts the scene and relay without opening the controller page.
.PARAMETER DryRun
Resolves executables and prints the launch plan without starting processes,
opening a browser, checking live ports, or creating log files.
.EXAMPLE
.\scripts\start_alignment_demo.ps1
.EXAMPLE
.\scripts\start_alignment_demo.ps1 -GodotPath 'C:\Tools\Godot_console.exe' -NoBrowser
.EXAMPLE
.\scripts\start_alignment_demo.ps1 -DryRun
#>
[CmdletBinding()]
param(
    [string]$GodotPath,
    [switch]$NoBrowser,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'
$demoRoot = Split-Path -Parent $PSScriptRoot
$demoScene = 'res://godot/simulation/AlignmentDemo.tscn'
$demoRelay = Join-Path $demoRoot 'tools\serve_alignment_demo.py'
$demoLogDirectory = Join-Path $demoRoot '.teleop\alignment-demo'
$demoControllerUrl = 'http://127.0.0.1:14860/'
$demoHttpPort = 14860
$demoUdpPort = 14861

function Find-DemoGodot {
    param([string]$ExplicitPath)
    if ($ExplicitPath) {
        if (-not (Test-Path -LiteralPath $ExplicitPath -PathType Leaf)) {
            throw "Godot executable not found: $ExplicitPath"
        }
        return (Resolve-Path -LiteralPath $ExplicitPath).ProviderPath
    }
    foreach ($name in @('godot', 'godot4', 'godot_console', 'godot4_console')) {
        $command = Get-Command $name -CommandType Application -ErrorAction SilentlyContinue |
            Select-Object -First 1
        if ($command) { return $command.Source }
    }
    $profileDirectory = $env:USERPROFILE
    if (-not $profileDirectory) { $profileDirectory = [Environment]::GetFolderPath('UserProfile') }
    $downloads = Join-Path $profileDirectory 'Downloads'
    if (Test-Path -LiteralPath $downloads -PathType Container) {
        $candidates = Get-ChildItem -LiteralPath $downloads -Filter 'Godot*_console.exe' -File -Recurse -ErrorAction SilentlyContinue
        $latest = $candidates | Sort-Object -Property @{
            Expression = {
                if ($_.Name -match '(\d+\.\d+(?:\.\d+)?)') { [version]$Matches[1] }
                else { [version]'0.0' }
            }; Descending = $true
        }, @{ Expression = 'LastWriteTimeUtc'; Descending = $true } | Select-Object -First 1
        if ($latest) { return $latest.FullName }
    }
    throw 'Godot was not found. Pass -GodotPath with your Godot executable, add it to PATH, or extract a Godot console build into Downloads.'
}

function Find-DemoPython {
    $venvPython = Join-Path $demoRoot '.venv\Scripts\python.exe'
    if (Test-Path -LiteralPath $venvPython -PathType Leaf) { return $venvPython }
    $command = Get-Command python -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) { return $command.Source }
    throw 'Python was not found. Install Python 3.10+ or create the project .venv; the simulation relay needs only the standard library.'
}

function Join-DemoArguments {
    param([string[]]$Values)
    # Start-Process joins ArgumentList with spaces even when passed an array.
    # Quote Windows native arguments, including paths containing spaces.
    return (($Values | ForEach-Object {
        if ($_ -match '[\s"]' -or $_ -eq '') {
            '"' + (($_ -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"'
        } else { $_ }
    }) -join ' ')
}

function Get-DemoRelayConfig {
    return Invoke-RestMethod -Uri ($demoControllerUrl + 'config') -TimeoutSec 2 -MaximumRedirection 0 -ErrorAction Stop
}

function Test-DemoRelayIdentity {
    param($Config)
    # An arbitrary process returning udp_port alone must never be reused.
    return ($null -ne $Config -and
        $Config.service -ceq 'robot-teleop-alignment-demo' -and
        $Config.protocol -ceq 'alignment_demo' -and
        ($Config.version -is [int] -or $Config.version -is [long]) -and $Config.version -eq 1 -and
        ($Config.udp_port -is [int] -or $Config.udp_port -is [long]) -and $Config.udp_port -eq $demoUdpPort)
}

function Test-DemoTcpListener {
    $listeners = [Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetActiveTcpListeners()
    return [bool]($listeners | Where-Object { $_.Port -eq $demoHttpPort })
}

function Assert-DemoUdpAvailable {
    $listeners = [Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetActiveUdpListeners()
    if ($listeners | Where-Object { $_.Port -eq $demoUdpPort }) {
        throw "Simulation UDP port $demoUdpPort is already in use. Use the existing demo, or close its window before running this launcher again. No existing process has been stopped."
    }
}

if (-not (Test-Path -LiteralPath (Join-Path $demoRoot 'godot\simulation\AlignmentDemo.tscn') -PathType Leaf) -or
    -not (Test-Path -LiteralPath $demoRelay -PathType Leaf)) {
    throw 'The standalone alignment scene or simulation relay is missing from this checkout.'
}
$demoGodot = Find-DemoGodot -ExplicitPath $GodotPath
$demoPython = Find-DemoPython
$demoStamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$demoGodotLog = Join-Path $demoLogDirectory ("godot-$demoStamp.log")
$demoRelayOut = Join-Path $demoLogDirectory ("relay-$demoStamp.stdout.log")
$demoRelayError = Join-Path $demoLogDirectory ("relay-$demoStamp.stderr.log")
$demoGodotArguments = @('--path', $demoRoot, '--rendering-method', 'gl_compatibility', '--log-file', $demoGodotLog, $demoScene, '--', '--simulation-input-port=14861')
$demoRelayArguments = @('-u', $demoRelay, '--port', '14860', '--udp-port', '14861')

if ($DryRun) {
    [pscustomobject]@{
        GodotExecutable = $demoGodot
        GodotArguments = Join-DemoArguments $demoGodotArguments
        PythonExecutable = $demoPython
        RelayArguments = Join-DemoArguments $demoRelayArguments
        ControllerUrl = $demoControllerUrl
        OpenBrowser = -not $NoBrowser
        LogDirectory = $demoLogDirectory
        Note = 'Dry run: no processes, port checks, browser, or log files started.'
    }
    return
}

# Refuse to launch a second receiver or replace any unrelated listener.
Assert-DemoUdpAvailable
$demoReuseRelay = $false
if (Test-DemoTcpListener) {
    try { $demoConfig = Get-DemoRelayConfig }
    catch {
        throw "Port $demoHttpPort is occupied but its /config endpoint could not be verified. Use the existing service or close it yourself before retrying. No process has been stopped."
    }
    if (-not (Test-DemoRelayIdentity $demoConfig)) {
        throw "Port $demoHttpPort is occupied by an unrelated or older relay. Expected robot-teleop-alignment-demo / alignment_demo version 1, UDP $demoUdpPort. Close or update that service yourself; this launcher will not replace it."
    }
    $demoReuseRelay = $true
}

New-Item -ItemType Directory -Path $demoLogDirectory -Force | Out-Null
if ($demoReuseRelay) {
    Write-Host "Reusing the verified simulation relay at $demoControllerUrl"
} else {
    $demoRelayProcess = Start-Process -FilePath $demoPython -ArgumentList (Join-DemoArguments $demoRelayArguments) -WorkingDirectory $demoRoot -WindowStyle Hidden -RedirectStandardOutput $demoRelayOut -RedirectStandardError $demoRelayError -PassThru
    $demoHealthy = $false
    for ($attempt = 0; $attempt -lt 12; $attempt++) {
        $demoRelayProcess.Refresh()
        if ($demoRelayProcess.HasExited) { break }
        try {
            if (Test-DemoRelayIdentity (Get-DemoRelayConfig)) { $demoHealthy = $true; break }
        } catch { }
        Start-Sleep -Milliseconds 250
    }
    if (-not $demoHealthy) {
        throw "The new simulation relay did not become healthy. See $demoRelayError (PID $($demoRelayProcess.Id)). No existing service was stopped and Godot was not launched."
    }
    Write-Host "Started simulation relay PID $($demoRelayProcess.Id). Logs: $demoRelayOut"
}

# Another demo may have opened while the relay was starting.
Assert-DemoUdpAvailable
$demoGodotProcess = Start-Process -FilePath $demoGodot -ArgumentList (Join-DemoArguments $demoGodotArguments) -WorkingDirectory $demoRoot -WindowStyle Normal -PassThru
Start-Sleep -Milliseconds 500
$demoGodotProcess.Refresh()
if ($demoGodotProcess.HasExited) {
    throw "Godot exited during startup. Check $demoGodotLog. The verified local relay remains available."
}
Write-Host "Alignment demo PID $($demoGodotProcess.Id). Godot log: $demoGodotLog"
Write-Host "Controller: $demoControllerUrl"
Write-Host 'Head tracking and leader input stay off until you enable them in the controller.'
if (-not $NoBrowser) {
    try { Start-Process $demoControllerUrl | Out-Null }
    catch { Write-Warning "Could not open the browser. Open $demoControllerUrl manually in Chrome or Edge." }
}
