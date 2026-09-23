<#
.SYNOPSIS
    Creates the project virtual environment (.venv) and installs requirements.txt.

.DESCRIPTION
    Uses the Windows "py" launcher to find a supported Python (3.12 preferred,
    then 3.13, 3.11, 3.10, 3.14). An existing .venv is reused and updated.
    On Linux use ./setup.sh instead.

.EXAMPLE
    .\setup.ps1                 # automatic Python selection
    .\setup.ps1 -Python 3.11    # force a Python version
    .\setup.ps1 -Recreate       # delete .venv and build it again
#>
param(
    [string]$Python = "",
    [switch]$Recreate
)

Set-Location -Path $PSScriptRoot
$Supported = @("3.12", "3.13", "3.11", "3.10", "3.14")   # in order of preference
$VenvPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"

function Fail([string]$Message) {
    Write-Host "`nERROR: $Message" -ForegroundColor Red
    exit 1
}

function Get-PythonVersion([string]$Exe, [string[]]$Prefix) {
    $out = & $Exe @Prefix -c "import sys; print('%d.%d' % sys.version_info[:2])" 2>$null
    if ($LASTEXITCODE -eq 0) { return "$out".Trim() }
    return $null
}

# --- 1. Find a supported Python --------------------------------------------------
$Wanted = if ($Python) { @($Python) } else { $Supported }
$PyExe = $null
$PyPrefix = @()
if (Get-Command py -ErrorAction SilentlyContinue) {
    foreach ($v in $Wanted) {
        if ((Get-PythonVersion "py" @("-$v")) -eq $v) { $PyExe = "py"; $PyPrefix = @("-$v"); break }
    }
}
if (-not $PyExe -and (Get-Command python -ErrorAction SilentlyContinue)) {
    $v = Get-PythonVersion "python" @()
    if ($v -and ($Wanted -contains $v)) { $PyExe = "python" }
}
if (-not $PyExe) {
    Fail ("No supported Python found (need one of: $($Wanted -join ', ')).`n" +
          "Install Python 3.12 from https://www.python.org/downloads/windows/ (keep 'py launcher' checked) and run this again.")
}
$Version = Get-PythonVersion $PyExe $PyPrefix
Write-Host "Using Python $Version ($PyExe $PyPrefix)" -ForegroundColor Cyan

# --- 2. Create (or reuse) .venv ---------------------------------------------------------
if ($Recreate -and (Test-Path ".venv")) {
    Write-Host "Removing the existing .venv ..."
    Remove-Item -Recurse -Force ".venv"
}
if ((Test-Path ".venv") -and -not (Test-Path $VenvPython)) {
    Fail "The existing .venv was not created on Windows (e.g. it comes from Linux). Run: .\setup.ps1 -Recreate"
}
if (Test-Path $VenvPython) {
    $venvVersion = Get-PythonVersion $VenvPython @()
    if (-not ($Supported -contains $venvVersion)) {
        Fail "The existing .venv uses Python $venvVersion, which is not supported. Run: .\setup.ps1 -Recreate"
    }
    Write-Host "Reusing .venv (Python $venvVersion)"
} else {
    Write-Host "Creating .venv ..."
    & $PyExe @PyPrefix -m venv .venv
    if ($LASTEXITCODE -ne 0) { Fail "Could not create the virtual environment." }
}

# --- 3. Install the requirements ----------------------------------------------------------
Write-Host "Installing requirements (this can take a few minutes the first time) ..."
& $VenvPython -m pip install --upgrade pip --disable-pip-version-check -q
if ($LASTEXITCODE -ne 0) { Fail "Upgrading pip failed (check the internet connection / proxy)." }
& $VenvPython -m pip install -r requirements.txt --disable-pip-version-check
if ($LASTEXITCODE -ne 0) { Fail "Installing requirements.txt failed." }

# --- 4. Check that the framework and the panel import -------------------------------------
& $VenvPython -c "import core.ur_control, webui.robot, webui.server; print('Import check: OK')"
if ($LASTEXITCODE -ne 0) { Fail "The packages installed, but UR_CONTROL does not import (see the error above)." }

Write-Host ""
Write-Host "Setup complete." -ForegroundColor Green
Write-Host "  Web panel:     run_webui.bat   (or  .venv\Scripts\python -m webui)"
Write-Host "  Activate venv: .venv\Scripts\Activate.ps1   (cmd: .venv\Scripts\activate.bat)"
