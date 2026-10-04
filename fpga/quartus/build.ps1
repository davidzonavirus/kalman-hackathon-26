# Compile the Kalman filter coprocessor with Quartus Prime (Lite is enough).
# Looks for quartus_sh on PATH, then in the usual install folders.
$q = (Get-Command quartus_sh -ErrorAction SilentlyContinue).Source
if (-not $q) {
    $q = Get-ChildItem "C:\altera_lite\*\quartus\bin64\quartus_sh.exe", "C:\intelFPGA_lite\*\quartus\bin64\quartus_sh.exe" `
         -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $q) { throw "quartus_sh not found: install Quartus Prime Lite or add its bin64 folder to PATH" }
Set-Location $PSScriptRoot
python ../model/kf_isa.py --emit ../rtl
& $q --flow compile kf_cpu
Get-Content output_files/kf_cpu.fit.summary
Get-Content output_files/kf_cpu.sta.summary | Select-Object -First 30
