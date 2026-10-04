# Renders analysis/out/GroundSpeed_Forensics.pptx to PNGs (analysis/out/slides_png/) with PowerPoint,
# for visual checking. Windows + PowerPoint only; the deck itself is built by 10_build_slides.py.
$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$pptx = (Resolve-Path (Join-Path $here "out\GroundSpeed_Forensics.pptx")).Path
$dest = Join-Path $here "out\slides_png"
New-Item -ItemType Directory -Force $dest | Out-Null
$app = New-Object -ComObject PowerPoint.Application
try {
    $pres = $app.Presentations.Open($pptx, $true, $false, $false)
    $i = 0
    foreach ($s in $pres.Slides) {
        $i++
        $s.Export((Join-Path $dest ("slide_{0:D2}.png" -f $i)), "PNG", 1600, 900)
    }
    $pres.Close()
    Write-Output "rendered $i slides to $dest"
} finally {
    $app.Quit()
}
