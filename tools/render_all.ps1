$env:PATH = 'C:\Compilers\msys64\mingw64\bin;' + $env:PATH
$root = 'C:\Users\lcknight\Desktop\tmp\CGAssignment1'
$exe  = "$root\build\windows-mingw-release\PathTracer.exe"
Set-Location $root

Write-Host '=== cornell-box (64 spp) ==='
& $exe "$root\example-scenes-cg25\cornell-box" 64

Write-Host '=== veach-mis (64 spp) ==='
& $exe "$root\example-scenes-cg25\veach-mis" 64

Write-Host '=== living-room (64 spp) ==='
& $exe "$root\example-scenes-cg25\living-room" 64

Write-Host '=== Image comparison ==='
python compare_images.py
