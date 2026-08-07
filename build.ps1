# foc_lib PC 构建 + 测试 一键脚本（Windows PowerShell）
# 优先用 CMake；无 CMake 时退化为直接 gcc 编译（需 mingw-w64 或 MSYS2 的 gcc）
$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
Push-Location $root
try {
    $cmake = Get-Command cmake -ErrorAction SilentlyContinue
    if ($cmake) {
        Write-Host "[1/3] CMake 配置..."
        & cmake -S . -B build -G "Ninja" 2>$null
        if ($LASTEXITCODE -ne 0) {
            Write-Host "     (Ninja 不可用，回退默认生成器)"
            & cmake -S . -B build
        }
        Write-Host "[2/3] 构建..."
        & cmake --build build
        Write-Host "[3/3] 运行测试..."
        & ctest --test-dir build --output-on-failure
        exit $LASTEXITCODE
    }

    $gcc = Get-Command gcc -ErrorAction SilentlyContinue
    if (-not $gcc) {
        Write-Host "未找到 cmake / gcc。请安装："
        Write-Host "  - CMake (https://cmake.org) 或"
        Write-Host "  - mingw-w64 gcc (https://winlibs.com) 或"
        Write-Host "  - 使用 STM32CubeIDE 内 arm-none-eabi-gcc 做交叉编译"
        exit 1
    }

    Write-Host "[1/2] gcc 编译..."
    New-Item -ItemType Directory -Force -Path build | Out-Null
    $srcs = @("src/foc/foc_types.c", "src/foc/config.c", "src/foc/foc_math.c", "src/foc/pid.c")
    $tests = @("test_foc_types", "test_config", "test_foc_math", "test_pid")
    foreach ($t in $tests) {
        & gcc -std=c11 -Wall -Wextra -Iinclude `
            "tests/algorithm/$t.c" $srcs -lm -o "build/$t.exe"
        if ($LASTEXITCODE -ne 0) { Write-Host "编译失败: $t"; exit 1 }
    }
    Write-Host "[2/2] 运行测试..."
    foreach ($t in $tests) {
        & "build/$t.exe"
        if ($LASTEXITCODE -ne 0) { Write-Host "测试失败: $t"; exit 1 }
    }
    Write-Host "全部测试通过 ✔"
}
finally {
    Pop-Location
}
