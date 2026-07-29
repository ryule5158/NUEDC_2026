$ErrorActionPreference = "Stop"

# 当前电脑缺少MSVC C运行库头文件，故执行与Contest_LMSProcess同公式的数值回归。
$numericTest = Join-Path $PSScriptRoot "adaptive_e_numeric_test.py"
$python = (Get-Command python -ErrorAction Stop).Source
$env:PYTHONIOENCODING = "utf-8" # 保证中文测试结果按UTF-8输出。

& $python $numericTest
if ($LASTEXITCODE -ne 0) {
    throw "Adaptive E numerical test failed with exit code $LASTEXITCODE"
}
