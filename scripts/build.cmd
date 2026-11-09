@echo off
setlocal
pushd "%~dp0.."
bash scripts/build.sh %*
set "build_exit_code=%ERRORLEVEL%"
popd
exit /b %build_exit_code%
