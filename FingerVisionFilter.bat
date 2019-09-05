@echo off
pushd %~dp0
start "%~n0 %*" %~n0.py %*
popd