@ECHO OFF
REM Used to automatically build the plugin in Debug
PUSHD "%~dp0"
SET VisualStudioVersion=
"%VS140COMNTOOLS%/../IDE/devenv" "Physics2dPlugin.sln" /build Debug
