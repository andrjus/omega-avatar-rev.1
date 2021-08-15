SET ROBOSD_CONTENT=C:\SourceTree\robosd-5\content
SET ROBOSD=%ROBOSD_CONTENT%\robosd
SET ARMA=%ROBOSD_CONTENT%\exlib\armadilio

SET ROBOSDPP_CONTENT=C:\SourceTree\robosd-pp\content
SET ROBOSDPP=%ROBOSDPP_CONTENT%\source

SET  JSON=C:\SourceTree\jsonsl

chcp 1251 

set settings=%COMPUTERNAME%
set settings=%settings::=,%
set settings=%settings:.=,%
set settings=%CD%\settings\%settings%

SET P=%CD%

cd /D "%~dp0"

if not EXIST reference (
	MD   reference 
)


IF EXIST ".\reference\robosd-ref"  RMDIR ".\reference\robosd-ref"
MKLINK /J ".\reference\robosd-ref"  "%ROBOSD%"  


IF EXIST ".\reference\robosdpp-ref"  RMDIR ".\reference\robosdpp-ref"
MKLINK /J ".\reference\robosdpp-ref"  "%ROBOSDPP%"  

IF EXIST ".\reference\json-ref"  RMDIR ".\reference\json-ref"
MKLINK /J ".\reference\json-ref"  "%JSON%"  





if not EXIST settings (
	MD   settings 
)


if not EXIST "%settings%" (
	MD   "%settings%" 
)


IF EXIST ".\reference\settings-ref"  RMDIR ".\reference\settings-ref"
MKLINK /J ".\reference\settings-ref"  "%settings%"  

IF EXIST ".\reference\inlib-ref"  RMDIR ".\reference\inlib-ref"
MKLINK /J ".\reference\inlib-ref"  "%INILIB%"


SET LP=%CD%\%IM%

IF EXIST "%LP%\settings"  RMDIR "%LP%\settings"
MKLINK /J "%LP%\settings"  "%settings%"  

CD %P%