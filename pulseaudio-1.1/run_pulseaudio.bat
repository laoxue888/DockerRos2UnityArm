@echo off
:loop
echo PulseAudio service is running...
.\bin\pulseaudio.exe --use-pid-file=false -D
echo The PulseAudio service has exited, will restart in 1 second...
timeout /t 1
goto loop