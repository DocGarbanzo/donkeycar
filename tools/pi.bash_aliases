
alias ll='ls -ahl'
#alias la='ls -A'
#alias l='ls -CF'
alias e='emacs'
alias bye='sudo shutdown -h now'
alias cpu='vcgencmd measure_clock arm; vcgencmd get_throttled; vcgencmd measure_temp'
alias wcpu='watch "vcgencmd measure_clock arm; vcgencmd get_throttled; vcgencmd measure_temp"'
alias wifi_restart='sudo wpa_cli -i wlan0 reconfigure'

export PATH=$PATH:.
