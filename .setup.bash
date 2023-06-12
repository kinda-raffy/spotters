alias spot_env='source ~/spotters/spot_venv/bin/activate'

alias wspt='cd ~/spotters/workspace/'
alias sspt='wspt; source devel/setup.bash'
alias buildspt='wspt; catkin build -j6; sspt'
alias cbuildspt='wspt; catkin clean --yes; catkin build -j6; sspt'

alias rcd='roscd'
