#!/bin/bash
# Kill the teleop tmux session on this machine.

SESSION="teleop"
if tmux has-session -t $SESSION 2>/dev/null; then
    tmux kill-session -t $SESSION
    echo "Killed tmux session: $SESSION"
else
    echo "No active session '$SESSION'"
fi
