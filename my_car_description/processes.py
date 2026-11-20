"""Forward shutdown to Gazebo children hidden behind ros_gz_sim's shell wrapper."""
import os
from pathlib import Path
import signal


def stop_gazebo_children(root_pid):
    processes={}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:
            status=(entry/'status').read_text()
            parent=int(next(line.split()[1] for line in status.splitlines() if line.startswith('PPid:')))
            command=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
            processes[int(entry.name)]=(parent,command)
        except (OSError,StopIteration,ValueError):continue
    descendants={root_pid}
    while True:
        expanded=descendants|{pid for pid,(parent,_) in processes.items() if parent in descendants}
        if expanded==descendants:break
        descendants=expanded
    for pid in descendants:
        command=processes.get(pid,(None,''))[1]
        if command in ('gz sim server','gz sim gui') or (command.startswith('gz sim ') and ' -s ' in command):
            try:os.kill(pid,signal.SIGINT)
            except ProcessLookupError:pass
