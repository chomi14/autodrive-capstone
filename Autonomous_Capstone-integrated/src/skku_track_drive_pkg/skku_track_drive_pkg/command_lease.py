"""One live canonical command producer per host; previews need no lease."""
import fcntl
import os
from pathlib import Path


class CommandLease:
    def __init__(self, topic, owner, path=None):
        self.stream = None
        if topic.startswith(('/dry_run/', '/sensors_only/')):
            return
        path = path or Path('/tmp', f'autodrive_command_{os.getuid()}.lock')
        stream = open(path, 'a+')
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            stream.seek(0)
            current = stream.read().strip()
            stream.close()
            raise RuntimeError(f'Another live command controller owns the vehicle: {current}. Stop it before starting {owner}.')
        stream.seek(0)
        stream.truncate()
        stream.write(owner)
        stream.flush()
        self.stream = stream

    def close(self):
        if self.stream is not None:
            self.stream.close()
            self.stream = None

    def __del__(self):
        self.close()
