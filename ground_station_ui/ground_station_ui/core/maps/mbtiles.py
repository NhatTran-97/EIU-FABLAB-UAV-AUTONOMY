"""Map tile cache in MBTiles format (SQLite, open standard readable by QGIS / GDAL).

One .mbtiles file per map layer. XYZ coordinates (as in web URLs) <-> TMS in MBTiles: the y axis is flipped.
Shared by the tile server thread and the prefetch threads: one connection plus a lock.

The main (UI) thread never waits for the lock: stats() returns counters kept in memory (no SELECT COUNT),
and the file uses WAL + synchronous=NORMAL so commits do not fsync per tile (prefetching with 4 workers
does not stall the UI).
"""

import sqlite3
import threading
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS tiles (
    zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB,
    PRIMARY KEY (zoom_level, tile_column, tile_row));
"""


def xyz_to_tms_row(z, y):
    return (1 << z) - 1 - y


class MBTiles:
    def __init__(self, path, name='', readonly=False):
        self.path = Path(path).expanduser()
        self.readonly = readonly
        self._lock = threading.Lock()
        if readonly:
            self._db = sqlite3.connect(f'file:{self.path}?mode=ro', uri=True, check_same_thread=False)
        else:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._db = sqlite3.connect(str(self.path), check_same_thread=False)
            self._db.execute('PRAGMA journal_mode=WAL')
            self._db.execute('PRAGMA synchronous=NORMAL')
            self._db.executescript(SCHEMA)
            self._db.execute("INSERT OR IGNORE INTO metadata VALUES ('name', ?)", (name,))
            self._db.execute("INSERT OR IGNORE INTO metadata VALUES ('format', 'jpg')")
            self._db.commit()
        self._count = self._db.execute('SELECT COUNT(*) FROM tiles').fetchone()[0]

    def get(self, z, x, y):
        with self._lock:
            try:
                row = self._db.execute(
                    'SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                    (z, x, xyz_to_tms_row(z, y))).fetchone()
            except sqlite3.ProgrammingError:
                return None             # closed (app shutting down)
        return row[0] if row else None

    def has(self, z, x, y):
        with self._lock:
            try:
                return self._db.execute(
                    'SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                    (z, x, xyz_to_tms_row(z, y))).fetchone() is not None
            except sqlite3.ProgrammingError:
                return False

    def put(self, z, x, y, data):
        if self.readonly:
            return
        with self._lock:
            try:
                key = (z, x, xyz_to_tms_row(z, y))
                cur = self._db.execute('UPDATE tiles SET tile_data=? WHERE zoom_level=? AND tile_column=? '
                                       'AND tile_row=?', (sqlite3.Binary(data), *key))
                if cur.rowcount == 0:
                    self._db.execute('INSERT INTO tiles VALUES (?, ?, ?, ?)', (*key, sqlite3.Binary(data)))
                    self._count += 1
                self._db.commit()
            except sqlite3.ProgrammingError:
                pass        # app shutting down: the tile finished downloading after the file was closed; ignore

    def stats(self):
        """(tile count, bytes). Takes no lock; safe from the main thread."""
        size = 0
        for p in (self.path, self.path.with_name(self.path.name + '-wal')):
            try:
                size += p.stat().st_size
            except OSError:
                pass
        return self._count, size

    def close(self):
        with self._lock:
            self._db.close()
