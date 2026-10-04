"""Cache o ban do dang MBTiles (SQLite, chuan mo -- mo duoc bang QGIS / GDAL).

Moi lop ban do 1 file .mbtiles. Toa do XYZ (nhu URL web) <-> TMS trong MBTiles: lat nguoc truc y.
Dung chung giua thread tile server va thread tai truoc -> 1 connection + khoa.

Khong de main thread (UI) phai cho khoa: stats() tra so o dem san trong bo nho (khong SELECT
COUNT), file mo che do WAL + synchronous=NORMAL -> commit khong fsync moi o (tai truoc 4 luong
khong lam giat UI).
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
            row = self._db.execute(
                'SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                (z, x, xyz_to_tms_row(z, y))).fetchone()
        return row[0] if row else None

    def has(self, z, x, y):
        with self._lock:
            return self._db.execute(
                'SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                (z, x, xyz_to_tms_row(z, y))).fetchone() is not None

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
                pass        # app dang tat: o tai xong sau khi da dong file -- bo qua

    def stats(self):
        """(so o, dung luong byte) -- KHONG khoa, goi tu main thread duoc."""
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
