"""FactGroup: 1 nhom du lieu (GPS, pin, link...) -- dict `values` + list `items` + co `dirty`.

Domain cap nhat o tan so cua message (co the ~50 Hz); tang Qt chi doc khi `dirty` va
toi da 5-10 lan/giay -> UI khong ve lai theo tung message.
"""


class FactGroup:
    def __init__(self, **defaults):
        self.values = dict(defaults)
        self.items = []
        self.dirty = True

    def set(self, **kv):
        for k, v in kv.items():
            if self.values.get(k) != v:
                self.values[k] = v
                self.dirty = True

    def set_items(self, items):
        if items != self.items:
            self.items = items
            self.dirty = True

    def attach(self, router):
        """Lop con dang ky cac message can nghe."""

    def tick(self, now):
        """Cap nhat theo thoi gian (tuoi heartbeat, stale...). Goi dinh ky tu UI ticker."""
