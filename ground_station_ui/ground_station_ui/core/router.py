"""MessageRouter: module nao can loai message nao thi tu dang ky -- khong co chuoi if/elif.

    router.subscribe('GPS_RAW_INT', gps.on_gps)
    router.subscribe('*', link_manager.on_any)        # moi message
    router.dispatch('px4', [msg1, msg2, ...])         # goi tu main thread, theo lo

Handler: handler(msg, link_name). Mot handler loi khong lam hong ca lo (log + bo qua).
"""

import logging
from collections import defaultdict

log = logging.getLogger(__name__)


class MessageRouter:
    def __init__(self):
        self._subs = defaultdict(list)

    def subscribe(self, msg_type, handler):
        self._subs[msg_type].append(handler)

    def dispatch(self, link_name, msgs):
        any_handlers = self._subs.get('*', ())
        for m in msgs:
            for h in self._subs.get(m.get_type(), ()):
                self._call(h, m, link_name)
            for h in any_handlers:
                self._call(h, m, link_name)

    @staticmethod
    def _call(handler, msg, link_name):
        try:
            handler(msg, link_name)
        except Exception:
            log.exception('Handler %s loi voi %s', getattr(handler, '__qualname__', handler),
                          msg.get_type())
