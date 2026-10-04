"""Doc thong bao bang giong noi (Qt TextToSpeech, speech-dispatcher tren Linux).

Giong tieng Anh (espeak tieng Viet nghe kem). Khong co engine -> im lang, app van chay.
Bat / tat: tab Cai dat (nho QSettings).
"""

import logging

from PySide6.QtCore import Property, QLocale, QObject, QSettings, Signal, Slot

from ground_station_ui.core.alerts.announcer import Announcer

log = logging.getLogger(__name__)


class VoiceAdapter(QObject):
    changed = Signal()

    def __init__(self, vehicle, parent=None):
        super().__init__(parent)
        self._settings = QSettings()
        self._enabled = self._settings.value('voice/enabled', True, type=bool)
        self.announcer = Announcer(vehicle)
        self._tts = None
        self._last = ''
        try:
            from PySide6.QtTextToSpeech import QTextToSpeech
            engines = QTextToSpeech.availableEngines()
            engine = 'speechd' if 'speechd' in engines else next((e for e in engines if e != 'mock'), None)
            if engine:
                self._tts = QTextToSpeech(engine, self)
                self._apply_locale()
        except Exception as e:      # thieu module / engine -> khong co giong noi
            log.warning('Khong dung duoc giong noi: %s', e)

    def _apply_locale(self):
        if self._tts is not None:
            self._tts.setLocale(QLocale('en_US'))
            self._tts.setRate(0.1)

    def tick(self):
        for text in self.announcer.tick():
            self.say(text)

    @Slot(str)
    def say(self, text):
        self._last = text
        self.changed.emit()
        if self._enabled and self._tts is not None:
            self._tts.enqueue(text)

    @Slot()
    def test(self):
        self.say(self.announcer._say('mode', mode='Position'))

    def _set_enabled(self, v):
        if v != self._enabled:
            self._enabled = v
            self._settings.setValue('voice/enabled', v)
            if not v and self._tts is not None:
                self._tts.stop()
            self.changed.emit()

    enabled = Property(bool, lambda self: self._enabled, _set_enabled, notify=changed)
    available = Property(bool, lambda self: self._tts is not None, constant=True)
    lastText = Property(str, lambda self: self._last, notify=changed)
