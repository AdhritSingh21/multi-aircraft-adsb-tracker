"""FastAPI backend: exposes the C++ tracker's state over HTTP + WebSocket.

The tracker itself runs in the adsb_stream subprocess (same adsb_core
library as adsb_replay); this package feeds it measurements and serves its
JSON snapshots.
"""
