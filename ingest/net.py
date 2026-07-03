"""Shared HTTP plumbing for the live-source clients."""
from __future__ import annotations

import http.client
import json
import urllib.error
import urllib.request


class IngestError(RuntimeError):
    """A fetch failed in a way the caller may want to retry."""


def get_json(url: str, timeout_s: float = 15.0) -> dict:
    """GET ``url`` and decode the JSON body, mapping transport failures to
    IngestError so the recorder can retry instead of crashing."""
    request = urllib.request.Request(
        url, headers={"User-Agent": "adsb-tracker/0.1 (portfolio project)"})
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as resp:
            return json.load(resp)
    except urllib.error.HTTPError as e:
        hint = " (rate limited; slow down polling)" if e.code == 429 else ""
        raise IngestError(f"HTTP {e.code} from {url}{hint}") from e
    except urllib.error.URLError as e:
        raise IngestError(f"unreachable {url}: {e.reason}") from e
    except json.JSONDecodeError as e:
        raise IngestError(f"invalid response from {url}: {e}") from e
    except (OSError, http.client.HTTPException) as e:
        # Covers failures after headers arrive: connection reset mid-body,
        # IncompleteRead, socket timeouts during read, etc.
        raise IngestError(f"connection failed reading {url}: {e!r}") from e
