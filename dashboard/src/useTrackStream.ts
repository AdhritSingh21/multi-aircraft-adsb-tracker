import { useEffect, useRef, useState } from 'react'
import type { Snapshot } from './types'

export interface TrackStream {
  snapshot: Snapshot | null
  connected: boolean
  /** Snapshot arrival rate over a 10 s rolling window (client-derived). */
  updateRateHz: number
}

const RATE_WINDOW_MS = 10_000

/**
 * Single source of truth for backend state: seeds from GET /tracks, then
 * follows WS /ws (latest snapshot on connect, then every new one), with
 * capped-backoff reconnect. Override the socket URL with VITE_WS_URL if
 * the dev proxy is not in play.
 */
export function useTrackStream(): TrackStream {
  const [snapshot, setSnapshot] = useState<Snapshot | null>(null)
  const [connected, setConnected] = useState(false)
  const [updateRateHz, setUpdateRateHz] = useState(0)
  const arrivals = useRef<number[]>([])

  useEffect(() => {
    let ws: WebSocket | null = null
    let retryTimer: number | undefined
    let retryMs = 1000
    let disposed = false

    fetch('/tracks')
      .then((r) => r.json())
      .then((seed: Snapshot) => setSnapshot((prev) => prev ?? seed))
      .catch(() => {}) // backend not up yet; WS reconnect will keep trying

    const computeRate = () => {
      const now = performance.now()
      const win = arrivals.current
      while (win.length > 0 && now - win[0] > RATE_WINDOW_MS) win.shift()
      setUpdateRateHz(
        win.length >= 2 ? (win.length - 1) / ((now - win[0]) / 1000) : 0,
      )
    }

    const connect = () => {
      if (disposed) return
      const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
      const url =
        (import.meta.env.VITE_WS_URL as string | undefined) ??
        `${proto}://${window.location.host}/ws`
      ws = new WebSocket(url)
      ws.onopen = () => {
        setConnected(true)
        retryMs = 1000
      }
      ws.onmessage = (ev) => {
        try {
          const snap = JSON.parse(ev.data as string) as Snapshot
          setSnapshot(snap)
          arrivals.current.push(performance.now())
          computeRate()
        } catch {
          // malformed frame: ignore, keep the stream alive
        }
      }
      ws.onclose = () => {
        setConnected(false)
        if (!disposed) {
          retryTimer = window.setTimeout(connect, retryMs)
          retryMs = Math.min(retryMs * 2, 10_000)
        }
      }
      ws.onerror = () => ws?.close()
    }
    connect()

    // Decay the displayed rate when the stream goes quiet.
    const rateTimer = window.setInterval(computeRate, 2000)

    return () => {
      disposed = true
      if (retryTimer !== undefined) window.clearTimeout(retryTimer)
      window.clearInterval(rateTimer)
      ws?.close()
    }
  }, [])

  return { snapshot, connected, updateRateHz }
}
