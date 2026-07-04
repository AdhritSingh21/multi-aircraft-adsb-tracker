import { useRef } from 'react'
import type { TrackJson } from '../types'

// SVG radar-style air picture. Positions are projected onto a local
// east/north tangent plane (equirectangular) around the first aircraft
// seen — the same projection the C++ tracking core uses (LocalFrame).
// The viewport auto-fits with expand-only bounds so it never jumps around.

const EARTH_R = 6371000
const DEG = Math.PI / 180
const RING_KM = [25, 50, 100, 150, 250]
const STALE_AFTER_S = 15

interface Bounds {
  minX: number
  maxX: number
  minY: number
  maxY: number
}

export default function AirPicture({ tracks }: { tracks: TrackJson[] }) {
  const origin = useRef<{ lat: number; lon: number } | null>(null)
  const bounds = useRef<Bounds | null>(null)

  if (origin.current === null && tracks.length > 0) {
    origin.current = { lat: tracks[0].lat, lon: tracks[0].lon }
  }
  const o = origin.current

  const project = (lat: number, lon: number): [number, number] => {
    if (!o) return [0, 0]
    const x = (lon - o.lon) * DEG * EARTH_R * Math.cos(o.lat * DEG)
    const y = -((lat - o.lat) * DEG * EARTH_R) // SVG y axis points down
    return [x, y]
  }

  if (o && tracks.length > 0) {
    let b = bounds.current ?? {
      minX: -20000,
      maxX: 20000,
      minY: -20000,
      maxY: 20000,
    }
    for (const t of tracks) {
      for (const [la, lo] of [[t.lat, t.lon] as [number, number], ...t.trail]) {
        const [x, y] = project(la, lo)
        b = {
          minX: Math.min(b.minX, x),
          maxX: Math.max(b.maxX, x),
          minY: Math.min(b.minY, y),
          maxY: Math.max(b.maxY, y),
        }
      }
    }
    bounds.current = b
  }

  const b = bounds.current ?? {
    minX: -50000,
    maxX: 50000,
    minY: -50000,
    maxY: 50000,
  }
  const pad = 0.08 * Math.max(b.maxX - b.minX, b.maxY - b.minY)
  const vx = b.minX - pad
  const vy = b.minY - pad
  const vw = b.maxX - b.minX + 2 * pad
  const vh = b.maxY - b.minY + 2 * pad
  const span = Math.max(vw, vh)
  const mk = span / 90 // marker size in viewBox (meter) units
  const fs = span / 70 // font size

  return (
    <svg
      className="air-picture"
      viewBox={`${vx} ${vy} ${vw} ${vh}`}
      preserveAspectRatio="xMidYMid meet"
    >
      <line x1={vx} x2={vx + vw} y1={0} y2={0} className="axis" />
      <line y1={vy} y2={vy + vh} x1={0} x2={0} className="axis" />
      {RING_KM.filter((km) => km * 1000 < span).map((km) => (
        <g key={km}>
          <circle cx={0} cy={0} r={km * 1000} className="ring" />
          <text
            x={0}
            y={-km * 1000}
            dy={-fs * 0.4}
            fontSize={fs * 0.8}
            textAnchor="middle"
            className="ring-label"
          >
            {km} km
          </text>
        </g>
      ))}
      {tracks.map((t) => {
        const [x, y] = project(t.lat, t.lon)
        const trailPoints = t.trail
          .map(([la, lo]) => project(la, lo).join(','))
          .join(' ')
        const cls = t.age_s > STALE_AFTER_S ? 'track stale' : 'track'
        return (
          <g key={t.id} className={cls}>
            {t.trail.length > 1 && (
              <polyline
                points={trailPoints}
                className="trail"
                strokeWidth={mk * 0.12}
              />
            )}
            <g transform={`translate(${x} ${y}) rotate(${t.heading_deg})`}>
              <polygon
                points={`0,${-mk} ${0.6 * mk},${mk} 0,${0.45 * mk} ${-0.6 * mk},${mk}`}
                className="marker"
              />
            </g>
            <text x={x + 1.2 * mk} y={y - 0.6 * mk} fontSize={fs} className="label">
              {t.icao || `T${t.id}`}
            </text>
          </g>
        )
      })}
      {tracks.length === 0 && (
        <text
          x={vx + vw / 2}
          y={vy + vh / 2}
          fontSize={fs}
          textAnchor="middle"
          className="label"
        >
          awaiting track data…
        </text>
      )}
    </svg>
  )
}
