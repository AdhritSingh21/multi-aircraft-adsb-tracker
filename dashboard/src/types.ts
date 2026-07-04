// Data contract with the Milestone 4 backend: GET /tracks and every
// WebSocket /ws message carry this exact shape (see cpp/src/json_out.cpp).

export interface TrackJson {
  id: number
  icao: string // may be "" in geometric association modes
  lat: number
  lon: number
  alt_m: number
  speed_mps: number
  heading_deg: number
  age_s: number
  hits: number
  trail: [number, number][] // up to 20 most recent [lat, lon] points
}

export interface SnapshotStats {
  measurements: number
  tracks_created: number
  stale_removed: number
  active: number
}

export interface Snapshot {
  time: number
  tracks: TrackJson[]
  stats: SnapshotStats
}

export const EMPTY_SNAPSHOT: Snapshot = {
  time: 0,
  tracks: [],
  stats: { measurements: 0, tracks_created: 0, stale_removed: 0, active: 0 },
}
