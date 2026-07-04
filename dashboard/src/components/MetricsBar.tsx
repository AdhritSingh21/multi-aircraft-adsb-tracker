import type { SnapshotStats } from '../types'

interface Props {
  stats: SnapshotStats
  updateRateHz: number
  connected: boolean
}

export default function MetricsBar({ stats, updateRateHz, connected }: Props) {
  return (
    <div className="metrics">
      <span className={connected ? 'badge live' : 'badge offline'}>
        {connected ? 'LIVE' : 'OFFLINE'}
      </span>
      <Metric label="active tracks" value={stats.active} />
      <Metric label="update rate" value={`${updateRateHz.toFixed(1)} Hz`} />
      <Metric label="stale removed" value={stats.stale_removed} />
      <Metric label="tracks created" value={stats.tracks_created} />
      <Metric label="measurements" value={stats.measurements} />
    </div>
  )
}

function Metric({ label, value }: { label: string; value: number | string }) {
  return (
    <div className="metric">
      <div className="metric-value">{value}</div>
      <div className="metric-label">{label}</div>
    </div>
  )
}
