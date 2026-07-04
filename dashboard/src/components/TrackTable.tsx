import type { TrackJson } from '../types'

const STALE_AFTER_S = 15

export default function TrackTable({ tracks }: { tracks: TrackJson[] }) {
  const rows = [...tracks].sort((a, b) =>
    (a.icao || `T${a.id}`).localeCompare(b.icao || `T${b.id}`),
  )
  return (
    <div className="track-table-wrap">
      <table className="track-table">
        <thead>
          <tr>
            <th>TRK</th>
            <th>ICAO</th>
            <th>LAT</th>
            <th>LON</th>
            <th>ALT m</th>
            <th>SPD m/s</th>
            <th>AGE s</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((t) => (
            <tr key={t.id} className={t.age_s > STALE_AFTER_S ? 'stale' : ''}>
              <td>{t.id}</td>
              <td>{t.icao || '—'}</td>
              <td>{t.lat.toFixed(4)}</td>
              <td>{t.lon.toFixed(4)}</td>
              <td>{Math.round(t.alt_m)}</td>
              <td>{t.speed_mps.toFixed(0)}</td>
              <td>{t.age_s.toFixed(1)}</td>
            </tr>
          ))}
          {rows.length === 0 && (
            <tr>
              <td colSpan={7} className="empty">
                no active tracks
              </td>
            </tr>
          )}
        </tbody>
      </table>
    </div>
  )
}
