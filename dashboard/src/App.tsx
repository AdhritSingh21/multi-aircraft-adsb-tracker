import AirPicture from './components/AirPicture'
import MetricsBar from './components/MetricsBar'
import TrackTable from './components/TrackTable'
import { EMPTY_SNAPSHOT } from './types'
import { useTrackStream } from './useTrackStream'

export default function App() {
  const { snapshot, connected, updateRateHz } = useTrackStream()
  const snap = snapshot ?? EMPTY_SNAPSHOT
  return (
    <div className="app">
      <header>
        <h1>ADS-B Multi-Aircraft Tracker</h1>
        <span className="sub">
          C++ Kalman tracking core · live air picture
        </span>
      </header>
      <MetricsBar
        stats={snap.stats}
        updateRateHz={updateRateHz}
        connected={connected}
      />
      <main>
        <section className="panel map-panel">
          <AirPicture tracks={snap.tracks} />
        </section>
        <section className="panel table-panel">
          <TrackTable tracks={snap.tracks} />
        </section>
      </main>
    </div>
  )
}
