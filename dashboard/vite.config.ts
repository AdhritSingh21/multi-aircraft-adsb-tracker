import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// Dev proxy: the app only uses same-origin relative URLs; Vite forwards
// them to the FastAPI backend (Milestone 4) on port 8000, including the
// WebSocket upgrade for /ws. No CORS configuration needed on the backend.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/tracks': 'http://127.0.0.1:8000',
      '/healthz': 'http://127.0.0.1:8000',
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
    },
  },
})
