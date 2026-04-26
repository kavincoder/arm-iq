import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Deploy base: GitHub Pages hosts at kavincoder.github.io/arm-iq/
export default defineConfig({
  plugins: [react()],
  base: '/arm-iq/',
  test: {
    // Vitest config — node environment for pure math tests (no DOM needed)
    environment: 'node',
    globals: true,
  },
})
