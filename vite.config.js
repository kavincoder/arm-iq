import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Deploy base: GitHub Pages hosts at kavincoder.github.io/arm-iq/
export default defineConfig({
  plugins: [react()],
  base: '/arm-iq/',
  test: {
    // Vitest config — uses jsdom so DOM APIs are available in math tests
    environment: 'jsdom',
    globals: true,
  },
})
