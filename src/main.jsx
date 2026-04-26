/**
 * main.jsx — Application entry point
 *
 * Mounts the React app into #root.
 * Global CSS (design tokens, reset, fonts) is imported here so it
 * applies to the entire application before any component renders.
 */

import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import './styles/global.css';
import App from './App.jsx';

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <App />
  </StrictMode>,
);
