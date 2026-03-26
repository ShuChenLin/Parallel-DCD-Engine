import { Routes, Route, Navigate } from 'react-router-dom'
import Navbar from './components/Navbar'
import Proposal from './pages/Proposal'
import Milestone from './pages/Milestone'
import Final from './pages/Final'

export default function App() {
  return (
    <div className="min-h-screen flex flex-col">
      <Navbar />
      <main className="flex-1">
        <Routes>
          <Route path="/" element={<Navigate to="/proposal" replace />} />
          <Route path="/proposal" element={<Proposal />} />
          <Route path="/milestone" element={<Milestone />} />
          <Route path="/final" element={<Final />} />
        </Routes>
      </main>
      <footer className="max-w-3xl mx-auto px-6 py-8 w-full">
        <hr />
        <p style={{ fontSize: '.8rem', color: '#aaa' }}>
          CMU 15-618 · Parallel Computer Architecture &amp; Programming · Spring 2026
        </p>
      </footer>
    </div>
  )
}
