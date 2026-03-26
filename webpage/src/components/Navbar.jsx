import { NavLink } from 'react-router-dom'

const links = [
  { to: '/proposal', label: 'Proposal' },
  { to: '/milestone', label: 'Milestone' },
  { to: '/final', label: 'Final Report' },
]

export default function Navbar() {
  return (
    <nav style={{ borderBottom: '1px solid #e5e5e5' }} className="sticky top-0 z-50 bg-white">
      <div className="max-w-3xl mx-auto px-6 h-11 flex items-center gap-6">
        <span className="text-xs text-gray-400 mr-2 hidden sm:block">15-618 · Spring 2026</span>
        {links.map(({ to, label }) => (
          <NavLink
            key={to}
            to={to}
            className={({ isActive }) =>
              `text-sm transition-colors ${isActive ? 'text-gray-900 font-semibold' : 'text-gray-400 hover:text-gray-700'}`
            }
          >
            {label}
          </NavLink>
        ))}
      </div>
    </nav>
  )
}
