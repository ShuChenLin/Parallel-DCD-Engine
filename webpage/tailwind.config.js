/** @type {import('tailwindcss').Config} */
export default {
  content: ['./index.html', './src/**/*.{js,jsx}'],
  theme: {
    extend: {
      colors: {
        cream: {
          50:  '#fdfcf9',
          100: '#f9f8f4',
          200: '#f3f1eb',
          300: '#e8e4db',
          400: '#d4cfc4',
        },
        sand: '#8b8680',
        accent: {
          DEFAULT: '#c96b47',
          light:   '#e8987a',
          subtle:  '#f5ece7',
        },
      },
      fontFamily: {
        mono: ['"JetBrains Mono"', '"Fira Code"', 'monospace'],
      },
    },
  },
  plugins: [],
}
