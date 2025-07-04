const plugin = require('tailwindcss/plugin');

/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./src/**/*.{js,jsx,ts,tsx,md,mdx}', './docs/**/*.{md,mdx}'],
  darkMode: ['class', '[data-theme="dark"]'],
  theme: {
    extend: {
      fontFamily: {
        sans: ['"Inter"', ...require('tailwindcss/defaultTheme').fontFamily.sans],
        jakarta: ['"Plus Jakarta Sans"', ...require('tailwindcss/defaultTheme').fontFamily.sans],
        mono: ['"Fira Code"', ...require('tailwindcss/defaultTheme').fontFamily.mono],
      },
      borderRadius: {
        sm: '4px',
      },
      screens: {
        sm: '0px',
        lg: '997px',
      },
      colors: {
        primary: {
          50: 'rgb(var(--docs-color-primary-100) / <alpha-value>)',
          100: 'rgb(var(--docs-color-primary-100) / <alpha-value>)',
          200: 'rgb(var(--docs-color-primary-200) / <alpha-value>)',
        },
        secondary: {
          700: 'rgb(var(--docs-color-secondary-700) / <alpha-value>)',
          800: 'rgb(var(--docs-color-secondary-800) / <alpha-value>)',
          900: 'rgb(var(--docs-color-secondary-900) / <alpha-value>)',
          1000: 'rgb(var(--docs-color-secondary-1000) / <alpha-value>)',
        },
        text: {
          400: 'rgb(var(--docs-color-text-400) / <alpha-value>)',
        },
      },
    },
  },
  plugins: [
    plugin(function ({ addVariant }) {
      addVariant('homepage', '.homepage &');
    }),
  ],
  corePlugins: {
    preflight: false,
  },
};
