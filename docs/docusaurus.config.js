import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Blue',
  tagline: 'A ROS 2 framework for development and deployment of underwater vehicles',
  favicon: 'img/favicon.ico',

  url: 'https://robotic-decision-making-lab.github.io',
  baseUrl: '/blue',

  organizationName: 'Robotic-Decision-Making-Lab',
  projectName: 'blue',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: true,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          editUrl: 'https://github.com/Robotic-Decision-Making-Lab/blue/tree/gh-pages/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.scss',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Blue Documentation',
        items: [
          {
            href: 'https://research.engr.oregonstate.edu/rdml/home',
            label: 'RDML',
            position: 'right',
          },
          {
            href: 'https://github.com/Robotic-Decision-Making-Lab/blue',
            position: 'right',
            className: 'header-github-link',
            'aria-label': 'GitHub repository',
          },
        ],
      },
      footer: {
        style: 'dark',
        copyright: `© ${new Date().getFullYear()} Robotic Decision Making Lab @ Oregon State University.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['bash'],
      },
    }),
  markdown: {
    mermaid: true,
  },
  plugins: ['docusaurus-plugin-sass', '@docusaurus/theme-mermaid'],
};

export default config;
