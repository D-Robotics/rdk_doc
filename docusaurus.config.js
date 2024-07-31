// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'RDK DOC',
  // tagline: 'Dinosaurs are cool',
  favicon: 'img/logo.png',

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'D-Robotics', // Usually your GitHub org/user name.
  projectName: 'rdk_doc', // Usually your repo name.


  // onBrokenLinks: 'throw',

  //add by xgs for build reduce bug
  onBrokenLinks: 'warn', // 或 'ignore'
  onBrokenMarkdownLinks: 'warn',

  //add vy xgs for analysis
  scripts: [
    {src: 'https://hm.baidu.com/hm.js?24dd63cad43b63889ea6bede5fd1ab9e',  async: true}
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".

  // i18n: {
  //   defaultLocale: 'en',
  //   locales: ['en'],
  // },

  
  // add by xgs for translate
  i18n: {
    defaultLocale: 'zh-Hans',
    locales: ['zh-Hans','en'],
    localeConfigs: {
      en: {
        label: 'EN',
        // direction: 'ltr',
        // path: 'i18n/en/',

      },
      'zh-Hans': {
        label: 'CN',
        // path: '/docs/intro/',

      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {

          //add by xgs for delete first page
          // routeBasePath: '/', // Serve the docs at the site's root

          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        // blog: {
        //   showReadingTime: true,
        //   // Please change this to your repo.
        //   // Remove this to remove the "edit this page" links.
        //   editUrl:
        //     'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        // },
        theme: {
          customCss: './src/css/custom.css',
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
        title: 'D-Robotics',
        logo: {
          alt: 'My Site Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'RDK',
          },
          // {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://developer.d-robotics.cc/',
            label: 'Community',
            position: 'left',
          },

          {
            href: 'https://github.com/D-Robotics',
            label: 'GitHub',
            position: 'right',
          },
// add by xgs for translate show
          {
            type: 'localeDropdown',
            position: 'right',
          },

        ],
      },
      footer: {
        style: 'dark',
        links: [
          // {
          //   title: 'Docs',
          //   items: [
          //     {
          //       label: 'RDK_DOC',
          //       to: '/docs/RDK',
          //     },
          //   ],
          // },
          {
            title: '友情链接',
            items: [
              {
                label: '古月居',
                href: 'https://www.guyuehome.com/',
              },
              {
                label: '智能驾驶社区',
                href: 'https://auto-developer.horizon.cc/developerForum/',
              },

            ],
          },
          {
            title: '联系我们',
            items: [
              {
                label: '邮箱：developer@d-robotics.cc',
                to: 'developer@d-robotics.cc',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/D-Robotics',
              },
              {
                label: 'BiLiBiLi',
                href: 'https://github.com/D-Robotics',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} D-Robotics.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
    themes: [
      // add by xgs for search.
      [
        require.resolve("@easyops-cn/docusaurus-search-local"),
        {
          // ... Your options.
          // `hashed` is recommended as long-term-cache of index file is possible.
          hashed: true,
          language: ["en", "zh"],
          highlightSearchTermsOnTargetPage: true,
          explicitSearchResultPath: true,
          docsRouteBasePath: '/'
          // For Docs using Chinese, The `language` is recommended to set to:
          // ```
          // language: ["en", "zh"],
          // ```
        },
      ],
    ],

};

export default config;
