// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import { themes as prismThemes } from "prism-react-renderer";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "RDK DOC",
  // tagline: 'Dinosaurs are cool',
  favicon: "img/logo.png",
  // Set the production url of your site here
  url: "https://developer.d-robotics.cc",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/rdk_doc/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "D-Robotics", // Usually your GitHub org/user name.
  projectName: "rdk_doc", // Usually your repo name.

  // onBrokenLinks: 'throw',

  //add by xgs for build reduce bug
  onBrokenLinks: "warn", // 或 'ignore'
  onBrokenMarkdownLinks: "warn",

  //add vy xgs for analysis
  scripts: [
    {
      src: "https://hm.baidu.com/hm.js?24dd63cad43b63889ea6bede5fd1ab9e",
      async: true,
    },
  ],

  // add by xgs for translate
  i18n: {
    defaultLocale: "zh-Hans",
    locales: ["zh-Hans", "en"],
    localeConfigs: {
      en: {
        label: "EN",
      },
      "zh-Hans": {
        label: "CN",
      },
    },
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          routeBasePath: "/", // 修改默认文档路径
          sidebarPath: "./sidebars.js",
          showLastUpdateTime: true,
        },
        blog: { showReadingTime: true },
        pages: { exclude: ["/imager/**", "**/dl/**"] },
        theme: { customCss: "./src/css/custom.css" },
        sitemap: { lastmod: "date" },
      }),
    ],
  ],
  // add by xgs for S100_doc 2025 年 4 月 21 日 16:34:51
  plugins: [
    [
      "@docusaurus/plugin-content-docs",
      {
        id: "docs_s",
        path: "docs_s",
        routeBasePath: "rdk_s",
        sidebarPath: "./sidebars.js",
        showLastUpdateTime: true,
      },
    ],
  ],

  markdown: {
    mermaid: true,
  },

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: "img/docusaurus-social-card.jpg",
      
                // ✅ 新增：支持 h2 ~ h5 add by xgs for table of contents
    tableOfContents: {
      minHeadingLevel: 2,
      maxHeadingLevel: 5,
    },
      navbar: {
        title: "D-Robotics",
        logo: {
          alt: "地瓜机器人社区 logo",
          src: "img/logo.png",
          href: "https://d-robotics.cc/", // 修改为文档根路径
        },
        items: [
          {
            type: "docSidebar",
            sidebarId: "tutorialSidebar",
            position: "left",
            label: "RDK X3 / X5",
          },
          // add by xgs for S100_doc 2025 年 4 月 21 日 16:34:51 新增S100_doc npm install 去新增插件
          // {
          //   to: '/docs_s/',  // 与routeBasePath保持一致
          //   label: 'RDK S Series',
          //   position: 'left',
          //   // activeBaseRegex: '/docs_s/',
          // },
          {
            type: "docSidebar",
            sidebarId: "tutorialSidebar",
            docsPluginId: "docs_s",
            position: "left",
            label: "RDK S100",
          },

          {
            href: "https://developer.d-robotics.cc/",
            label: "Community",
            position: "left",
          },

          {
            href: "https://github.com/D-Robotics",
            label: "GitHub",
            position: "right",
          },
          // add by xgs for translate show
          {
            type: "localeDropdown",
            position: "right",
          },
        ],
      },
      footer: {
        style: "dark",
        links: [
          {
            title: "友情链接",
            items: [
              {
                label: "古月居",
                href: "https://www.guyuehome.com/",
              },
            ],
          },
          {
            title: "联系我们",
            items: [
              {
                label: "GitHub",
                href: "https://github.com/D-Robotics",
              },
              {
                label: "BiLiBiLi",
                href: (() => {
                  if (process.env.DOCUSAURUS_CURRENT_LOCALE === "en") {
                    return "https://www.youtube.com/@D-Robotics";
                  }
                  return "https://space.bilibili.com/437998606";
                })(),
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
        // Performance optimizations
        hashed: true, // Enable long-term caching for better performance
        language: ["en", "zh"], // Chinese and English support
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
        docsRouteBasePath: ["/", "rdk_s"],
        
        // Optimize index size and loading speed
        indexDocs: true,
        indexBlog: false, // Disable blog indexing to reduce index size
        indexPages: false, // Disable pages indexing to reduce index size
        
        // Search behavior optimizations
        searchResultContextMaxLength: 50, // Reduce context length for smaller index
      },
    ],
    "@docusaurus/theme-mermaid",
  ],
};

export default config;
