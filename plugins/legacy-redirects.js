// 旧站 /rdk_doc/ 路径跳转到新资料中心对应页面（add by ql for url redirect 2026-09-20）
//
// 为什么不用官方的 @docusaurus/plugin-client-redirects：
// 官方插件的 `redirects` 选项不区分语言，同一条 {from,to} 会应用到所有 locale、
// 指向同一个 to；而这里的旧 URL 中英文目标地址不同（新站 /rdk_s_doc/ 与 /rdk_s_doc/en/），
// 需要按 locale 写不同的目标，官方插件做不到。故用本自定义插件在 postBuild 阶段按语言生成跳转页。
const path = require('path');
const fs = require('fs-extra');

// from 是相对“各语言版本站点根”的路径（默认语言 build/、英文 build/en/，baseUrl 由部署层映射），
// 与旧站 URL 一一对应：from=/rdk_s/... → 旧站 /rdk_doc/rdk_s/...（英文 /rdk_doc/en/rdk_s/...）。
// to 按 locale 区分；某语言缺省表示该语言不生成跳转。
const REDIRECTS = [
  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/01_Quick_start/01_hardware_introduction/01_rdk_s100/01_rdk_s100_kit?v=4.0.5&p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100_camera_expansion_board',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/01_Quick_start/01_hardware_introduction/01_rdk_s100/02_rdk_s100_camera_expansion_board?v=4.0.5&p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/01_Quick_start/01_hardware_introduction/01_rdk_s100/02_rdk_s100_camera_expansion_board?v=4.0.5&p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100_mcu_port_expansion_board',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/hardware_introduction/rdk_s100/rdk_s100_mcu_port_expansion_board?v=4.0.5&p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/Quick_start/hardware_introduction/rdk_s100/rdk_s100_mcu_port_expansion_board?v=4.0.5&p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Advanced_development/toolchain_development/LLM_Toolchain',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/toolchain_development/LLM_Toolchain/rdk_s100/s100_LLM_Toolchain?v=4.0.5&p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/Advanced_development/toolchain_development/LLM_Toolchain/rdk_s100/s100_LLM_Toolchain?v=4.0.5&p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Advanced_development/toolchain_development/overview',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/toolchain_development/algorithm_toolchain?v=4.0.5&p=RDK+S100',
    },
  },
];

// 与官方 client-redirects 生成的跳转页同构（meta refresh + canonical + JS 兜底）
function renderRedirectPage(toUrl) {
  return `<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
    <meta http-equiv="refresh" content="0; url=${toUrl}">
    <link rel="canonical" href="${toUrl}" />
  </head>
  <script>
    window.location.href = '${toUrl}';
  </script>
</html>
`;
}

module.exports = function legacyRedirects() {
  return {
    name: 'legacy-redirects',
    async postBuild({ outDir }) {
      const locale = path.basename(path.resolve(outDir)) === 'en' ? 'en' : 'zh-Hans';
      for (const r of REDIRECTS) {
        const to = r.to[locale];
        if (!to) continue;
        const filePath = path.join(outDir, r.from.replace(/^\/+/, ''), 'index.html');
        // 未被迁移到 /legacy/ 的旧文档仍在原路由上时，不覆盖（保持文档可访问）
        if (fs.existsSync(filePath)) continue;
        await fs.outputFile(filePath, renderRedirectPage(to));
      }
    },
  };
};
