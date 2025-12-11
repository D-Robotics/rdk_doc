---
sidebar_position: 7
---

# 1.7 下载资源汇总

本文汇总了 D-Robotics RDK 套件的相关下载资源，旨在为用户提供便捷、全面的资源获取渠道。涵盖了 RDK X3（旭日 X3 派）、RDK X3 Module（旭日 X3 模组）、RDK X5、RDK X5 Module 以及 RDK Ultra 等系列产品的各类相关资源下载目录。

:::tip 快速导航
- 📦 [系统与手册资源](#系统与手册资源汇总)
- 🔧 [硬件设计资料](#规格书原理图与设计资料汇总)
- 🛠️ [开发工具资源](#开发资源与工具)
- 📋 [认证资料](#认证资料)
:::

---


## 系统与手册资源汇总

:::info 版本说明
- **系统镜像 < 3.0.0**: 基于 Ubuntu 20.04
- **系统镜像 >= 3.0.0**: 基于 Ubuntu 22.04
:::

### 🖥️ Ubuntu 系统资源

:::tip Ubuntu 系统说明
**适用用户：** 熟悉树莓派开发板使用，基于RDK X3/X5开发板，快速上手体验，聚焦应用层开发的用户。

**系统特点：**
1. Ubuntu 22.04系统，支持图形桌面操作
2. 新增功能组件支持在线deb安装，省去源码交叉编译过程
3. 提供Python接口，简化图像处理、AI推理开发难度
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 产品类别</th>
      <th>RDK X5(Module)</th>
      <th>RDK X3(Module)</th>
      <th>RDK Ultra</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>🖥️ **系统镜像**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_x5/">RDK X5 镜像</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_x3/">RDK X3 镜像</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_ultra/">RDK Ultra 镜像</a></td>
      <td>操作系统镜像文件，包含完整的软件环境</td>
    </tr>
    <tr>
      <td>🚀 **MiniBoot**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/">RDK X5 MiniBoot</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/miniboot/">RDK X3 MiniBoot</a></td>
      <td>-</td>
      <td>轻量级启动程序，用于引导操作系统，一般不需要手动安装</td>
    </tr>
    <tr>
      <td>🔬 **OpenExplore**</td>
      <td><a href="https://developer.d-robotics.cc/forumDetail/251934919646096384">RDK X5 OpenExplore</a></td>
      <td><a href="https://developer.d-robotics.cc/forumDetail/136488103547258769">RDK X3 OpenExplore</a></td>
      <td><a href="https://developer.d-robotics.cc/forumDetail/118363912788935318">RDK Ultra OpenExplore</a></td>
      <td>包含大量转化示例和源码，搭配Docker在x86开发机上使用</td>
    </tr>
    <tr>
      <td>📚 **算法工具链手册**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/index.html">RDK X5 手册</a></td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/horizon_xj3_open_explorer_cn_doc/index.html">RDK X3 手册</a></td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/index.html">RDK Ultra 手册</a></td>
      <td>OpenExplore商业算法手册，建议优先查阅社区手册</td>
    </tr>
  </tbody>
</table>
</div>

### ⚙️ Buildroot 系统资源

:::warning Buildroot 系统说明
**适用用户：** 有一定嵌入式Linux开发经验，基于RDK X5 Module eMMC版本，进行内核定制、驱动适配、系统构建等方面的开发，对系统底层开放度和灵活性有要求的用户。

**系统特点：**
1. 基于Buildroot构建Linux发行版，不支持图形桌面操作
2. 新增功能组件需要源码编译，自主定制系统镜像
3. 提供HBRE C接口，完整支持芯片图像多媒体、AI推理底层能力
:::

:::tip SDK总仓
📁 [SDK总仓](https://archive.d-robotics.cc/downloads/sdk/) - 包含所有SDK相关资料
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 资源类别</th>
      <th>RDK X5 Module</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📦 **SDK源码包**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/platform_source_code.tar.gz">RDK X5 Module SDK源码包</a></td>
      <td>完整的Buildroot源码包，支持内核定制、驱动开发、系统构建（约3.1GB）</td>
    </tr>
    <tr>
      <td>💾 **eMMC预编译镜像**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/firmwares/product_ddr_auto_detect_non-secure_release.zip">非安全版本固件</a> / <a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/firmwares/product_ddr_auto_detect_secure_release.zip">安全版本固件</a></td>
      <td>基于Buildroot构建的eMMC版本预编译系统镜像，支持DDR自动检测</td>
    </tr>
    <tr>
      <td>📚 **用户手册**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/user_manual/user_manual_v1.1.0.zip">SDK用户手册</a></td>
      <td>包含buildroot编译指南、开发环境配置、API参考等完整文档（约163MB）</td>
    </tr>
    <tr>
      <td>📋 **发布说明**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/release_note_and_change_log/LNX6.1.83_PL5.1_V1.1.0%20ReleaseNotes.pdf">ReleaseNotes</a></td>
      <td>SDK版本更新说明、功能特性、已知问题和解决方案</td>
    </tr>
    <tr>
      <td>🛠️ **烧录工具**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/software_tools/download_tools/">xburn下载工具</a></td>
      <td>支持Linux、macOS、Windows的专业烧录工具，用于固件刷写</td>
    </tr>
  </tbody>
</table>
</div>

---

## 规格书、原理图与设计资料汇总

:::tip 硬件设计总仓
📁 [Hardware](https://archive.d-robotics.cc/downloads/hardware) - 包含所有硬件设计相关资料
:::

### 🔌 核心产品硬件资料

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📋 产品类别</th>
      <th>RDK X5</th>
      <th>RDK X5 Module</th>
      <th>RDK X3</th>
      <th>RDK X3 Module</th>
      <th>RDK Ultra</th>
      <th>说明</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📄 **规格书**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDX%20X5.pdf">RDK X5 规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/drobotics_rdk_x5_module_zh_v1_2.pdf">RDK X5 Module 规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3.pdf">RDK X3 规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3_module/RDK%20X3%20MD.pdf">RDK X3 Module 规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/RDK_Ultra_Product_Brief.pdf">开发套件规格书</a> / <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/RDK_Ultra_Module_Product_Brief.pdf">Module规格书</a></td>
      <td>产品规格、特色、尺寸及型号等详细信息</td>
    </tr>
    <tr>
      <td>🔧 **原理图**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_IO_CONN_PUBLIC_V1.0.pdf">RDK X5 原理图</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_Module_Carrier_Board_V1P0_0526.pdf">官方底板原理图</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2.0_IO_Schematics.pdf">RDK X3-2.0 原理图</a></td>
      <td>暂不开放</td>
      <td>暂不开放</td>
      <td>电路设计原理图，供参考设计使用</td>
    </tr>
    <tr>
      <td>📐 **机械尺寸图**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_bottom_dxf.dxf">2D bottom</a> / <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_top_dxf.dxf">2D top</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P1_TOP.dxf">模组结构图_V1P1 (TOP)</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P1_BOTTOM.dxf">模组结构图_V1P1 (BOTTOM)</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P2_TOP.dxf">模组结构图_V1P2 (TOP)</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P2_BOTTOM.dxf">模组结构图_V1P2 (BOTTOM)</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2D_2.0.pdf">RDK X3-2.0 2D</a></td>
      <td>暂不开放</td>
      <td>暂不开放</td>
      <td>机械产品各部分尺寸大小的图纸</td>
    </tr>
    <tr>
      <td>🎯 **3D图**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_pcb.stp">RDK X5 3D</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p0_0708.stp">模组结构图_V1P0</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p1-0709.stp">模组结构图_V1P1</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module V1P2 3D.stp">模组结构图_V1P2</a><br/></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_3D_Step_Models.STEP">RDK X3-2.0 3D</a></td>
      <td>暂不开放</td>
      <td>暂不开放</td>
      <td>三维形式展示机械结构的图形</td>
    </tr>
    <tr>
      <td>🛠️ **参考设计资料**</td>
      <td>即将发布</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK%20X5%20Module%20Hardware%20Design%20Guide_V1P0_0526.pdf">硬件设计指南</a> / <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK%20X5%20Module%20Pinout%20Description%20and%20Application%20Note_V1P0_0526.xlsx">接口定义文档</a></td>
      <td>暂不开放</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3_module/reference_design">设计资料</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/reference_design">设计资料</a></td>
      <td>硬件原理图、PCB、3D模型、BOM、gerber等技术资料</td>
    </tr>
  </tbody>
</table>
</div>

### RDK 系列配件及扩展规格书

:::info 配件及扩展文档说明
RDK X3/X5 相关配件、扩展产品的完整规格书资料，提供详细的技术参数和使用说明。
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 **产品类型**</th>
      <th>🔵 **RDK X5 系列**</th>
      <th>🟠 **RDK X3 系列**</th>
      <th>📝 **说明**</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📄 **开发板规格书**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDX%20X5.pdf">RDK X5 产品规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3.pdf">RDK X3 产品规格书</a></td>
      <td>开发板详细技术规格和产品说明</td>
    </tr>
    <tr>
      <td>📄 **Module 规格书**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/drobotics_rdk_x5_module_zh_v1_2.pdf">RDK X5 Module</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3_module/RDK%20X3%20MD.pdf">RDK X3 Module</a></td>
      <td>核心计算模组详细技术规格和产品说明</td>
    </tr>
    <tr>
      <td>📷 **摄像头模组**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK%20X5%20Camera%20Module.pdf">RDK X5 摄像头模组</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3%20Camera%20Module%20RS800w.pdf">RS800w</a> / <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3%20Camera%20Module%20RS400w.pdf">RS400w</a></td>
      <td>摄像头模组技术规格和接口详细说明</td>
    </tr>
    <tr>
      <td rowspan="2">👁️ **双目摄像头模组**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK%20Stereo%20Camera%20Module.pdf">RDK 双目摄像头模组</a></td>
      <td>-</td>
      <td>用于立体视觉应用的双摄像头模组</td>
    </tr>
    <tr>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/d_robotics_rdk_stereo_camera_gs130w_zh_v1_0.pdf">RDK 双目摄像头 GS130W</a></td>
      <td> - </td>
      <td>用于机器人视觉、机器视觉检测和实时运动监测等场景</td>
    </tr>
    <tr>
      <td>🔌 **摄像头转接板**</td>
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3%20Camera%20Adapter.pdf">RDK X3 摄像头转接板</a></td>
      <td>摄像头转接板电路原理图和连接说明</td>
    </tr>
    <tr>
      <td>🔌 **PoE 扩展模组**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK%20X5%20PoE%20Module.pdf">RDK X5 PoE 模组</a></td>
      <td>-</td>
      <td>网络供电扩展模组，用于通过网线供电的应用场景</td>
    </tr>
    <tr>
      <td>🛡️ **保护外壳**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK%20X5%20Case.pdf">RDK X5 外壳</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK%20X3%20Case.pdf">RDK X3 外壳</a></td>
      <td>保护外壳尺寸和安装规格说明</td>
    </tr>
    <tr>
      <td>🔋 **电源适配器**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDX%20X5%20Power%20Adapter.pdf">RDK X5 电源适配器</a></td>
      <td>-</td>
      <td>官方电源适配器规格和电气要求</td>
    </tr>
    <tr>
      <td>📡 **IMU规格书**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK%20IMU.pdf">RDK X5 IMU规格书</a></td>
      <td>-</td>
      <td>惯性测量单元(IMU)技术规格和使用说明</td>
    </tr>
  </tbody>
</table>
</div>

---

### 📋 RDK X5 Module 详细硬件资料

:::info RDK X5 Module 专属资源
为模组客户提供完整的硬件设计支持资料，助力快速产品化开发。
:::

<div class="hardware-docs-grid">
  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📊</span>
      <h4>模组规格书</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>RDK X5 Module Datasheet</p>
      <p><strong>内容简介：</strong>提供模组的详细技术参数，包括电气特性、接口规范、尺寸定义、工作环境等关键信息。</p>
      <p><strong>适用场景：</strong>便于客户全面了解模组性能指标，进行选型评估或系统方案规划。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/drobotics_rdk_x5_module_zh_v1_2.pdf" className="download-link">📥 RDK X5 Module Datasheet</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🔧</span>
      <h4>硬件设计指南</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>RDK X5 Module Hardware Design Guide</p>
      <p><strong>内容简介：</strong>详细指导客户如何基于模组设计定制化的底板，包括供电、电源管理、接口连接、电气保护等关键设计要点。</p>
      <p><strong>适用场景：</strong>适用于底板硬件开发初期，确保设计与模组高度兼容、稳定可靠。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK%20X5%20Module%20Hardware%20Design%20Guide_V1P0_0526.pdf" className="download-link">📥 硬件设计指南</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📌</span>
      <h4>接口定义与应用笔记</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>RDK X5 Module Pinout Description and Application Note</p>
      <p><strong>内容简介：</strong>对模组的各个接口引脚进行详细定义，配合典型应用场景说明，帮助开发者快速理解信号含义与使用方式。</p>
      <p><strong>适用场景：</strong>接口设计阶段，确保引脚连接正确，便于功能调试与扩展开发。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK%20X5%20Module%20Pinout%20Description%20and%20Application%20Note_V1P0_0526.xlsx" className="download-link">📥 接口定义文档</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🎛️</span>
      <h4>官方底板参考设计</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>RDK_X5_Module_Carrier_Board</p>
      <p><strong>内容简介：</strong>提供官方底板的完整原理图，可作为客户底板设计的重要参考，涵盖各类接口、供电、功能模块布局等。</p>
      <p><strong>适用场景：</strong>参考设计使用，有助于缩短底板开发周期、减少设计错误。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_Module_Carrier_Board_V1P0_0526.pdf" className="download-link">📥 官方底板原理图</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📐</span>
      <h4>模组结构图纸</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>&nbsp;RDK_X5_MD_HW_V1P1_BOTTOM.dxf<br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RDK_X5_MD_HW_V1P1_TOP.dxf<br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RDK_X5_MD_HW_V1P2_BOTTOM.dxf<br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RDK_X5_MD_HW_V1P2_TOP.dxf<br/></p>
      <p><strong>内容简介：</strong>提供模组顶部和底部的二维结构图（DXF格式），包含孔位、外形尺寸、接口布局等关键结构信息。</p>
      <p><strong>适用场景：</strong>支持客户进行整机结构设计、模组贴装、外壳预留等工作。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P1_TOP.dxf" className="download-link">📥 TOP结构图_V1P1</a>
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P1_BOTTOM.dxf" className="download-link">📥 BOTTOM结构图_V1P1</a><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P2_TOP.dxf" className="download-link">📥 TOP结构图_V1P2</a>
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK_X5_MD_HW_V1P2_BOTTOM.dxf" className="download-link">📥 BOTTOM结构图_V1P2</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📋</span>
      <h4>模组简要规格介绍</h4>
    </div>
    <div class="doc-content">
      <p><strong>文件：</strong>drobotics_rdk_x5_md_brief_v0p5.pdf</p>
      <p><strong>内容简介：</strong>RDK X5 模组的简洁版产品介绍，涵盖主要特性、应用方向、核心优势等，便于快速了解产品定位。</p>
      <p><strong>适用场景：</strong>前期市场推广、项目提案或客户初步了解时使用。</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/drobotics_rdk_x5_md_brief_v0p5.pdf" className="download-link">📥 模组简要介绍</a>
      </div>
    </div>
  </div>
</div>

<style>
{`
.table-wrapper {
  overflow-x: auto;
  margin: 20px 0;
}

.no-wrap-table {
  width: 100%;
  min-width: 800px;
  border-collapse: collapse;
  table-layout: auto;
}

.no-wrap-table th,
.no-wrap-table td {
  white-space: nowrap;
  padding: 12px 8px;
  border: 1px solid #e1e1e1;
  text-align: center;
  vertical-align: middle;
}

.no-wrap-table th {
  background-color: #f8f9fa;
  font-weight: 600;
  color: #2c3e50;
}

.no-wrap-table td:last-child {
  white-space: normal;
  min-width: 200px;
  max-width: 300px;
}

.no-wrap-table a {
  color: #ff6900 !important;
  text-decoration: none;
}

.no-wrap-table a:hover {
  color: #e55a00 !important;
  text-decoration: underline;
}

.hardware-docs-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
  gap: 20px;
  margin: 20px 0;
}

.doc-card {
  border: 1px solid #e1e1e1;
  border-radius: 12px;
  padding: 20px;
  background: #fafafa;
  transition: all 0.3s ease;
}

.doc-card:hover {
  box-shadow: 0 4px 12px rgba(0,0,0,0.1);
  transform: translateY(-2px);
}

.doc-header {
  display: flex;
  align-items: center;
  margin-bottom: 15px;
}

.doc-icon {
  font-size: 24px;
  margin-right: 10px;
}

.doc-header h4 {
  margin: 0;
  color: #2e8b57;
  font-size: 18px;
}

.doc-content p {
  margin: 8px 0;
  font-size: 14px;
  line-height: 1.5;
}

.doc-content strong {
  color: #333;
}

.doc-link {
  margin-top: 15px;
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.download-link {
  display: inline-block;
  padding: 8px 16px;
  background: linear-gradient(45deg, #ff6900, #e55a00);
  color: white !important;
  text-decoration: none;
  border-radius: 6px;
  font-size: 13px;
  text-align: center;
  transition: all 0.3s ease;
}

.download-link:hover {
  background: linear-gradient(45deg, #e55a00, #cc4f00);
  transform: scale(1.02);
  color: white !important;
}

@media (max-width: 768px) {
  .table-wrapper {
    margin: 10px -20px;
  }

  .no-wrap-table {
    min-width: 600px;
  }

  .no-wrap-table th,
  .no-wrap-table td {
    padding: 8px 6px;
    font-size: 14px;
  }
}
`}
</style>

:::warning 注意事项
- RDK X3 Module 原理图、PCB等设计资料暂不开放
- 部分 RDK X5 Module 资料正在整理中，即将发布
:::

---

## 开发资源与工具

:::tip 推荐资源
⭐ **Model Zoo** - 提供主流算法部署例程，包含完整的训练到部署流程
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🛠️ 名称</th>
      <th>🔗 链接</th>
      <th>📝 描述</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>🤖 **TogetheROS 源码**</td>
      <td><a href="https://github.com/D-Robotics">D-Robotics 组织</a></td>
      <td>TogetheROS.Bot是D-Robotics面向机器人厂商和生态开发者推出的机器人操作系统，旨在释放机器人场景的智能潜能，助力生态开发者和商业客户能够高效、便捷的进行机器人开发，打造具有竞争力的智能机器人产品。</td>
    </tr>
    <tr>
      <td>⭐ **Model Zoo**</td>
      <td><a href="https://github.com/D-Robotics/rdk_model_zoo">Github仓库</a></td>
      <td>RDK Model Zoo 基于RDK开发, 提供大多数主流算法的部署例程. 例程包含导出D-Robotics *.bin模型, 使用 Python 等 API 推理 D-Robotics *.bin模型的流程. 部分模型还包括数据采集, 模型训练, 导出, 转化, 部署流程.</td>
    </tr>
    <tr>
      <td>📁 **TogetheRos 文件服务器**</td>
      <td><a href="https://archive.d-robotics.cc/TogetheROS/source_code">点击前往</a></td>
      <td>防止条件受限无法从github拉取代码时的备用下载源</td>
    </tr>
    <tr>
      <td>🎯 **RDK Studio**</td>
      <td><a href="https://developer.d-robotics.cc/rdkstudio">前往官网</a></td>
      <td>RDK Studio为RDK使用者提供了丰富的功能以及便利，其中包括设备管理、Demo快速入手、社区论坛快速访问等功能。板卡支持：完美支持RDK全系列板卡的使用，可以帮助用户管理手中所有的RDK板卡。系统更新：提供全新的板卡升级方式，一键获取最新RDK OS，只需要一根数据线就可以快速完成板卡升级</td>
    </tr>
    <tr>
      <td>🔄 **hbupdate**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hbupdate">点击前往</a></td>
      <td>hbupdate 是用于烧录RDK开发板操作系统镜像和最小启动镜像的专业工具</td>
    </tr>
    <tr>
      <td>💾 **镜像烧录工具**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/static/fileData/1650948745611.rar">Rufus</a></td>
      <td>用于将镜像文件写入到SD卡的软件工具，支持多种镜像格式</td>
    </tr>
    <tr>
      <td>🖥️ **SSH 远程工具**</td>
      <td><a href="https://mobaxterm.mobatek.net/download.html">MobaXterm</a> / <a href="https://www.putty.org/?hl=zh-cn">Putty</a></td>
      <td>用于远程连接和管理RDK开发套件的工具软件，提供SSH、SFTP等功能</td>
    </tr>
    <tr>
      <td>⚙️ **相关驱动**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/static/fileData/1651770860901.zip">串口驱动（CP210x）</a> / <a href="https://developer.d-robotics.cc/api/v1/static/fileData/1651120312998.rar">android_hobot</a></td>
      <td>必要的系统驱动程序，确保设备正常连接和通信</td>
    </tr>
  </tbody>
</table>
</div>

---
## 认证资料

:::tip 认证资料总仓
📁 [Certification](https://archive.d-robotics.cc/downloads/certification/) - 包含所有产品的认证文件
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🏷️ **产品名称**</th>
      <th>🇪🇺 **CE RED**</th>
      <th>🇪🇺 **CE EMC**</th>
      <th>🇺🇸 **FCC**</th>
      <th>🇯🇵 **MIC**</th>
      <th>🇨🇳 **SRRC**</th>
      <th>🇰🇷 **KCC**</th>
      <th>♻️ **RoHS**</th>
      <th>♻️ **RoHS**</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>**RDK X5**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/CE-RED210115.pdf">📄 CE RED</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/CE-EMC210115.pdf">📄 CE EMC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/D-ROBOTICS%202BGUG-RDKX5K%20FCC%20Grant%20-%20DSS.PDF">📄 FCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/D-ROBOTICS%20211-241225%20MIC%20Radio%20Certificate.pdf">📄 MIC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/SRRC.pdf">📄 SRRC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/KCC.pdf">📄 KCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/RoHS/2402Z107564E_CNAS.pdf">📄 RoHS</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/RoHS/2402Z107564E_CNAS.pdf">📄 RoHS</a></td>
    </tr>
    <tr>
      <td>**RDK X3**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/CE/C03-2402T78337E-RF%20C2%20RED%20210115.pdf">📄 CE RED</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/CE/C02-2402T78337E-RF%20C2%20EMC%20210115.pdf">📄 CE EMC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/FCC/D-ROBOTICS%202BGUG-RDKX3K%20FCC%20Grant%20-%20DTS.PDF">📄 FCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/MIC/D-ROBOTICS%20211-240607%20MIC%20Radio%20Certificate.pdf">📄 MIC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/SRRC/SRRC_Approval_RDK_X3.pdf">📄 SRRC</a></td>
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/RoHS/P-2401V86686E%20RDK%20X3.pdf">📄 RoHS</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/RoHS/P-2401V86686E%20RDK%20X3.pdf">📄 RoHS</a></td>
    </tr>
    <tr>
      <td>**RDK X3 Module**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3_MD/CE/C03-%202402T78342E-RF%20C2%20RED%20210115.pdf">📄 CE RED</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3_MD/CE/C02-%202402T78342E-RF%20C2%20EMC%20210115.pdf">📄 CE EMC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3_MD/FCC/D-ROBOTICS%202BGUG-RDKX3M%20FCC%20Grant%20-%20DTS.PDF">📄 FCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3_MD/MIC/D-ROBOTICS%20211-240608%20MIC%20Radio%20Certificate.pdf">📄 MIC</a></td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
      <td>-</td>
    </tr>
  </tbody>
</table>
</div>

---
## 附录：历史版本资料

### 📦 RDK X3 1.0版本资料

:::note 历史版本说明
以下为 RDK X3 1.0 版本的相关资料，仅供参考使用。
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🏷️ **资料类型**</th>
      <th>📄 **规格书**</th>
      <th>🔧 **原理图**</th>
      <th>📐 **机械尺寸图**</th>
      <th>🎯 **3D图**</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>**RDK X3 1.0**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_1.0_Product_Brief.pdf">📋 规格书</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_IO_Schematics.pdf">🔧 原理图</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2D_1.0.pdf">📐 2D图纸</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_3D_Step_Models_1.0.STEP">🎯 3D模型</a></td>
    </tr>
  </tbody>
</table>
</div>

---

## 注意事项

### ⚠️ 1. 版本兼容性

:::warning 重要提醒
烧录前请确认硬件型号与镜像匹配，避免兼容性问题。
:::

- **RDK X3 派 1.0 版本**：需下载 1.x.x 目录镜像
- **RDK X3 Module**：需搭配树莓派 CM4 载板使用

### 🔥 2. 烧录说明

:::info 烧录建议
- **RDK Module 系列**：首次使用建议通过 [烧录工具](https://archive.d-robotics.cc/downloads/hbupdate/) 升级系统
- **系统版本查询**：使用命令 `cat /etc/version` 或 `rdkos_info`
:::

### 🔓 3. 开源资源

:::tip 开源支持
系统源码托管于 [D-Robotics GitHub](https://github.com/d-robotics)，欢迎贡献代码！
:::

---

## 📞 技术支持

如果您在使用过程中遇到问题，可以通过以下方式获取帮助：

- 🌐 [开发者社区](https://developer.d-robotics.cc/)
- 📧 技术支持邮箱
- 📱 官方技术交流群
