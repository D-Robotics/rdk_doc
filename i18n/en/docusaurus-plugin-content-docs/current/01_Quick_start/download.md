---
sidebar_position: 7
---

# 1.7 Download Resources Summary

This document summarizes the download resources related to the D-Robotics RDK suite, aiming to provide users with convenient and comprehensive access to resources. It covers various resource download directories for products such as RDK X3 (Sunrise X3 Pi), RDK X3 Module, RDK X5, RDK X5 Module, and RDK Ultra series.

:::tip Quick Navigation
- 📦 [System & Manual Resources](#system-and-manual-resources-summary)
- 🔧 [Hardware Design Materials](#specifications-schematics-and-design-resources-summary)
- 🛠️ [Development Tool Resources](#development-resources-and-tools)
- 📋 [Certification Materials](#certification-resources)
:::

---

## System and Manual Resources Summary

:::info Version Notes
- **System Images < 3.0.0**: Based on Ubuntu 20.04
- **System Images >= 3.0.0**: Based on Ubuntu 22.04
:::

### 🖥️ Ubuntu System Resources

:::tip Ubuntu System Description
**Target Users:** Users familiar with Raspberry Pi development boards, focusing on application layer development with quick hands-on experience using RDK X3/X5 development boards.

**System Features:**
1. Ubuntu 22.04 system with graphical desktop operation support
2. New functional components support online deb installation, eliminating source code cross-compilation
3. Provides Python interfaces to simplify image processing and AI inference development complexity
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 Product Category</th>
      <th>RDK X5(Module)</th>
      <th>RDK X3(Module)</th>
      <th>RDK Ultra</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>🖥️ **System Images**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_x5/">RDK X5 Images</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_x3/">RDK X3 Images</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/os_images/rdk_ultra/">RDK Ultra Images</a></td>
      <td>Operating system image files with complete software environment</td>
    </tr>
    <tr>
      <td>🚀 **MiniBoot**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/miniboot/rdk_x5/">RDK X5 MiniBoot</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/miniboot/">RDK X3 MiniBoot</a></td>
      <td>-</td>
      <td>Lightweight bootloader for OS booting, generally not requiring manual installation</td>
    </tr>
    <tr>
      <td>🔬 **OpenExplore**</td>
      <td>[RDK X5 OpenExplore](../07_Advanced_development/04_toolchain_development/intermediate/environment_config.md#rdk-x5)</td>
      <td>[RDK X3 OpenExplore](../07_Advanced_development/04_toolchain_development/intermediate/environment_config.md#rdk-x3)</td>
      <td>[RDK Ultra OpenExplore](../07_Advanced_development/04_toolchain_development/intermediate/environment_config.md#rdk-ultra)</td>
      <td>Contains numerous conversion examples and source code, used with Docker on x86 development machines</td>
    </tr>
    <tr>
      <td>📚 **Algorithm Toolchain Manual**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/x5_doc-v126cn/index.html">RDK X5 Manual</a></td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/horizon_xj3_open_explorer_cn_doc/index.html">RDK X3 Manual</a></td>
      <td><a href="https://developer.d-robotics.cc/api/v1/fileData/horizon_j5_open_explorer_cn_doc/index.html">RDK Ultra Manual</a></td>
      <td>OpenExplore commercial algorithm manual, prioritize community manuals for reference</td>
    </tr>
  </tbody>
</table>
</div>

### ⚙️ Buildroot System Resources

:::warning Buildroot System Description
**Target Users:** Users with embedded Linux development experience, focused on kernel customization, driver adaptation, and system building based on RDK X5 Module eMMC version, requiring system-level openness and flexibility.

**System Features:**
1. Linux distribution built on Buildroot, no graphical desktop operation support
2. New functional components require source code compilation and custom system image building
3. Provides HBRE C interface with complete support for chip image multimedia and AI inference capabilities
:::

:::tip SDK Main Repository
📁 [SDK Main Repository](https://archive.d-robotics.cc/downloads/sdk/) - Includes all SDK-related materials.
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 Resource Category</th>
      <th>RDK X5 Module</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📦 **SDK Source Package**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/platform_source_code.tar.gz">RDK X5 Module SDK Source</a></td>
      <td>Complete Buildroot source package supporting kernel customization, driver development, and system building (approx. 3.1GB)</td>
    </tr>
    <tr>
      <td>💾 **eMMC Precompiled Images**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/firmwares/product_ddr_auto_detect_non-secure_release.zip">Non-secure Firmware</a> / <a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/board_support_package/firmwares/product_ddr_auto_detect_secure_release.zip">Secure Firmware</a></td>
      <td>Precompiled system images for eMMC version built on Buildroot with DDR auto-detection support</td>
    </tr>
    <tr>
      <td>📚 **User Manual**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/user_manual/user_manual_v1.1.0.zip">SDK User Manual</a></td>
      <td>Comprehensive documentation including buildroot compilation guide, development environment setup, API reference (approx. 163MB)</td>
    </tr>
    <tr>
      <td>📋 **Release Notes**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/release_note_and_change_log/LNX6.1.83_PL5.1_V1.1.0%20ReleaseNotes.pdf">ReleaseNotes</a></td>
      <td>SDK version updates, feature descriptions, known issues and solutions</td>
    </tr>
    <tr>
      <td>🛠️ **Flashing Tool**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/sdk/LNX6.1.83_PL5.1_V1.1.0/software_tools/download_tools/">xburn Download Tools</a></td>
      <td>Professional flashing tools supporting Linux, macOS, and Windows for firmware installation</td>
    </tr>
  </tbody>
</table>
</div>

---

## Specifications, Schematics, and Design Resources Summary

:::tip Hardware Design Repository
📁 [Hardware](https://archive.d-robotics.cc/downloads/hardware) - Contains all hardware design related materials
:::

### 🔌 Core Product Hardware Materials

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📋 Product Category</th>
      <th>RDK X5</th>
      <th>RDK X5 Module</th>
      <th>RDK X3</th>
      <th>RDK X3 Module</th>
      <th>RDK Ultra</th>
      <th>Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📄 **Specifications**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/d_robotics_rdk_x5_en_v1_1.pdf">RDK X5 Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5_module/drobotics_rdk_x5_module_en_v1_2.pdf">RDK X5 Module Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3.pdf">RDK X3 Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3_module/RDK%20X3%20MD.pdf">RDK X3 Module Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/RDK_Ultra_Product_Brief.pdf">Development Kit Specifications</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/RDK_Ultra_Module_Product_Brief.pdf">Module Specifications</a></td>
      <td>Product specifications, features, dimensions, and model details</td>
    </tr>
    <tr>
      <td>🔧 **Schematics**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_IO_CONN_PUBLIC_V1.1.pdf">RDK X5 Schematics</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Carrier Board V1P1.pdf">Official Carrier Board Schematics</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2.0_IO_Schematics.pdf">RDK X3-2.0 Schematics</a></td>
      <td>N/A</td>
      <td>Not Currently Available</td>
      <td>Circuit design schematics for reference design use</td>
    </tr>
    <tr>
      <td>📐 **2D structural drawings**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_top_dxf.dxf">2D top</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_bottom_dxf.dxf">2D bottom</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p1_dxf.zip">Module Structure_V1P1 </a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p2_dxf.zip">Module Structure_V1P2 </a><br/></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2D_2.0.pdf">RDK X3-2.0 2D</a></td>
      <td>Not Currently Available</td>
      <td>Not Currently Available</td>
      <td>2D drawings</td>
    </tr>
    <tr>
      <td>🎯 **3D structural drawings**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/RDK_X5_LPDDR4_4266MHz_V1P0_pcb.stp">RDK X5 3D</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p0_0708.stp">Module Structure_V1P0</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p1-0709.stp">Module Structure_V1P1</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module V1P2 3D.stp">Module Structure_V1P2</a><br/></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_3D_Step_Models.STEP">RDK X3-2.0 3D</a></td>
      <td>Not Currently Available</td>
      <td>Not Currently Available</td>
      <td>3D models</td>
    </tr>
    <tr>
      <td>🛠️ **Reference Design Materials**</td>
      <td>N/A</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Hardware Design Guide V1P1.pdf">Hardware Design Guide</a><br/><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Pinout Description and Application Note V1P1.xlsx">Interface Definition Document</a></td>
      <td>N/A</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3_module/reference_design">Design Materials</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_ultra/reference_design">Design Materials</a></td>
      <td>Hardware schematics, PCB, 3D models, BOM, gerber, and other technical resources</td>
    </tr>
  </tbody>
</table>
</div>

### RDK Series Product & Expansion Specifications

:::info RDK X3/X5 Product & Expansion Documentation
Complete specification materials for RDK X3/X5 related products, accessories and expansion modules, providing detailed technical parameters and usage instructions.
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>📦 **Product Type**</th>
      <th>🔵 **RDK X5 Series**</th>
      <th>🟠 **RDK X3 Series**</th>
      <th>📝 **Description**</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>📄 **Development Board Specifications**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/d_robotics_rdk_x5_en_v1_1.pdf">RDK X5 Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3.pdf">RDK X3 Specifications</a></td>
      <td>Development board detailed technical specifications and product description</td>
    </tr>
    <tr>
      <td>📄 **Module Specifications**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5_module/drobotics_rdk_x5_module_en_v1_2.pdf">RDK X5 Module</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3_module/RDK%20X3%20MD.pdf">RDK X3 Module</a></td>
      <td>Core computing module detailed technical specifications and product description</td>
    </tr>
    <tr>
      <td>📷 **Camera Module**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/RDK%20X5%20Camera%20Module.pdf">RDK X5 Camera Module</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3%20Camera%20Module%20RS800w.pdf">RS800w</a> / <a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3%20Camera%20Module%20RS400w.pdf">RS400w</a></td>
      <td>Camera module technical specifications and interface details</td>
    </tr>
    <tr>
      <td rowspan="3">👁️ **Stereo Camera Module**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/RDK%20Stereo%20Camera%20Module.pdf">RDK Stereo Camera Module</a></td>
      <td>-</td>
      <td>Dual camera module for stereo vision applications</td>
    </tr>
    <tr>
      <td>
      <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/d_robotics_rdk_stereo_camera_gs130w_en_v1_0.pdf">RDK Stereo Camera  
       GS130W</a>
      </td>
      <td>-</td>
      <td>
      Used in scenarios such as robotic vision, machine vision inspection, and real-time motion monitoring.
      </td>
    </tr>
    <tr>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5/d_robotics_rdk_RDK_stereo_camera_gs130wi_en_v1_1.pdf">RDK Stereo Camera  
       GS130WI</a></td>
      <td> - </td>
      <td>Used in applications such as robot vision, machine vision inspection, and motion posture sensing.</td>
    </tr>
    <tr>
      <td>🔌 **Camera Adapter**</td>
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3%20Camera%20Adapter.pdf">RDK X3 Camera Adapter</a></td>
      <td>Camera adapter board circuit schematics and connection instructions</td>
    </tr>
    <tr>
      <td>🔌 **PoE Expansion Module**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/RDK%20X5%20PoE%20Module.pdf">RDK X5 PoE Module</a></td>
      <td>-</td>
      <td>Power over Ethernet expansion module for network-powered applications</td>
    </tr>
    <tr>
      <td>🛡️ **Protective Case**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/RDK%20X5%20Case.pdf">RDK X5 Case</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3/RDK%20X3%20Case.pdf">RDK X3 Case</a></td>
      <td>Protective case dimensions and installation specifications</td>
    </tr>
    <tr>
      <td>🔋 **Power Adapter**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5/RDX%20X5%20Power%20Adapter.pdf">RDK X5 Power Adapter</a></td>
      <td>-</td>
      <td>Official power adapter specifications and electrical requirements</td>
    </tr>
    <tr>
      <td>📄 **Module Specifications**</td>
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x3_module/RDK%20X3%20MD.pdf">RDK X3 Module</a></td>
      <td>Module detailed technical specifications and product description</td>
    </tr>
  </tbody>
</table>
</div>

---

### 📋 RDK X5 Module Detailed Hardware Resources

:::info RDK X5 Module Exclusive Resources
Providing comprehensive hardware design support materials for module customers to accelerate product development.
:::

<div class="hardware-docs-grid">
  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📊</span>
      <h4>Module Specifications</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>RDK X5 Module Datasheet</p>
      <p><strong>Content:</strong>Provides detailed technical parameters including electrical characteristics, interface specifications, dimensional definitions, and operating environment.</p>
      <p><strong>Use Case:</strong>Helps customers comprehensively understand module performance metrics for selection evaluation or system planning.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5_module/drobotics_rdk_x5_module_en_v1_2.pdf" className="download-link">📥 RDK X5 Module Datasheet</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🔧</span>
      <h4>Hardware Design Guide</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>RDK X5 Module Hardware Design Guide V1P1</p>
      <p><strong>Content:</strong>Detailed guidance on designing custom carrier boards, including power supply, power management, interface connections, and electrical protection.</p>
      <p><strong>Use Case:</strong>Applicable during initial carrier board hardware development to ensure design compatibility and reliability.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Hardware Design Guide V1P1.pdf" className="download-link">📥 Hardware Design Guide</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📌</span>
      <h4>Interface Definition & Application Notes</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>RDK X5 Module Pinout Description and Application Note V1P1</p>
      <p><strong>Content:</strong>Detailed definition of module interface pins with typical application scenarios to help developers understand signal meanings and usage.</p>
      <p><strong>Use Case:</strong>For interface design phase, ensuring correct pin connections and facilitating debugging.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Pinout Description and Application Note V1P1.xlsx" className="download-link">📥 Interface Definition Document</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🎛️</span>
      <h4>Official Carrier Board Reference Design</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>RDK X5 Module Carrier Board V1P1</p>
      <p><strong>Content:</strong>Complete schematics of the official carrier board, serving as an important reference for customer designs.</p>
      <p><strong>Use Case:</strong>Reference design usage to shorten development cycles and reduce design errors.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Carrier Board V1P1.pdf" className="download-link">📥 Official Carrier Board Schematics</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📐</span>
      <h4>Module Structural Drawings</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>&nbsp;rdk_x5_md_hw_v1p1<br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; rdk_x5_md_hw_v1p2<br/>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</p>
      <p><strong>Content:</strong>2D structural drawings (DXF format) of module top and bottom, including hole positions, dimensions, and interface layout.</p>
      <p><strong>Use Case:</strong>Supports mechanical design, module mounting, and enclosure design.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p1_dxf.zip" className="download-link">📥 Drawing_V1P1</a>
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/rdk_x5_md_hw_v1p2_dxf.zip" className="download-link">📥 Drawing_V1P2</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🔋</span>
      <h4>Module Typical Scenario Power Consumption Introduction</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong>RDK X5 Module Typical Scenario Power Consumption V1P0</p>
      <p><strong>Content:</strong>RDK X5 Module typical scenario power consumption.</p>
      <p><strong>Use Case:</strong>Used to illustrate the typical power consumption of the module, helping customers complete power consumption assessment during the whole machine design phase.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/en/hardware/rdk_x5_module/RDK X5 Module Typical Scenario Power Consumption V1P0-EN.xlsx" className="download-link">📥 Module Typical Scenario Power Consumption Introduction</a>
      </div>
    </div>
  </div>

<div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">🌡️</span>
      <h4>Thermal Design Guide</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong> RDK X5 Module Thermal Design Guide V1P0</p>
      <p><strong>Content:</strong> This document provides thermal design guidelines for the RDK X5 Module, offering thermal simulation support and heat dissipation design recommendations.</p>
      <p><strong>Use Case:</strong> The RDK X5 Module Thermal Design Guide assists users in developing reliable system-level thermal solutions.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Thermal Design Guide V1P0.pdf" className="download-link">📥 Thermal Design Guide</a>
      </div>
    </div>
  </div>

  <div class="doc-card">
    <div class="doc-header">
      <span class="doc-icon">📝</span>
      <h4>Hardware Design Checklist</h4>
    </div>
    <div class="doc-content">
      <p><strong>File:</strong> RDK X5 Module Hardware Design Checklist V1P0</p>
      <p><strong>Content:</strong> This document serves as a guide for customers to verify hardware design, ensuring compliance and completeness to enhance design success rates.</p>
      <p><strong>Use Case:</strong> Serves as key guidance prior to hardware design to ensure functional reliability and manufacturing feasibility.</p>
      <div class="doc-link">
        <a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x5_module/RDK X5 Module Hardware Design Checklist V1P0.xlsx" className="download-link">📥 Hardware Design Checklist</a>
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

:::warning Notes
- RDK X3 Module schematics, PCB, and design materials are not currently available.
- Some RDK X5 Module materials are being prepared and will be released soon.
:::

---

## Development Resources and Tools

:::tip Recommended Resources
⭐ **Model Zoo** - Provides deployment routines for mainstream algorithms, including complete training-to-deployment workflows.
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🛠️ Name</th>
      <th>🔗 Link</th>
      <th>📝 Description</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>🤖 **TogetheROS Source Code**</td>
      <td><a href="https://github.com/D-Robotics">D-Robotics Organization</a></td>
      <td>TogetheROS.Bot is a robot operating system launched by D-Robotics for robot manufacturers and ecosystem developers. It aims to unleash the intelligent potential of robot scenarios and help ecosystem developers and commercial customers efficiently and conveniently develop robots, creating competitive intelligent robot products.</td>
    </tr>
    <tr>
      <td>⭐ **Model Zoo**</td>
      <td><a href="https://github.com/D-Robotics/rdk_model_zoo">Github Repository</a></td>
      <td>RDK Model Zoo is developed based on RDK and provides deployment routines for most mainstream algorithms. The routines include exporting D-Robotics *.bin models and using Python and other APIs to infer D-Robotics *.bin models. Some models also include data collection, model training, export, conversion, and deployment processes.</td>
    </tr>
    <tr>
      <td>📁 **TogetheRos File Server**</td>
      <td><a href="https://archive.d-robotics.cc/TogetheROS/source_code">Click to Visit</a></td>
      <td>Backup download source to prevent conditions from restricting code retrieval from GitHub.</td>
    </tr>
    <tr>
      <td><img src="http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/image.png" alt="icon" width="8%"/>  **RDK Studio**</td>
      <td><a href="https://developer.d-robotics.cc/rdkstudio">Visit Official Website</a></td>
      <td>RDK Studio provides device connection and management, quick AI sample experience, and fast access to the community forum. Board Support: Perfectly supports the use of the entire RDK series of boards, helping users manage all their RDK boards. System Updates: Offers a brand-new board upgrade method—users can obtain the latest RDK OS with one click and quickly complete board upgrades using just a data cable.</td>
    </tr>
    <tr>
      <td>🔄 **hbupdate**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hbupdate">Click to Visit</a></td>
      <td>hbupdate is a professional tool for burning operating system images and minimal boot images for RDK development boards.</td>
    </tr>
    <tr>
      <td>💾 **Image Burning Tool**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/static/fileData/1650948745611.rar">Rufus</a><br/><a href="https://developer.d-robotics.cc/en/rdkstudio">RDK Studio</a></td>
      <td>Software tool for writing image files to SD cards, supporting multiple image formats.</td>
    </tr>
    <tr>
      <td>🖥️ **SSH Remote Tool**</td>
      <td><a href="https://mobaxterm.mobatek.net/download.html">MobaXterm</a> / <a href="https://www.putty.org/?hl=zh-cn">Putty</a></td>
      <td>Tool software for remotely connecting to and managing RDK development kits, providing functions such as SSH and SFTP.</td>
    </tr>
    <tr>
      <td>⚙️ **Related Drivers**</td>
      <td><a href="https://developer.d-robotics.cc/api/v1/static/fileData/1651770860901.zip">Serial Port Driver (CP210x)</a> / <a href="https://developer.d-robotics.cc/api/v1/static/fileData/1651120312998.rar">android_hobot</a></td>
      <td>Necessary system drivers to ensure normal device connection and communication.</td>
    </tr>
  </tbody>
</table>
</div>

---
## Certification Resources

:::tip Certification Repository
📁 [Certification](https://archive.d-robotics.cc/downloads/certification/) - Contains certification files for all products
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🏷️ **Product Name**</th>
      <th>🇪🇺 **CE RED**</th>
      <th>🇪🇺 **CE EMC**</th>
      <th>🇺🇸 **FCC**</th>
      <th>🇯🇵 **MIC**</th>
      <th>🇨🇳 **SRRC**</th>
      <th>🇰🇷 **KCC**</th>
      <th>🇬🇧 **UKCA**</th>
      <th>🇪🇺 **RoHS**</th>
      <th>🇹🇭 **NBTC**</th>
      <th>🇲🇾 **SIRIM**</th>
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
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/RoHS/2402Z107564E_CNAS.pdf">📄 RoHS</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/NBTC.pdf">📄 NBTC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5/SIRIM.pdf">📄 SIRIM</a></td>
    </tr>
    <tr>
      <td>**RDK X3**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/CE/C03-2402T78337E-RF%20C2%20RED%20210115.pdf">📄 CE RED</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/CE/C02-2402T78337E-RF%20C2%20EMC%20210115.pdf">📄 CE EMC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/FCC/D-ROBOTICS%202BGUG-RDKX3K%20FCC%20Grant%20-%20DTS.PDF">📄 FCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/MIC/D-ROBOTICS%20211-240607%20MIC%20Radio%20Certificate.pdf">📄 MIC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/SRRC/SRRC_Approval_RDK_X3.pdf">📄 SRRC</a></td>
      <td>-</td>
      <td>-</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X3/RoHS/P-2401V86686E%20RDK%20X3.pdf">📄 RoHS</a></td>
      <td>-</td>
      <td>-</td>
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
      <td>-</td>
      <td>-</td>
    </tr>
     <tr>
      <td>**RDK X5 Module**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/CE/AOC_for_CE_certification.pdf">📄 CE RED</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/CE/AOC_for_CE_certification.pdf">📄 CE EMC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/FCC/D-ROBOTICS 2BGUG-RDKX5M FCC Grant - DSS.PDF">📄 FCC Grant - DSS.pdf</a><br/><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/FCC/D-ROBOTICS 2BGUG-RDKX5M FCC Grant - DTS.PDF">📄 FCC Grant - DTS.pdf</a><br/><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/FCC/D-ROBOTICS 2BGUG-RDKX5M FCC Grant - NII.PDF">📄 FCC Grant - NII.pdf</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/MIC/D-ROBOTICS 211-250814 MIC Radio Certificate.pdf">📄 MIC</a></td>
      <td>-</td>
       <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/KCC/RDKX5MD108064_KCC.pdf">📄 KCC</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/UKCA/AOC for UKCA 证书.pdf">📄 UKCA</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/certification/RDK_X5_MD/ROHS/AOC 2501X28112E-13.pdf">📄 RoHS</a></td>
      <td>-</td>
      <td>-</td>
    </tr>
  </tbody>
</table>
</div>

---
## Appendix: Historical Version Materials

### 📦 RDK X3 1.0 Version Materials

:::note Historical Version Information
The following materials are for RDK X3 1.0 version, for reference only.
:::

<div class="table-wrapper">
<table className="no-wrap-table">
  <thead>
    <tr>
      <th>🏷️ **Material Type**</th>
      <th>📄 **Specifications**</th>
      <th>🔧 **Schematics**</th>
      <th>📐 **Mechanical Drawings**</th>
      <th>🎯 **3D Models**</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>**RDK X3 1.0**</td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_1.0_Product_Brief.pdf">📋 Specifications</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_IO_Schematics.pdf">🔧 Schematics</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_2D_1.0.pdf">📐 2D Drawings</a></td>
      <td><a href="https://archive.d-robotics.cc/downloads/hardware/rdk_x3/RDK_X3_3D_Step_Models_1.0.STEP">🎯 3D Models</a></td>
    </tr>
  </tbody>
</table>
</div>

---

## Notes

### ⚠️ 1. Version Compatibility

:::warning Important Reminder
Please confirm hardware model matches the image before flashing to avoid compatibility issues.
:::

- **RDK X3 Pi 1.0 Version**: Download images from 1.x.x directory
- **RDK X3 Module**: Requires use with Raspberry Pi CM4 carrier board

### 🔥 2. Flashing Instructions

:::info Flashing Recommendations
- **RDK Module Series**: For first-time use, it's recommended to upgrade the system through [Flashing Tool](https://archive.d-robotics.cc/downloads/hbupdate/)
- **System Version Query**: Use command `cat /etc/version` or `rdkos_info`
:::

### 🔓 3. Open Source Resources

:::tip Open Source Support
System source code is hosted on [D-Robotics GitHub](https://github.com/d-robotics), contributions welcome!
:::

---

## 📞 Technical Support

If you encounter any issues during use, you can get help through the following methods:

- 🌐 [Developer Community](https://developer.d-robotics.cc/en)
- 📧 [Technical Support Email](mailto:developer@d-robotics.cc)
- 📱 Official Technical Discussion Group
