---
sidebar_position: 2
---

# 手势识别示例

1. 点击示例应用底部的下拉列表，已连接的所有 RDK 设备都会在此处显示，选择要安装示例应用的 RDK 设备。
   
   ![RDK 设备列表页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_example_nodered_static_image_select_device.png)

2. 点击 `示例应用` 进入 RDK 示例应用展示界面，可以选择示例点击 `Install Example` 进行安装。
   
   ![示例应用安装页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_example_application.png)

3. 示例应用安装成功后底部会出现可打开示例的应用，点击 Node-RED，如未安装 Node-RED 会弹出提示，需在 “设备管理 - 应用空间” 中安装 Node-RED。
   
   ![Node-RED安装页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_installnodered.png)

4. Node-RED 安装成功后，回到 `示例应用` 界面，点击所选示例下方的 Node-RED 图标，即可在 Node-RED 中打开示例应用。
   
   ![Node-RED打开示例页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_device_manage_hr_add_device_installnodered.png)

5. Node-RED 打开时会弹出更新提示：Node-RED 在有新版本可用时可以通知您，这能确保您及时获得最新的功能与修复。此功能需要向 Node-RED 团队发送匿名数据，其中不会包含您的流（flows）或用户的任何详细信息。关于所收集信息的完整说明及其使用方式，可以在提示信息中点击 `documentation` 查阅完整文档。您可以在用户设置中随时更改此选项。<font color="red">图片替换</font>

    ![Node-RED通知页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_example_nodered_static_image_help.png)

6. 点击 `Yes，enable notifications` 启用通知，或点击 `No，do not enable notifications` 禁用通知；设置完成后进入 Node-RED 导览窗口：
   - 在有新的Node-RED版本或您已安装的节点有更新时，接收通知以保持最新状态。
   - 快速查看哪些节点附有新文档图标，单击该图标可打开节点编辑对话框的 “描述” 选项卡。
   - 对调色板管理器进行了大量改进：
        - 搜索结果按下载量排序，助您快速找到最受欢迎的节点
        - 可查看作者已标注弃用的节点，避免使用不再推荐的节点
        - 已安装的节点可直接链接至对应节点文档
   - 从 Node-RED 4.1 版本导出的流程，现已包含需额外安装的模块信息。导入此类流程时，编辑器会告知缺失项，并协助安装。 
   - 核心节点进行了大量细微修复、文档更新和小幅改进。完整的变更列表请查看“帮助”侧边栏中的更新日志。主要更新包括：
        - Function 节点支持 node: 前缀模块
        - 可通过运行时设置，为 Function 节点配置全局超时时间
        - Debug 侧边栏中错误对象的显示方式已优化
        - 以及其他多项改进...

7. 阅读完导览信息后点击 `关闭` ，关闭导览弹窗。进入示例应用内容界面。
    
     ![示例应用列表页面](http://rdk-doc.oss-cn-beijing.aliyuncs.com/doc/img/01_Quick_start/image/rdk_studio/rdk_studio_left_menu_example_nodered_static_image_help.png)