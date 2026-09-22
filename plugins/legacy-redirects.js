// 旧站 /rdk_doc/ 路径跳转到新资料中心对应页面（add by ql for url redirect 2026-09-20；430 条有曝光页由 IA 于 2026-09-21 补入，17 条 C++ Sample 死目标已剔除）
//
// 为什么不用官方的 @docusaurus/plugin-client-redirects：
// 官方插件的 `redirects` 选项不区分语言，同一条 {from,to} 会应用到所有 locale、
// 指向同一个 to；而这里的旧 URL 中英文目标地址不同（新站 /rdk_x_doc/、/rdk_s_doc/、
// /tros_doc/ 与各自的 /en/），需要按 locale 写不同的目标，官方插件做不到。故用本
// 自定义插件在 postBuild 阶段按语言生成跳转页。
const path = require('path');
const fs = require('fs-extra');

// from 是相对"各语言版本站点根"的路径（默认语言 build/、英文 build/en/，baseUrl 由部署层映射），
// 与旧站 URL 一一对应：from=/rdk_s/... → 旧站 /rdk_doc/rdk_s/...（英文 /rdk_doc/en/rdk_s/...）。
// to 按 locale 区分；某语言缺省表示该语言不生成跳转。
// 430 条有曝光页：目标用精确路径（旧 rdk_doc → rdk_x_doc/rdk_s_doc/tros_doc），
// 不带 ?p= 参数；路径本身已定位到具体页面，且无版本号、天然最新。
const REDIRECTS = [

  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/01_Quick_start/01_hardware_introduction/01_rdk_s100/01_rdk_s100_kit?p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100_camera_expansion_board',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/01_Quick_start/01_hardware_introduction/01_rdk_s100/02_rdk_s100_camera_expansion_board?p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/01_Quick_start/01_hardware_introduction/01_rdk_s100/02_rdk_s100_camera_expansion_board?p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Quick_start/hardware_introduction/rdk_s100_mcu_port_expansion_board',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/hardware_introduction/rdk_s100/rdk_s100_mcu_port_expansion_board?p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/Quick_start/hardware_introduction/rdk_s100/rdk_s100_mcu_port_expansion_board?p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Advanced_development/toolchain_development/LLM_Toolchain',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/toolchain_development/LLM_Toolchain/rdk_s100/s100_LLM_Toolchain?p=RDK+S100',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/Advanced_development/toolchain_development/LLM_Toolchain/rdk_s100/s100_LLM_Toolchain?p=RDK+S100',
    },
  },
  {
    from: '/rdk_s/Advanced_development/toolchain_development/overview',
    to: {
      'zh-Hans':
        'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/toolchain_development/algorithm_toolchain?p=RDK+S100',
    },
  },
  {
    from: '/Quick_start/hardware_introduction/rdk_x5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/hardware_introduction/rdk_x5',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Quick_start/hardware_introduction/rdk_x5'
    },
  },
  {
    from: '/Quick_start/download',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/download',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Quick_start/download'
    },
  },
  {
    from: '/Robot_development/quick_demo/demo_sensor',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/demo_sensor',
      en: 'https://developer.d-robotics.cc/tros_doc/en/quick_demo/demo_sensor'
    },
  },
  {
    from: '/rdk_s/RDK',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/RDK',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/RDK'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/hobot_stereonet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/hobot_stereonet',
      en: 'https://developer.d-robotics.cc/tros_doc/en/boxs/spatial/hobot_stereonet'
    },
  },
  {
    from: '/Robot_development/apps/slam',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/slam',
      en: 'https://developer.d-robotics.cc/tros_doc/en/apps/slam'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/hardware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/hardware',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Advanced_development/hardware_development/rdk_x5/hardware'
    },
  },
  {
    from: '/Quick_start',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Quick_start'
    },
  },
  {
    from: '/Robot_development/apps/car_tracking',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/car_tracking',
      en: 'https://developer.d-robotics.cc/tros_doc/en/apps/car_tracking'
    },
  },
  {
    from: '/Robot_development/quick_demo/ai_predict',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/ai_predict',
      en: 'https://developer.d-robotics.cc/tros_doc/en/quick_demo/ai_predict'
    },
  },
  {
    from: '/Robot_development/apps/car_audio_control',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/car_audio_control',
      en: 'https://developer.d-robotics.cc/tros_doc/en/apps/car_audio_control'
    },
  },
  {
    from: '/System_configuration/frequency_management',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration/frequency_management',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/System_configuration/frequency_management'
    },
  },
  {
    from: '/Robot_development/quick_start/changelog',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/changelog',
      en: 'https://developer.d-robotics.cc/tros_doc/en/quick_start/changelog'
    },
  },
  {
    from: '/Robot_development/quick_demo/hobot_tts',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/hobot_tts',
      en: 'https://developer.d-robotics.cc/tros_doc/en/quick_demo/hobot_tts'
    },
  },
  {
    from: '/Robot_development/quick_start/preparation',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/preparation',
      en: 'https://developer.d-robotics.cc/tros_doc/en/quick_start/preparation'
    },
  },
  {
    from: '/FAQ/applications_and_examples',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/applications_and_examples',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/FAQ/applications_and_examples'
    },
  },
  {
    from: '/Basic_Application/vision/RDK_X5/mipi_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/vision/RDK_X5/mipi_camera',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Basic_Application/vision/RDK_X5/mipi_camera'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Advanced_development/linux_development/driver_development_x5'
    },
  },
  {
    from: '/Robot_development/boxs/segmentation/yolov8_seg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/segmentation/yolov8_seg'
    },
  },
  {
    from: '/Robot_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc'
    },
  },
  {
    from: '/Basic_Application/vision/RDK_X3/mipi_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/vision/RDK_X3/mipi_camera',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Basic_Application/vision/RDK_X3/mipi_camera'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/Advanced_development/linux_development/driver_development'
    },
  },
  {
    from: '/FAQ',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ'
    },
  },
  {
    from: '/Robot_development/apps/car_gesture_control',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/car_gesture_control',
      en: 'https://developer.d-robotics.cc/tros_doc/en/apps/car_gesture_control'
    },
  },
  {
    from: '/hardware_introduction',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/hardware_introduction',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/hardware_introduction'
    },
  },
  {
    from: '/linux_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/linux_development',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/linux_development'
    },
  },
  {
    from: '/03_multimedia_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/03_multimedia_development',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/03_multimedia_development'
    },
  },
  {
    from: '/Basic_Development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Development',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/Basic_Development'
    },
  },
  {
    from: '/Robot_development/tros',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros',
      en: 'https://developer.d-robotics.cc/tros_doc/en/tros'
    },
  },
  {
    from: '/hardware_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/hardware_development'
    },
  },
  {
    from: '/04_toolchain_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/04_toolchain_development',
      en: 'https://developer.d-robotics.cc/rdk_x_doc/en/04_toolchain_development'
    },
  },
  {
    from: '/install_os',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/install_os'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test'
    },
  },
  {
    from: '/rdk_s/02_install_os',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/02_install_os',
      en: 'https://developer.d-robotics.cc/rdk_s_doc/en/02_install_os'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/mono3d_indoor_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/mono3d_indoor_detection'
    },
  },
  {
    from: '/rdk_s/01_hardware_introduction',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/01_hardware_introduction'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_hrut_boardid_rdkx5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_hrut_boardid_rdkx5'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_dmesg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_dmesg'
    },
  },
  {
    from: '/Basic_Application/accessory_instructions/rdk_x5/imu/icm42688',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/accessory_instructions/rdk_x5/imu/icm42688'
    },
  },
  {
    from: '/Robot_development/boxs/body/hand_lmk_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/hand_lmk_detection'
    },
  },
  {
    from: '/rdk_s/03_multimedia_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/03_multimedia_development'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/open_instance_segment_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/open_instance_segment_sample'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_hrut_somstatus',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_hrut_somstatus'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/rtsp2display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/rtsp2display'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_pinctrl_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_pinctrl_dev'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/USB_Camera_yolov5x',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/USB_Camera_yolov5x'
    },
  },
  {
    from: '/Robot_development/boxs/segmentation/mono_mobilesam',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/segmentation/mono_mobilesam'
    },
  },
  {
    from: '/rdk_s/System_configuration/network_bluetooth',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/network_bluetooth'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/bpu_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/bpu_api'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_ssh',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_ssh'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/ddr_bandwidth',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/ddr_bandwidth'
    },
  },
  {
    from: '/Robot_development/boxs/generate/hobot_llamacpp',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/generate/hobot_llamacpp'
    },
  },
  {
    from: '/rdk_s/16_driver_ethernet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/16_driver_ethernet'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/object_decoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/object_decoder'
    },
  },
  {
    from: '/rdk_s/FAQ/interface',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/interface'
    },
  },
  {
    from: '/Release_Note/release_note',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Release_Note/release_note'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/LaneNet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/LaneNet'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5_module/hardware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5_module/hardware'
    },
  },
  {
    from: '/Robot_development/boxs/classification/mobilenetv2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/classification/mobilenetv2'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/overview'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_zip',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_zip'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_ddr_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_ddr_dev'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Summary',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Summary'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/vio_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/vio_api'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_pwm',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_pwm'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/spi',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/spi'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/yolov5x_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/yolov5x_sample'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/classification_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/classification_sample'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_devmem',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_devmem'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_mount',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_mount'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/object_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/object_camera'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/overview'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_io_domain_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_io_domain_dev'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_usb',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_usb'
    },
  },
  {
    from: '/Robot_development/tros_dev/flame_graph',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros_dev/flame_graph'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/UNetMobileNet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/UNetMobileNet'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/uart',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/uart'
    },
  },
  {
    from: '/Robot_development/quick_demo/hobot_codec',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/hobot_codec'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_mount',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_mount'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_zip',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_zip'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/overview'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_i2c_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_i2c_dev'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/vio_capture',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/vio_capture'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/detection_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/detection_sample'
    },
  },
  {
    from: '/03_Basic_Application/02_cdev_demo_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/03_Basic_Application/02_cdev_demo_sample'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Ultralytics_YOLOv5x',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Ultralytics_YOLOv5x'
    },
  },
  {
    from: '/Advanced_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development'
    },
  },
  {
    from: '/Robot_development/boxs/function/mono_pwcnet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/function/mono_pwcnet'
    },
  },
  {
    from: '/rdk_s/FAQ/toolchain',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/toolchain'
    },
  },
  {
    from: '/rdk_s/FAQ/applications_and_examples',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/applications_and_examples'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_top',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_top'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/object_encoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/object_encoder'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/kernel_headers',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/kernel_headers'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/accessory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/accessory'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/uboot_kernel_config',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/uboot_kernel_config'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/performance_debug',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/performance_debug'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_top',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_top'
    },
  },
  {
    from: '/Quick_start/accessory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/accessory'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/cdev_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/cdev_demo'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_io_domain_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_io_domain_dev'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/overview'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_camera'
    },
  },
  {
    from: '/Application_case/line_follower',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Application_case/line_follower'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/video_input',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/video_input'
    },
  },
  {
    from: '/rdk_s/Robot_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Robot_development'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_dpkg-deb',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_dpkg-deb'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_api_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/pydev_api_demo'
    },
  },
  {
    from: '/rdk_s/FAQ/multimedia',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/multimedia'
    },
  },
  {
    from: '/Robot_development/apps/video_boxs',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/video_boxs'
    },
  },
  {
    from: '/rdk_s/System_configuration/gui_network_config',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/gui_network_config'
    },
  },
  {
    from: '/Application_case',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Application_case'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/bpu_cpu_ddr_stress',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/bpu_cpu_ddr_stress'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/elevation_net',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/elevation_net'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_hrut_boardid',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_hrut_boardid'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/wifi_performance',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/wifi_performance'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/vio2encoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/vio2encoder'
    },
  },
  {
    from: '/Robot_development/boxs/body/mono_face_landmarks_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/mono_face_landmarks_detection'
    },
  },
  {
    from: '/rdk_s/Application_case',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Application_case'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_gdc',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_gdc'
    },
  },
  {
    from: '/Robot_development/quick_start/install_tros',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/install_tros'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Ultralytics_YOLO11_Seg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Ultralytics_YOLO11_Seg'
    },
  },
  {
    from: '/category/imu',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/category/imu'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/sys_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/sys_api'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3/boot_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3/boot_system'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_uart_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_uart_dev'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_hrut_socuid',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_hrut_socuid'
    },
  },
  {
    from: '/03_Basic_Application/05_audio/rdk_x3_and_rdk_x3_module',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/03_Basic_Application/05_audio/rdk_x3_and_rdk_x3_module'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3_module/interface',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3_module/interface'
    },
  },
  {
    from: '/Robot_development/boxs/body/hand_lmk_gesture_mediapipe',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/hand_lmk_gesture_mediapipe'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_find',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_find'
    },
  },
  {
    from: '/Robot_development/boxs/driver/hobot_bev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/driver/hobot_bev'
    },
  },
  {
    from: '/rdk_s/02_multimedia_application',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/02_multimedia_application'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_pym',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_pym'
    },
  },
  {
    from: '/Robot_development/boxs/driver/parking_perception',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/driver/parking_perception'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_uart',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_uart'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_ifconfig',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_ifconfig'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/web_display_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/web_display_camera_sample'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_tar',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_tar'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Ultralytics_YOLO11',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Ultralytics_YOLO11'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_apt',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_apt'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_ps',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_ps'
    },
  },
  {
    from: '/Advanced_development/linux_development/realtime_kernel',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/realtime_kernel'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/dstereo_occupancy',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/dstereo_occupancy'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/pydev_api_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/pydev_api_demo'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/isp_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/isp_system'
    },
  },
  {
    from: '/Basic_Application/vision/RDK_X5/usb_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/vision/RDK_X5/usb_camera'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_netstat',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_netstat'
    },
  },
  {
    from: '/rdk_s/hardware_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/hardware_development'
    },
  },
  {
    from: '/Robot_development/tros_dev/breakpad',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros_dev/breakpad'
    },
  },
  {
    from: '/rdk_s/FAQ/hardware_and_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/hardware_and_system'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_environment_reliability',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_environment_reliability'
    },
  },
  {
    from: '/rdk_s/OTA',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/OTA'
    },
  },
  {
    from: '/Robot_development/quick_demo/demo_tool',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/demo_tool'
    },
  },
  {
    from: '/System_configuration/srpi-config',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration/srpi-config'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/yolov3_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/yolov3_sample'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_grep',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_grep'
    },
  },
  {
    from: '/Basic_Application',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x3_and_rdk_x3_module/audio_driver_hat2_rev2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x3_and_rdk_x3_module/audio_driver_hat2_rev2'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_i2c_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_i2c_dev'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/video_processing',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/video_processing'
    },
  },
  {
    from: '/Robot_development/quick_demo/demo_render',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/demo_render'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/decoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/decoder_api'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_ssh',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_ssh'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3_module/FAQ',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3_module/FAQ'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/cdev/vio_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/cdev/vio_api'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_rdk-backup',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_rdk-backup'
    },
  },
  {
    from: '/Robot_development/tros_dev/mono2d_trash_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros_dev/mono2d_trash_detection'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_nohup',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_nohup'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/cdev/sys_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/cdev/sys_api'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_tar',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_tar'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3/FAQ',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3/FAQ'
    },
  },
  {
    from: '/rdk_s/Quick_start/classification',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/classification'
    },
  },
  {
    from: '/Robot_development/quick_start/hello_world',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/hello_world'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/memory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/memory'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/region_processing',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/region_processing'
    },
  },
  {
    from: '/FAQ/tros_ros',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/tros_ros'
    },
  },
  {
    from: '/rdk_s/Advanced_development/rdk_gen',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/rdk_gen'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/overview'
    },
  },
  {
    from: '/rdk_s/Appendix',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x5/audio_driver_hat2_rev2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x5/audio_driver_hat2_rev2'
    },
  },
  {
    from: '/Robot_development/boxs/body/hand_gesture_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/hand_gesture_detection'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/ResNet18',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/ResNet18'
    },
  },
  {
    from: '/Robot_development/apps/car_audio_tracking',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/car_audio_tracking'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_decoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_decoder'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/video_decode',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/video_decode'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_devmem',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_devmem'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_apt',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_apt'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/decoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/decoder_api'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3_module/system_burn',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3_module/system_burn'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_gpio_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_gpio_dev'
    },
  },
  {
    from: '/Robot_development/boxs/body/mono2d_yolo_pose',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/mono2d_yolo_pose'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/overview'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/hbmem_sample_guide',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/hbmem_sample_guide'
    },
  },
  {
    from: '/rdk_s/05_MCU_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/05_MCU_development'
    },
  },
  {
    from: '/Robot_development/boxs/generate/hobot_xlm',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/generate/hobot_xlm'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_rdkos_info',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_rdkos_info'
    },
  },
  {
    from: '/rdk_s/Quick_start/configuration_wizard/configuration_wizard_s100',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/configuration_wizard/configuration_wizard_s100'
    },
  },
  {
    from: '/rdk_s/Quick_start/remote_login',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/remote_login'
    },
  },
  {
    from: '/Application_case/amr',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Application_case/amr'
    },
  },
  {
    from: '/Robot_development/boxs/detection/efficientnet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/efficientnet'
    },
  },
  {
    from: '/rdk_s/13_driver_pcie',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/13_driver_pcie'
    },
  },
  {
    from: '/Robot_development/quick_start/cross_compile',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/cross_compile'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_dpkg-deb',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_dpkg-deb'
    },
  },
  {
    from: '/Robot_development/boxs/function/hobot_clip',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/function/hobot_clip'
    },
  },
  {
    from: '/Basic_Application/accessory_instructions/rdk_x5/imu/rdk_imu_module',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/accessory_instructions/rdk_x5/imu/rdk_imu_module'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/ai-python-api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/ai-python-api'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/yolov5s_v6_v7_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/yolov5s_v6_v7_sample'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/environment_build',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/environment_build'
    },
  },
  {
    from: '/03_Basic_Application/02_audio',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/03_Basic_Application/02_audio'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/centernet_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/centernet_sample'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_pipeline',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_pipeline'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/i2c',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/i2c'
    },
  },
  {
    from: '/Advanced_development/toolchain_development/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/toolchain_development/overview'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_dpkg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_dpkg'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_lcd',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_lcd'
    },
  },
  {
    from: '/rdk_s/Application_case/intro',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Application_case/intro'
    },
  },
  {
    from: '/Robot_development/apps/fall_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/fall_detection'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_hrut_ps',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_hrut_ps'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_ip',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_ip'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_scp',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_scp'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_decoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_decoder'
    },
  },
  {
    from: '/FAQ/multimedia',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/multimedia'
    },
  },
  {
    from: '/Advanced_development/linux_development/environment_build',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/environment_build'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_gpu_3d',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_gpu_3d'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_gpio_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_gpio_dev'
    },
  },
  {
    from: '/Robot_development/boxs/segmentation/mono_edgesam',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/segmentation/mono_edgesam'
    },
  },
  {
    from: '/rdk_s/System_configuration/share_file_tool',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/share_file_tool'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Ultralytics_YOLOE11_Seg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Ultralytics_YOLOE11_Seg'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3_module/nand_flash_firmware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3_module/nand_flash_firmware'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_bpu_sysfs_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_bpu_sysfs_dev'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x5/in_board_es8326',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x5/in_board_es8326'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/Ultralytics_YOLO11_Pose',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/Ultralytics_YOLO11_Pose'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_hrut_ps',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_hrut_ps'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_camera'
    },
  },
  {
    from: '/Robot_development/quick_demo/demo_cv',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/demo_cv'
    },
  },
  {
    from: '/rdk_s/Basic_Application',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/pose_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/pose_sample'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/uart_stress',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/uart_stress'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/instance_segment_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/instance_segment_sample'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/OTA/ota_miniboot',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/OTA/ota_miniboot'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_spi',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_spi'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/PaddleOCR',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/PaddleOCR'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_rsync',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_rsync'
    },
  },
  {
    from: '/rdk_s/04_toolchain_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/04_toolchain_development'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_display'
    },
  },
  {
    from: '/Robot_development/boxs/segmentation/mobilenet_unet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/segmentation/mobilenet_unet'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_thermal_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_thermal_dev'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/basic_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/basic_sample'
    },
  },
  {
    from: '/rdk_s/Quick_start/download',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Quick_start/download'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3_module/camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3_module/camera'
    },
  },
  {
    from: '/Robot_development/boxs/detection/yolo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/yolo'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/cdev_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/cdev_demo'
    },
  },
  {
    from: '/Robot_development/quick_demo/demo_communication',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_demo/demo_communication'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_rdk-miniboot-update',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_rdk-miniboot-update'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/mipi_camera_yolov5x',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/mipi_camera_yolov5x'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3_module/accessory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3_module/accessory'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_hrut_socuid',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_hrut_socuid'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_ps',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_ps'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3/accessory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3/accessory'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/segment_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/segment_sample'
    },
  },
  {
    from: '/rdk_s/Advanced_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3/nand_flash_firmware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3/nand_flash_firmware'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_thermal_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_thermal_dev'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_pwm_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_pwm_dev'
    },
  },
  {
    from: '/rdk_s/System_configuration/self_start',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/self_start'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/hobot_vio',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/hobot_vio'
    },
  },
  {
    from: '/Quick_start/display_use/display_rdkx5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/display_use/display_rdkx5'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/debug_info',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/debug_info'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3/hardware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3/hardware'
    },
  },
  {
    from: '/Quick_start/rdk_studio',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/rdk_studio'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/ASR',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/ASR'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sunrise_camera_user_guide',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sunrise_camera_user_guide'
    },
  },
  {
    from: '/rdk_s/linux_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/linux_development'
    },
  },
  {
    from: '/rdk_s/hardware_unit_test',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/hardware_unit_test'
    },
  },
  {
    from: '/rdk_s/Advanced_development/hardware_development/hardware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/hardware_development/hardware'
    },
  },
  {
    from: '/display_use',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/display_use'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/cdev/encoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/cdev/encoder_api'
    },
  },
  {
    from: '/install_os/rdk_x3_module',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/install_os/rdk_x3_module'
    },
  },
  {
    from: '/Quick_start/remote_login',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/remote_login'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_find',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_find'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/uboot_kernel_config',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/uboot_kernel_config'
    },
  },
  {
    from: '/Appendix/rdk-command-manual/cmd_rdkos_info',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/rdk-command-manual/cmd_rdkos_info'
    },
  },
  {
    from: '/FAQ/hardware_and_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/hardware_and_system'
    },
  },
  {
    from: '/System_configuration/config_txt',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration/config_txt'
    },
  },
  {
    from: '/Robot_development/apps/parking_search',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/parking_search'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_ethernet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_ethernet'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/encoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/encoder_api'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/sys_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/sys_api'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3_module/hardware',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3_module/hardware'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_bpu_mem_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_bpu_mem_dev'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/OTA/ota_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/OTA/ota_system'
    },
  },
  {
    from: '/Robot_development/quick_start/ros_pkg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/quick_start/ros_pkg'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_encoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_encoder'
    },
  },
  {
    from: '/rdk_s/FAQ',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ'
    },
  },
  {
    from: '/rdk_s/System_configuration',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_spi_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_spi_dev'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_pinctrl_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_pinctrl_dev'
    },
  },
  {
    from: '/Robot_development/boxs/detection/hobot_dosod',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/hobot_dosod'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_netstat',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_netstat'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/rtsp_yolov5x_display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/rtsp_yolov5x_display'
    },
  },
  {
    from: '/Robot_development/boxs/body/mono_face_age_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/mono_face_age_detection'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/display_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/display_api'
    },
  },
  {
    from: '/Quick_start/classification',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/classification'
    },
  },
  {
    from: '/Robot_development/boxs/spatial/stereo_imu_cam',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/spatial/stereo_imu_cam'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/WebSocket_yolov5x',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/WebSocket_yolov5x'
    },
  },
  {
    from: '/Robot_development/boxs/generate/hobot_llm',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/generate/hobot_llm'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sunrise_camera_develop_guide',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sunrise_camera_develop_guide'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_emmc',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_emmc'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/object_display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/object_display'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/usb_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/usb_camera_sample'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_grep',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_grep'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/kernel_debug',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/kernel_debug'
    },
  },
  {
    from: '/Robot_development/tros_dev/zero_copy',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros_dev/zero_copy'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/cpu_performance',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/cpu_performance'
    },
  },
  {
    from: '/rdk_s/Basic_Development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Development'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/vio_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/vio_api'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/pydev/pydev_multimedia_api_s100',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/pydev/pydev_multimedia_api_s100'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/overview'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/10-3d_gpu',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/10-3d_gpu'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/bpu_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/bpu_api'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/gpio',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/gpio'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/ethernet_performance',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/ethernet_performance'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/bpu',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/bpu'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/video_output',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/video_output'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/Python_Sample/MobileNetV2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/Python_Sample/MobileNetV2'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_hrut_boardid',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_hrut_boardid'
    },
  },
  {
    from: '/Advanced_development/linux_development/kernel_headers',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/kernel_headers'
    },
  },
  {
    from: '/Robot_development/boxs/body/mono2d_body_detection',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/mono2d_body_detection'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/can',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/can'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/decode_rtsp_stream',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/decode_rtsp_stream'
    },
  },
  {
    from: '/Basic_Application/vision/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/vision/overview'
    },
  },
  {
    from: '/RDK',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/RDK'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_rtc_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_rtc_dev'
    },
  },
  {
    from: '/rdk_s/03_configuration_wizard',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/03_configuration_wizard'
    },
  },
  {
    from: '/category/37-%E9%85%8D%E4%BB%B6%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/category/37-%E9%85%8D%E4%BB%B6%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E'
    },
  },
  {
    from: '/System_configuration',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/the_auto_test',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/the_auto_test'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_vin',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_vin'
    },
  },
  {
    from: '/FAQ/desktop_app',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/desktop_app'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/video_encode',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/video_encode'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/pydev_vio_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/pydev_vio_demo'
    },
  },
  {
    from: '/Robot_development/boxs/body/mono_edgetam',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/mono_edgetam'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/realtime_kernel',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/realtime_kernel'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_watchdog_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_watchdog_dev'
    },
  },
  {
    from: '/Quick_start/hardware_introduction/rdk_x3',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/hardware_introduction/rdk_x3'
    },
  },
  {
    from: '/Robot_development/tros_dev/ai_predict',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/tros_dev/ai_predict'
    },
  },
  {
    from: '/rdk_s/System_configuration/frequency_management',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/frequency_management'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_scp',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_scp'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_dpkg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_dpkg'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_route',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_route'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_ifconfig',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_ifconfig'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_dmesg',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_dmesg'
    },
  },
  {
    from: '/rdk_s/03_Python_Sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/03_Python_Sample'
    },
  },
  {
    from: '/Robot_development/boxs/audio/hobot_audio',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/audio/hobot_audio'
    },
  },
  {
    from: '/Advanced_development/linux_development/hardware_unit_test/test_cpu',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/hardware_unit_test/test_cpu'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/cdev/decoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/cdev/decoder_api'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_route',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_route'
    },
  },
  {
    from: '/Robot_development/boxs/driver/hobot_centerpoint',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/driver/hobot_centerpoint'
    },
  },
  {
    from: '/rdk_s/Advanced_development/vdsp_development',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/vdsp_development'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/spi_stress',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/spi_stress'
    },
  },
  {
    from: '/Robot_development/apps/navigation2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/navigation2'
    },
  },
  {
    from: '/rdk_s/Algorithm_Application/python-api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Algorithm_Application/python-api'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_isp',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_isp'
    },
  },
  {
    from: '/rdk_s/System_configuration/config_txt',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/config_txt'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/display'
    },
  },
  {
    from: '/FAQ/interface',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/interface'
    },
  },
  {
    from: '/install_os/rdk_x3',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/install_os/rdk_x3'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/usb_performance',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/usb_performance'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/40pin_define',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/40pin_define'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3/system_burn',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3/system_burn'
    },
  },
  {
    from: '/search',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/search'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_display'
    },
  },
  {
    from: '/rdk_s/Advanced_development/hardware_development/accessory',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/hardware_development/accessory'
    },
  },
  {
    from: '/rdk_s/15_driver_hbmem',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/15_driver_hbmem'
    },
  },
  {
    from: '/rdk_s/Basic_Application/Image/usb_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/Image/usb_camera'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x5/wm8960_audio_hat',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x5/wm8960_audio_hat'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_nohup',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_nohup'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_codec_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_codec_dev'
    },
  },
  {
    from: '/Basic_Application/01_40pin_user_sample/pwm',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/01_40pin_user_sample/pwm'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/segment_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/segment_sample'
    },
  },
  {
    from: '/FAQ/toolchain',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/FAQ/toolchain'
    },
  },
  {
    from: '/Basic_Application/vision/RDK_X3/usb_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/vision/RDK_X3/usb_camera'
    },
  },
  {
    from: '/Robot_development/boxs/detection/fcos',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/fcos'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_lowpower',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_lowpower'
    },
  },
  {
    from: '/Robot_development/boxs/body/reid',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/body/reid'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/encoder_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/cdev_multimedia_api_x3/encoder_api'
    },
  },
  {
    from: '/rdk_s/Quick_start',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Quick_start'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/system_control',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/system_control'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_encoder',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/pydev_multimedia_api_x5/object_encoder'
    },
  },
  {
    from: '/Robot_development/boxs/detection/mobilenet',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/mobilenet'
    },
  },
  {
    from: '/rdk_s/Basic_Application/Image/mipi_camera',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/Image/mipi_camera'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/overview'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/POE',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/POE'
    },
  },
  {
    from: '/Robot_development/boxs/detection/hobot_yolo_world',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/detection/hobot_yolo_world'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development_x5/driver_spi_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development_x5/driver_spi_dev'
    },
  },
  {
    from: '/Quick_start/install_os/rdk_x3_module/boot_system',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/install_os/rdk_x3_module/boot_system'
    },
  },
  {
    from: '/rdk_s/Appendix/linux-command-manual/cmd_ip',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/linux-command-manual/cmd_ip'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x5/V4l2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x5/V4l2'
    },
  },
  {
    from: '/rdk_s/Advanced_development/multimedia_development/multimedia_application/sample_codec',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/multimedia_development/multimedia_application/sample_codec'
    },
  },
  {
    from: '/rdk_s/Advanced_development/linux_development/hardware_unit_test/emmc_stress',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Advanced_development/linux_development/hardware_unit_test/emmc_stress'
    },
  },
  {
    from: '/Advanced_development/linux_development/driver_development/driver_uart_dev',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/linux_development/driver_development/driver_uart_dev'
    },
  },
  {
    from: '/rdk_s/FAQ/tros_ros',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/tros_ros'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/overview',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/overview'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/pydev_api_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/pydev_api_demo'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/vio2display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/vio2display'
    },
  },
  {
    from: '/rdk_s/Appendix/rdk-command-manual/cmd_hrut_somstatus',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Appendix/rdk-command-manual/cmd_hrut_somstatus'
    },
  },
  {
    from: '/Robot_development/boxs/audio/sensevoice_ros2',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/boxs/audio/sensevoice_ros2'
    },
  },
  {
    from: '/rdk_s/System_configuration/srpi-config',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/System_configuration/srpi-config'
    },
  },
  {
    from: '/rdk_s/Basic_Application/multi_media/multi_media_api/cdev/display_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/Basic_Application/multi_media/multi_media_api/cdev/display_api'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/mipi_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/mipi_camera_sample'
    },
  },
  {
    from: '/Robot_development/apps/hobot_llamacpp',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/tros_doc/apps/hobot_llamacpp'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x3_and_rdk_x3_module/wm8960_audio_hat',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x3_and_rdk_x3_module/wm8960_audio_hat'
    },
  },
  {
    from: '/Quick_start/configuration_wizard',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Quick_start/configuration_wizard'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/display_api',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/display_api'
    },
  },
  {
    from: '/Appendix',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix'
    },
  },
  {
    from: '/Advanced_development/multimedia_development/multimedia_samples',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/multimedia_development/multimedia_samples'
    },
  },
  {
    from: '/rdk_s/FAQ/desktop_app',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_s_doc/FAQ/desktop_app'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/web_display_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/web_display_camera_sample'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/yolov5_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/yolov5_sample'
    },
  },
  {
    from: '/System_configuration/self_start',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration/self_start'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X3/usb_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X3/usb_camera_sample'
    },
  },
  {
    from: '/Basic_Application/pydev_demo_sample/RDK_X5/mipi_camera_sample',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/pydev_demo_sample/RDK_X5/mipi_camera_sample'
    },
  },
  {
    from: '/System_configuration/network_blueteeth',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/System_configuration/network_blueteeth'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_pyeasy_dnn',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X3/pydev_multimedia_api_x3/object_pyeasy_dnn'
    },
  },
  {
    from: '/Advanced_development/hardware_development/rdk_x3_module/display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Advanced_development/hardware_development/rdk_x3_module/display'
    },
  },
  {
    from: '/Basic_Application/audio/rdk_x5/hiwonder_rasb5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/audio/rdk_x5/hiwonder_rasb5'
    },
  },
  {
    from: '/Basic_Application/cdev_demo_sample/decode2display',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/cdev_demo_sample/decode2display'
    },
  },
  {
    from: '/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/cdev_demo',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Basic_Application/multi_media_sp_dev_api/RDK_X5/cdev_multimedia_api_x5/cdev_demo'
    },
  },
  {
    from: '/Appendix/linux-command-manual/cmd_rsync',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/Appendix/linux-command-manual/cmd_rsync'
    },
  },
  {
    from: '/03_Basic_Application/05_audio/rdk_x5',
    to: {
      'zh-Hans': 'https://developer.d-robotics.cc/rdk_x_doc/03_Basic_Application/05_audio/rdk_x5'
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
        // 直接覆盖：旧文档仍长期保留在仓库，但原路由一律生成跳转页（不再依赖 /legacy/ 迁移腾空路由）
        await fs.outputFile(filePath, renderRedirectPage(to));
      }
    },
  };
};
