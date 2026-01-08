import React from 'react';
import DocPaginator from '@theme-original/DocPaginator';
import {useLocation} from '@docusaurus/router';
import type {Props} from '@theme/DocPaginator';

// 动态标题映射规则
const getCustomTitle = (permalink: string, locale: string): string | null => {
  const rules = {
    en: [
      // s100
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Quick_start$/, title: '1. Quick Start' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/System_configuration$/, title: '2. System Configuration'},
      { pattern: /^\/rdk_doc\/en\/rdk_s\/01_hardware_introduction$/, title: '1.1 Hardware Introduction' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Basic_Application$/, title: '3. Basic Application Development' },
      { pattern: /^\/Algorithm_Application$/, title: '4. Algorithms' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/02_install_os$/, title: '1.2 Install Operating System' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Quick_start\/hardware_introduction\/rdk_s100_mcu_port_expansion_board$/, title: '1.1.3 MCU Port Expansion Board' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/03_configuration_wizard$/, title: '1.3 Getting Started Configuration' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/System_configuration\/network_bluetooth$/, title: '2.1 Network and Bluetooth Configuration' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Basic_Application$/, title: '3. Basic Application Development' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Robot_development$/, title: '5. Robotics Application Development' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Basic_Development$/, title: '4. Algorithm Application Development' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/03_Python_Sample$/, title: '4.2 Reference Example (Python)' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/03_C\+\+_Sample$/, title: '4.3 Reference Example (C++)' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Application_case$/, title: '6. Application Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Advanced_development$/, title: '7. Advanced Development' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/hardware_development$/, title: '7.1 Hardware Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/linux_development$/, title: '7.2. Linux Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/driver_development_s100$/, title: '7.2.4 RDK S100 Driver Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/15_driver_hbmem$/, title: 'HBMEM Usage Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/16_driver_ethernet$/, title: 'Ethernet Usage Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/hardware_unit_test$/, title: '7.2.5 Hardware Unit Testing' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/03_multimedia_development$/, title: '7.3 Multimedia Development Guide' },
      { pattern: /\/rdk_doc\/en\/rdk_s\/category\/731\-s100/, title: '7.3.1 S100 Multimedia Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/02_multimedia_application$/, title: '7.3.2 Multimedia Application Development' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/04_toolchain_development$/, title: '7.4 Algorithm Toolchain Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/05_MCU_development$/, title: '7.5 MCU Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/01_S100$/, title: '7.5.1 S100 MCU Development Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/07_Advanced_development\/05_mcu_development\/01_S100\/12_mcu_port$/, title: 'PORT Module Guide' },
      { pattern: /^\/rdk_doc\/en\/rdk_s\/Appendix$/, title: '9. Appendix' },



      // x3/x5
      { pattern: /^\/rdk_doc\/en\/Quick_start$/, title: '1. Quick Start' },
      { pattern: /^\/rdk_doc\/en\/System_configuration$/, title: '2. System Configuration'},
      { pattern: /^\/rdk_doc\/en\/hardware_introduction$/, title: '1.1 Hardware Introduction' },
      { pattern: /^\/rdk_doc\/en\/install_os$/, title: '1.2 Install Operating System' },
      { pattern: /^\/rdk_doc\/en\/display_use$/, title: '1.5 Display Usage' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application$/, title: '3. Basic Application Development' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/40pin_define$/, title: '3.1.1 Pin Configuration and Definition' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/gpio$/, title: '3.1.2 Using GPIO' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/pwm$/, title: '3.1.3 Using PWM' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/uart$/, title: '3.1.4 UART_usage' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/i2c$/, title: '3.1.5 Using I2C' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/01_40pin_user_sample\/spi$/, title: '3.1.6 Using SPI' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\/bpu$/, title: '3.2.1 BPU Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/03_Basic_Application\/02_cdev_demo_sample$/, title: '3.2 C DEV Interface Examples' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\/decode2display$/, title: '3.2.2 decode2display Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\/rtsp2display$/, title: '3.2.3 rtsp2display Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\/vio_capture$/, title: '3.2.4 vio_capture Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\/vio2display$/, title: '3.2.5 vio2display Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/cdev_demo_sample\vio2encoder$/, title: '3.2.6 vio2encoder Sample Introduction' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/basic_sample$/, title: '3.3.1 Basic Image Classification Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/segment_sample$/, title: '3.3.2 Segment Model Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/yolov3_sample$/, title: '3.3.3 YOLOv3 Model Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/yolov5_sample$/, title: '3.3.4 YOLOv5 Model Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/yolov5x_sample$/, title: '3.3.5 YOLOv5x Model Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/centernet_sample$/, title: '3.3.6 CenterNet Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/yolov5s_v6_v7_sample$/, title: '3.3.7 YOLOv5s v6/v7 Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/usb_camera_sample$/, title: '3.3.8 USB Camera Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/mipi_camera_sample$/, title: '3.3.9 MIPI Camera Sample Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/web_display_camera_sample$/, title: '3.3.10 Web Display Camera Example Introduction' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/pydev_demo_sample\/decode_rtsp_stream$/, title: '3.3.11 RTSP Stream Decoding Example Introduction' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/vision\/mipi_camera$/, title: '3.4.1 Using MIPI Camera' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/vision\/usb_camera$/, title: '3.4.2 Using USB Camera' },

      { pattern: /^\/rdk_doc\/en\/03_Basic_Application\/02_audio$/, title: '3.5 Acoustic Solution' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/audio\/rdk_x5\/in_board_es8326$/, title: 'On-board Earphone Audio Port' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/audio\/rdk_x5\/audio_driver_hat2_rev2$/, title: 'Waveshare Audio Driver HAT REV2' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/audio\/rdk_x5\/wm8960_audio_hat$/, title: 'Waveshare WM8960 Audio HAT' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/audio\/rdk_x5\/hiwonder_rasb5$/, title: 'Hiwonder Carrier Board' },

      { pattern: /^\/rdk_doc\/en\/Basic_Application\/multi_media_sp_dev_api\/cdev_demo$/, title: '3.6.1  Reference Example （C++）' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/multi_media_sp_dev_api\/pydev_vio_demo$/, title: '3.6.2 Reference Examples (python)' },
      { pattern: /^\/rdk_doc\/en\/Basic_Application\/multi_media_sp_dev_api\/pydev_multimedia_api_x3$/, title: '3.6.3 RDK X3/X5 Multimedia Interface User Guide' },
     
      { pattern: /^\/rdk_doc\/en\/Robot_development$/, title: '5. Robotics Application' },
      { pattern: /^\/rdk_doc\/en\/Robot_development\/boxs\/generate\/hobot_xlm$/, title: 'DeepSeek large language model' },

      { pattern: /^\/rdk_doc\/en\/Application_case$/, title: '6. Application Development Guide' },
      { pattern: /^\/rdk_doc\/en\/Advanced_development$/, title: '7. Advanced Development' },
      { pattern: /^\/rdk_doc\/en\/hardware_development$/, title: '7.1 Hardware Development Guide' },

      { pattern: /^\/rdk_doc\/en\/linux_development$/, title: '7.2. Linux Development Guide' },
      { pattern: /^\/rdk_doc\/en\/Advanced_development\/linux_development\/driver_development_x5\/memory$/, title: 'Linux System Memory Usage' },
      { pattern: /^\/rdk_doc\/en\/Advanced_development\/linux_development\/hardware_unit_test$/, title: '7.4 Hardware Unit Testing' },
      { pattern: /^\/rdk_doc\/en\/03_multimedia_development$/, title: '7.3 RDK X3 Multimedia Development Guide' },
      { pattern: /^\/rdk_doc\/en\/04_toolchain_development$/, title: '7.4 Algorithm Toolchain Development Guide' },
      { pattern: /^\/rdk_doc\/en\/FAQ$/, title: '8. FAQs' },

    ],
    // zh: [
    //   { pattern: /\/Quick_start/, title: '1. 快速开始' },
    //   { pattern: /\/System_configuration/, title: '2. 系统配置' },
    //   { pattern: /\/Basic_Application/, title: '3. 基础应用' },
    //   { pattern: /\/Algorithm_Application/, title: '4. 算法应用' },
    // ]
  };

  const localeRules = rules[locale as keyof typeof rules] || rules.en;
  const matchedRule = localeRules.find(rule => rule.pattern.test(permalink));
  return matchedRule ? matchedRule.title : null;
};

export default function DocPaginatorWrapper(props: Props): JSX.Element {
  const { pathname } = useLocation();
  const { previous, next } = props;
  
  const getCurrentLocale = () => {
    if (pathname.includes('/zh/')) return 'zh';
    if (pathname.includes('/en/')) return 'en';
    return 'en';
  };
  
  const currentLocale = getCurrentLocale();
  
  const customNext = next ? {
    ...next,
    title: getCustomTitle(next.permalink, currentLocale) || next.title
  } : null;

  const customPrevious = previous ? {
    ...previous,
    title: getCustomTitle(previous.permalink, currentLocale) || previous.title
  } : null;

  return (
    <DocPaginator
      previous={customPrevious}
      next={customNext}
    />
  );
}