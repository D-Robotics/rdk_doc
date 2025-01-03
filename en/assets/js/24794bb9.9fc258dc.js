"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[4984],{2473:(e,n,t)=>{t.r(n),t.d(n,{assets:()=>o,contentTitle:()=>d,default:()=>h,frontMatter:()=>s,metadata:()=>a,toc:()=>c});var r=t(74848),i=t(28453);const s={sidebar_position:2},d="1.1.2 RDK X5",a={id:"Quick_start/hardware_introduction/rdk_x5",title:"1.1.2 RDK X5",description:"Interface Overview",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/01_Quick_start/hardware_introduction/rdk_x5.md",sourceDirName:"01_Quick_start/hardware_introduction",slug:"/Quick_start/hardware_introduction/rdk_x5",permalink:"/rdk_doc/en/Quick_start/hardware_introduction/rdk_x5",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/01_Quick_start/hardware_introduction/rdk_x5.md",tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"tutorialSidebar",previous:{title:"1.1.1 RDK X3",permalink:"/rdk_doc/en/Quick_start/hardware_introduction/rdk_x3"},next:{title:"1.1.3 RDK Ultra",permalink:"/rdk_doc/en/Quick_start/hardware_introduction/rdk_ultra"}},o={},c=[{value:"Interface Overview",id:"interface-overview",level:2},{value:"Power Interface",id:"power-interface",level:2},{value:"Debug Serial Port",id:"debug_uart",level:2},{value:"Ethernet Port",id:"ethernet-port",level:2},{value:"HDMI Display Interface",id:"hdmi_interface",level:2},{value:"USB Interfaces",id:"usb-interfaces",level:2},{value:"Connecting USB Flash Drives",id:"connecting-usb-flash-drives",level:3},{value:"Connecting USB-to-Serial Adapters",id:"connecting-usb-to-serial-adapters",level:3},{value:"Connecting USB Cameras",id:"connecting-usb-cameras",level:3},{value:"MIPI Camera Interface",id:"mipi_port",level:2},{value:"LCD Display Interface",id:"lcd-display-interface",level:2},{value:"Micro SD Interface",id:"micro-sd-interface",level:2},{value:"Wi-Fi Antenna Interface",id:"wi-fi-antenna-interface",level:2},{value:"CAN FD Interface",id:"can-fd-interface",level:2},{value:"40PIN Interface",id:"40pin-interface",level:2}];function l(e){const n={a:"a",admonition:"admonition",br:"br",code:"code",h1:"h1",h2:"h2",h3:"h3",hr:"hr",img:"img",li:"li",p:"p",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",ul:"ul",...(0,i.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(n.h1,{id:"112-rdk-x5",children:"1.1.2 RDK X5"}),"\n",(0,r.jsx)(n.h2,{id:"interface-overview",children:"Interface Overview"}),"\n",(0,r.jsx)(n.p,{children:"RDK X5 provides various functional interfaces, including Ethernet, USB, camera, LCD, HDMI, CAN FD, and 40PIN, enabling users to develop and test applications such as image multimedia and deep learning algorithms. The layout of the development board's interfaces is shown below:"}),"\n",(0,r.jsx)(n.p,{children:(0,r.jsx)(n.img,{alt:"RDK_X5_interface",src:t(92216).A+"",width:"1920",height:"1356"})}),"\n",(0,r.jsxs)(n.table,{children:[(0,r.jsx)(n.thead,{children:(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.th,{children:"No."}),(0,r.jsx)(n.th,{children:"Function"}),(0,r.jsx)(n.th,{children:"No."}),(0,r.jsx)(n.th,{children:"Function"}),(0,r.jsx)(n.th,{children:"No."}),(0,r.jsx)(n.th,{children:"Function"})]})}),(0,r.jsxs)(n.tbody,{children:[(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"1"}),(0,r.jsx)(n.td,{children:"Power Interface (USB Type C)"}),(0,r.jsx)(n.td,{children:"2"}),(0,r.jsx)(n.td,{children:"RTC Battery Interface"}),(0,r.jsx)(n.td,{children:"3"}),(0,r.jsx)(n.td,{children:"Easy Connect Port (USB Type C)"})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"4"}),(0,r.jsx)(n.td,{children:"Debug Serial Port (Micro USB)"}),(0,r.jsx)(n.td,{children:"5"}),(0,r.jsx)(n.td,{children:"Dual MIPI Camera Ports"}),(0,r.jsx)(n.td,{children:"6"}),(0,r.jsx)(n.td,{children:"Gigabit Ethernet Port with PoE"})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"7"}),(0,r.jsx)(n.td,{children:"4 USB 3.0 Type A Ports"}),(0,r.jsx)(n.td,{children:"8"}),(0,r.jsx)(n.td,{children:"High-Speed CAN FD Interface"}),(0,r.jsx)(n.td,{children:"9"}),(0,r.jsx)(n.td,{children:"40PIN Interface"})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"10"}),(0,r.jsx)(n.td,{children:"HDMI Display Interface"}),(0,r.jsx)(n.td,{children:"11"}),(0,r.jsx)(n.td,{children:"Multi-standard Headphone Jack"}),(0,r.jsx)(n.td,{children:"12"}),(0,r.jsx)(n.td,{children:"Onboard Wi-Fi Antenna"})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"13"}),(0,r.jsx)(n.td,{children:"TF Card Interface (Bottom)"}),(0,r.jsx)(n.td,{children:"14"}),(0,r.jsx)(n.td,{children:"LCD Display Interface (MIPI DSI)"}),(0,r.jsx)(n.td,{}),(0,r.jsx)(n.td,{})]})]})]}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"power-interface",children:"Power Interface"}),"\n",(0,r.jsxs)(n.p,{children:["The development board provides a USB Type C interface (No. 1) as the power interface. It requires a ",(0,r.jsx)(n.strong,{children:"5V/5A"})," power adapter for supplying power to the board. Once the adapter is connected, the ",(0,r.jsx)(n.strong,{children:"green power indicator"})," and the ",(0,r.jsx)(n.strong,{children:"orange indicator"})," will light up, indicating normal power supply."]}),"\n",(0,r.jsx)(n.admonition,{type:"caution",children:(0,r.jsxs)(n.p,{children:["Do not use a computer USB port to power the board. Insufficient power may cause ",(0,r.jsx)(n.strong,{children:"abnormal shutdown or repeated reboots"}),"."]})}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"debug_uart",children:"Debug Serial Port"}),"\n",(0,r.jsx)(n.p,{children:"The development board includes a debug serial port (No. 4) for serial login and debugging functions. Configure the parameters in the serial tool on your computer as follows:"}),"\n",(0,r.jsxs)(n.ul,{children:["\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Baud rate"}),": 115200"]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Data bits"}),": 8"]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Parity"}),": None"]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Stop bits"}),": 1"]}),"\n",(0,r.jsxs)(n.li,{children:[(0,r.jsx)(n.strong,{children:"Flow control"}),": None"]}),"\n"]}),"\n",(0,r.jsxs)(n.p,{children:["To connect, use a Micro USB cable to link the board's interface No. 4 to your PC.",(0,r.jsx)(n.br,{}),"\n","For first-time use, you may need to install the CH340 driver on your computer. Search for ",(0,r.jsx)(n.code,{children:"CH340\u4e32\u53e3\u9a71\u52a8"})," to download and install it."]}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"ethernet-port",children:"Ethernet Port"}),"\n",(0,r.jsxs)(n.p,{children:["The development board features a Gigabit Ethernet port (No. 6) supporting 1000BASE-T and 100BASE-T standards. By default, it uses a static IP configuration with the address ",(0,r.jsx)(n.code,{children:"192.168.127.10"}),".",(0,r.jsx)(n.br,{}),"\n","To verify the board's IP address, log in via the serial port and use the ",(0,r.jsx)(n.code,{children:"ifconfig"})," command to check the configuration of the ",(0,r.jsx)(n.code,{children:"eth0"})," interface."]}),"\n",(0,r.jsx)(n.p,{children:"Additionally, this port supports PoE (Power over Ethernet), allowing simultaneous data and power transmission via a single Ethernet cable for easier installation."}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"hdmi_interface",children:"HDMI Display Interface"}),"\n",(0,r.jsxs)(n.p,{children:["The development board includes an HDMI display interface (No. 10) that supports a maximum resolution of 1080P. Using the HDMI interface, the board can output the Ubuntu system desktop (on the Ubuntu Server version, it displays the logo).",(0,r.jsx)(n.br,{}),"\n","The HDMI interface also supports real-time display of camera and network stream images."]}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"usb-interfaces",children:"USB Interfaces"}),"\n",(0,r.jsx)(n.p,{children:"The development board supports multiple USB interface extensions to accommodate various USB devices. Details are as follows:"}),"\n",(0,r.jsxs)(n.table,{children:[(0,r.jsx)(n.thead,{children:(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.th,{children:"Interface Type"}),(0,r.jsx)(n.th,{children:"Interface No."}),(0,r.jsx)(n.th,{children:"Quantity"}),(0,r.jsx)(n.th,{children:"Description"})]})}),(0,r.jsxs)(n.tbody,{children:[(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"USB 3.0 Type C"}),(0,r.jsx)(n.td,{children:"No. 3"}),(0,r.jsx)(n.td,{children:"1 port"}),(0,r.jsx)(n.td,{children:"USB Device mode for ADB, Fastboot, system flashing, etc."})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"USB 3.0 Type A"}),(0,r.jsx)(n.td,{children:"No. 7"}),(0,r.jsx)(n.td,{children:"4 ports"}),(0,r.jsx)(n.td,{children:"USB Host mode for connecting USB 3.0 peripherals"})]})]})]}),"\n",(0,r.jsx)(n.h3,{id:"connecting-usb-flash-drives",children:"Connecting USB Flash Drives"}),"\n",(0,r.jsxs)(n.p,{children:["The USB Type A ports (No. 7) support USB flash drives, which will be automatically detected and mounted. The default mount directory is ",(0,r.jsx)(n.code,{children:"/media/sda1"}),"."]}),"\n",(0,r.jsx)(n.h3,{id:"connecting-usb-to-serial-adapters",children:"Connecting USB-to-Serial Adapters"}),"\n",(0,r.jsxs)(n.p,{children:["The USB Type A ports (No. 7) support USB-to-serial adapters, which will be automatically detected and create device nodes such as ",(0,r.jsx)(n.code,{children:"/dev/ttyUSB*"})," or ",(0,r.jsx)(n.code,{children:"/dev/ttyACM*"})," (where the asterisk represents a number starting from 0). Refer to the ",(0,r.jsx)(n.a,{href:"/rdk_doc/en/Basic_Application/03_40pin_user_guide/uart#40pin_uart_usage",children:"\u4f7f\u7528\u4e32\u53e3"})," section for details."]}),"\n",(0,r.jsx)(n.h3,{id:"connecting-usb-cameras",children:"Connecting USB Cameras"}),"\n",(0,r.jsxs)(n.p,{children:["The USB Type A ports support USB cameras, which will be automatically detected and create device nodes such as ",(0,r.jsx)(n.code,{children:"/dev/video0"}),"."]}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"mipi_port",children:"MIPI Camera Interface"}),"\n",(0,r.jsx)(n.p,{children:"The development board provides two MIPI CSI interfaces (No. 5) for connecting up to two MIPI cameras, including stereo cameras. Compatible camera modules and specifications are as follows:"}),"\n",(0,r.jsxs)(n.table,{children:[(0,r.jsx)(n.thead,{children:(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.th,{children:"No."}),(0,r.jsx)(n.th,{children:"Sensor"}),(0,r.jsx)(n.th,{children:"Resolution"}),(0,r.jsx)(n.th,{children:"FOV"}),(0,r.jsx)(n.th,{children:"I2C Device Address"})]})}),(0,r.jsxs)(n.tbody,{children:[(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"1"}),(0,r.jsx)(n.td,{children:"IMX219"}),(0,r.jsx)(n.td,{children:"8 MP"}),(0,r.jsx)(n.td,{}),(0,r.jsx)(n.td,{})]}),(0,r.jsxs)(n.tr,{children:[(0,r.jsx)(n.td,{children:"2"}),(0,r.jsx)(n.td,{children:"OV5647"}),(0,r.jsx)(n.td,{children:"5 MP"}),(0,r.jsx)(n.td,{}),(0,r.jsx)(n.td,{})]})]})]}),"\n",(0,r.jsxs)(n.p,{children:["Connect the camera module to the board using an FPC cable with the blue side facing upwards.",(0,r.jsx)(n.br,{}),"\n","After installation, use the ",(0,r.jsx)(n.code,{children:"i2cdetect"})," command to check if the I2C address of the module can be detected."]}),"\n",(0,r.jsx)(n.admonition,{type:"caution",children:(0,r.jsx)(n.p,{children:"Important: Do not connect or disconnect the camera while the board is powered on, as this may damage the camera module."})}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"lcd-display-interface",children:"LCD Display Interface"}),"\n",(0,r.jsx)(n.p,{children:"The RDK X5 provides an LCD display interface (MIPI DSI, No. 14) that supports LCD screens. This interface uses a 15-pin FPC connector and is compatible with several Raspberry Pi LCD displays."}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"micro-sd-interface",children:"Micro SD Interface"}),"\n",(0,r.jsx)(n.p,{children:"The development board includes a Micro SD card interface (No. 13). It is recommended to use a card with at least 8GB of storage to meet the installation requirements of Ubuntu and related packages."}),"\n",(0,r.jsx)(n.admonition,{type:"caution",children:(0,r.jsx)(n.p,{children:"Do not hot-swap the TF card during use, as it may cause system abnormalities or file system corruption."})}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"wi-fi-antenna-interface",children:"Wi-Fi Antenna Interface"}),"\n",(0,r.jsx)(n.p,{children:"The board supports both onboard and external antennas for wireless networking. The onboard antenna is sufficient for most scenarios. If the board is enclosed in a metal casing, connect an external antenna to the port near interface No. 12 to enhance signal strength."}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"can-fd-interface",children:"CAN FD Interface"}),"\n",(0,r.jsxs)(n.p,{children:["The RDK X5 provides a CAN FD interface for CAN and CAN FD communication. Refer to the ",(0,r.jsx)(n.a,{href:"/rdk_doc/en/Advanced_development/hardware_development/rdk_x5/can",children:"CAN\u4f7f\u7528"})," section for details."]}),"\n",(0,r.jsx)(n.hr,{}),"\n",(0,r.jsx)(n.h2,{id:"40pin-interface",children:"40PIN Interface"}),"\n",(0,r.jsx)(n.p,{children:"The development board includes a 40PIN interface with IO signals designed at 3.3V. The pin definition is compatible with Raspberry Pi and similar products. For detailed pin definitions and multiplexing information, refer to the hardware development section."}),"\n",(0,r.jsx)(n.hr,{})]})}function h(e={}){const{wrapper:n}={...(0,i.R)(),...e.components};return n?(0,r.jsx)(n,{...e,children:(0,r.jsx)(l,{...e})}):l(e)}},92216:(e,n,t)=>{t.d(n,{A:()=>r});const r=t.p+"assets/images/RDK_X5_interface-fccfc8c221e9eacb4e5d8f4d41b3f9f2.jpg"},28453:(e,n,t)=>{t.d(n,{R:()=>d,x:()=>a});var r=t(96540);const i={},s=r.createContext(i);function d(e){const n=r.useContext(s);return r.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function a(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:d(e.components),r.createElement(s.Provider,{value:n},e.children)}}}]);