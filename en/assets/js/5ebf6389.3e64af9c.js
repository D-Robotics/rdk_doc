"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[677],{90103:(e,s,t)=>{t.r(s),t.d(s,{assets:()=>l,contentTitle:()=>o,default:()=>h,frontMatter:()=>r,metadata:()=>a,toc:()=>d});var n=t(74848),i=t(28453);const r={sidebar_position:3},o="1.2.3 RDK Ultra",a={id:"Quick_start/install_os/rdk_ultra",title:"1.2.3 RDK Ultra",description:"Preparation for Flashing",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/01_Quick_start/install_os/rdk_ultra.md",sourceDirName:"01_Quick_start/install_os",slug:"/Quick_start/install_os/rdk_ultra",permalink:"/rdk_doc/en/Quick_start/install_os/rdk_ultra",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/01_Quick_start/install_os/rdk_ultra.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"tutorialSidebar",previous:{title:"1.2.2 RDK X5",permalink:"/rdk_doc/en/Quick_start/install_os/rdk_x5"},next:{title:"1.3 Getting Started Configuration",permalink:"/rdk_doc/en/Quick_start/configuration_wizard"}},l={},d=[{value:"Preparation for Flashing",id:"preparation-for-flashing",level:2},{value:"<strong>Power Supply</strong>",id:"power-supply",level:3},{value:"<strong>Storage</strong>",id:"storage",level:3},{value:"<strong>Display</strong>",id:"display",level:3},{value:"<strong>Network Connection</strong>",id:"network-connection",level:3},{value:"System Flashing",id:"system-flashing",level:2},{value:"Image Download",id:"img_download",level:3},{value:"System Flashing",id:"system-flashing-1",level:3},{value:"System Boot",id:"system-boot",level:3},{value:"<strong>FAQ</strong>",id:"faq",level:2},{value:"<strong>Precautions</strong>",id:"precautions",level:3}];function c(e){const s={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",h3:"h3",hr:"hr",img:"img",li:"li",ol:"ol",p:"p",strong:"strong",ul:"ul",...(0,i.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsx)(s.h1,{id:"123-rdk-ultra",children:"1.2.3 RDK Ultra"}),"\n",(0,n.jsx)(s.h2,{id:"preparation-for-flashing",children:"Preparation for Flashing"}),"\n",(0,n.jsx)(s.h3,{id:"power-supply",children:(0,n.jsx)(s.strong,{children:"Power Supply"})}),"\n",(0,n.jsxs)(s.p,{children:["The RDK Ultra development board is powered via a DC interface. It is recommended to use the power adapter included in the ",(0,n.jsx)(s.code,{children:"official kit"})," or a power adapter with at least ",(0,n.jsx)(s.strong,{children:"12V/5A"})," output."]}),"\n",(0,n.jsxs)(s.admonition,{type:"caution",children:[(0,n.jsxs)(s.p,{children:["Do not use a computer USB port to power the board. Insufficient power may cause ",(0,n.jsx)(s.strong,{children:"abnormal shutdowns or repeated reboots"}),"."]}),(0,n.jsxs)(s.p,{children:["For more troubleshooting tips, refer to the ",(0,n.jsx)(s.a,{href:"/rdk_doc/en/FAQ/hardware_and_system",children:"FAQ"})," section."]})]}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"storage",children:(0,n.jsx)(s.strong,{children:"Storage"})}),"\n",(0,n.jsx)(s.p,{children:"The RDK Ultra board includes 64GB of onboard eMMC storage, so no additional storage card is required."}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"display",children:(0,n.jsx)(s.strong,{children:"Display"})}),"\n",(0,n.jsx)(s.p,{children:"The RDK Ultra development board supports an HDMI display interface. Connect the board to a monitor using an HDMI cable to enable graphical desktop display."}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"network-connection",children:(0,n.jsx)(s.strong,{children:"Network Connection"})}),"\n",(0,n.jsx)(s.p,{children:"The RDK Ultra development board supports both Ethernet and Wi-Fi network interfaces. Users can use either interface to connect to a network."}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h2,{id:"system-flashing",children:"System Flashing"}),"\n",(0,n.jsx)(s.p,{children:"The RDK Ultra currently provides an Ubuntu 20.04 system image, supporting graphical interaction with the Desktop version."}),"\n",(0,n.jsx)(s.admonition,{title:"Note",type:"info",children:(0,n.jsxs)(s.p,{children:["The ",(0,n.jsx)(s.strong,{children:"RDK Ultra"})," comes with a pre-installed test system image. To ensure the use of the latest version, ",(0,n.jsx)("font",{color:"Red",children:"it is recommended to follow this guide to flash the latest system image"}),"."]})}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"img_download",children:"Image Download"}),"\n",(0,n.jsxs)(s.p,{children:["Click ",(0,n.jsx)(s.a,{href:"https://archive.d-robotics.cc/downloads/os_images",children:(0,n.jsx)(s.strong,{children:"Download Image"})}),", select the ",(0,n.jsx)(s.code,{children:"rdk_ultra"})," directory, and choose the appropriate version to go to the file download page. For example, to download version 1.0.0 of the system image:"]}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-20230510143353330",src:t(58320).A+"",width:"1181",height:"196"})}),"\n",(0,n.jsxs)(s.p,{children:["After downloading, extract the Ubuntu system image file, such as ",(0,n.jsx)(s.code,{children:"ubuntu-preinstalled-desktop-arm64-rdkultra.img"}),"."]}),"\n",(0,n.jsx)(s.admonition,{type:"tip",children:(0,n.jsxs)(s.ul,{children:["\n",(0,n.jsxs)(s.li,{children:[(0,n.jsx)(s.strong,{children:"Desktop"}),": Ubuntu system with a desktop interface, enabling operation with an external screen and mouse."]}),"\n",(0,n.jsxs)(s.li,{children:[(0,n.jsx)(s.strong,{children:"Server"}),": Ubuntu system without a desktop, accessible via serial or remote network connection."]}),"\n"]})}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"system-flashing-1",children:"System Flashing"}),"\n",(0,n.jsxs)(s.p,{children:["When flashing the Ubuntu system on the RDK Ultra development kit, use the D-Robotics ",(0,n.jsx)(s.code,{children:"hbupdate"})," flashing tool. The tool supports both Windows and Linux versions, named ",(0,n.jsx)(s.code,{children:"hbupdate_win64"})," and ",(0,n.jsx)(s.code,{children:"hbupdate_linux"}),", respectively. Download the tool from ",(0,n.jsx)(s.a,{href:"https://archive.d-robotics.cc/downloads/hbupdate/",children:"hbupdate"}),"."]}),"\n",(0,n.jsx)(s.admonition,{title:"Notes",type:"tip",children:(0,n.jsxs)(s.ul,{children:["\n",(0,n.jsxs)(s.li,{children:["Extract the tool archive, ensuring the extraction path does not contain ",(0,n.jsx)(s.strong,{children:"spaces, Chinese characters, or special symbols"}),"."]}),"\n",(0,n.jsxs)(s.li,{children:["The tool communicates with the RDK Ultra via the Ethernet port. For optimal flashing speed, ensure that the ",(0,n.jsx)(s.strong,{children:"PC supports a Gigabit Ethernet port and uses a direct connection"}),"."]}),"\n",(0,n.jsxs)(s.li,{children:["Configure the PC's network settings to use a ",(0,n.jsx)(s.strong,{children:"static IP address"})," as follows:","\n",(0,n.jsxs)(s.ul,{children:["\n",(0,n.jsxs)(s.li,{children:[(0,n.jsx)(s.strong,{children:"IP"}),": 192.168.1.195"]}),"\n",(0,n.jsxs)(s.li,{children:[(0,n.jsx)(s.strong,{children:"Netmask"}),": 255.255.255.0"]}),"\n",(0,n.jsxs)(s.li,{children:[(0,n.jsx)(s.strong,{children:"Gateway"}),": 192.168.1.1"]}),"\n"]}),"\n"]}),"\n"]})}),"\n",(0,n.jsxs)(s.ol,{children:["\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"Use an Ethernet cable to connect the RDK Ultra directly to the PC, and ensure the connection is reachable via ping."}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsxs)(s.p,{children:["Short-circuit the ",(0,n.jsx)(s.code,{children:"FC_REC"})," and ",(0,n.jsx)(s.code,{children:"GND"})," signals on the function control interface (Interface 10)."]}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-ultra-fc-rec",src:t(42056).A+"",width:"3014",height:"1867"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsxs)(s.p,{children:["Run the ",(0,n.jsx)(s.code,{children:"hbupdate"})," tool, select the development board model as ",(0,n.jsx)(s.code,{children:"RDK_ULTRA"})," (required)."]}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system1",src:t(20968).A+"",width:"2148",height:"686"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsxs)(s.p,{children:["Click the ",(0,n.jsx)(s.code,{children:"Browse"})," button to select the system image file to be flashed (required)."]}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system2",src:t(31895).A+"",width:"2798",height:"900"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsxs)(s.p,{children:["Click the ",(0,n.jsx)(s.code,{children:"Start"})," button to begin flashing. After confirming the prompts, click the ",(0,n.jsx)(s.code,{children:"OK"})," button:"]}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system3",src:t(33640).A+"",width:"470",height:"143"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"When the tool displays the following log, it indicates that the flashing process has started. The process duration depends on the network transmission speed. Please wait patiently."}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system4",src:t(97193).A+"",width:"1804",height:"1002"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"Wait for the tool to complete the flashing process and check the results:"}),"\n"]}),"\n"]}),"\n",(0,n.jsxs)(s.ul,{children:["\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"If successful, the tool will display the following message:"}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system6",src:t(13475).A+"",width:"1070",height:"262"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"If an error occurs, check if steps 1\u20133 were performed correctly:"}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system7",src:t(12250).A+"",width:"1072",height:"260"})}),"\n"]}),"\n",(0,n.jsxs)(s.li,{children:["\n",(0,n.jsx)(s.p,{children:"If an error occurs indicating slow network speed, use a higher-performance PC to retry the process:"}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-flash-system8",src:t(28989).A+"",width:"1428",height:"812"})}),"\n"]}),"\n"]}),"\n",(0,n.jsx)(s.admonition,{type:"caution",children:(0,n.jsx)(s.p,{children:"If the flashing process is interrupted, repeat the steps above to retry."})}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h3,{id:"system-boot",children:"System Boot"}),"\n",(0,n.jsxs)(s.p,{children:["First, ensure the board is powered off, then remove the short circuit between the ",(0,n.jsx)(s.code,{children:"FC_REC"})," and ",(0,n.jsx)(s.code,{children:"GND"})," signals on the function control interface (Interface 10). Connect the board to a monitor using an HDMI cable, and then power on the board."]}),"\n",(0,n.jsx)(s.p,{children:"During the first boot, the system will perform default environment configuration, which takes approximately 45 seconds. After the configuration is complete, the Ubuntu system desktop will be displayed on the monitor."}),"\n",(0,n.jsx)(s.p,{children:"Once the Ubuntu Desktop system boots, the system desktop will be displayed via the HDMI interface, as shown below:"}),"\n",(0,n.jsx)(s.p,{children:(0,n.jsx)(s.img,{alt:"image-desktop_display.jpg",src:t(56264).A+"",width:"1595",height:"878"})}),"\n",(0,n.jsx)(s.hr,{}),"\n",(0,n.jsx)(s.h2,{id:"faq",children:(0,n.jsx)(s.strong,{children:"FAQ"})}),"\n",(0,n.jsx)(s.h3,{id:"precautions",children:(0,n.jsx)(s.strong,{children:"Precautions"})}),"\n",(0,n.jsxs)(s.ul,{children:["\n",(0,n.jsx)(s.li,{children:"Do not plug or unplug any devices except USB, HDMI, and Ethernet cables while the board is powered on."}),"\n"]}),"\n",(0,n.jsx)(s.admonition,{type:"tip",children:(0,n.jsxs)(s.p,{children:["For more troubleshooting tips, refer to the ",(0,n.jsx)(s.a,{href:"/rdk_doc/en/FAQ/hardware_and_system",children:"FAQ"})," section. You can also visit the ",(0,n.jsx)(s.a,{href:"https://developer.d-robotics.cc/forum",children:"D-Robotics Developer Official Forum"})," for help."]})})]})}function h(e={}){const{wrapper:s}={...(0,i.R)(),...e.components};return s?(0,n.jsx)(s,{...e,children:(0,n.jsx)(c,{...e})}):c(e)}},58320:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/20231010120539-e1d9b4909c6fad6d4351cf3dd1a358ad.png"},56264:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-desktop_display-7b73c6fb12cd3b20372172194a712ec7.jpg"},20968:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system1-1fc55d622600f258028db5eeba5d0400.jpg"},31895:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system2-f0d0c7ec03da6ae3a83ccb4535afedb6.jpg"},97193:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system4-d0fdb0bd29de9da59a6869c62565ca5f.jpg"},13475:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system6-4d933b3d04e88de46637d1a78ed2c595.png"},12250:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system7-12e74260a7e0777b5efe5a60bc453d57.png"},28989:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-rdk-ultra-system8-5ff7388fac90e93cbb4d4f671843511b.jpg"},33640:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-system-download3-dc3c2e1248d80e0203ae2525b96e7819.jpg"},42056:(e,s,t)=>{t.d(s,{A:()=>n});const n=t.p+"assets/images/image-ultra-fc-rec-929492b22074e6a29d11ef513a428ad0.jpg"},28453:(e,s,t)=>{t.d(s,{R:()=>o,x:()=>a});var n=t(96540);const i={},r=n.createContext(i);function o(e){const s=n.useContext(r);return n.useMemo((function(){return"function"==typeof e?e(s):{...s,...e}}),[s,e])}function a(e){let s;return s=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:o(e.components),n.createElement(r.Provider,{value:s},e.children)}}}]);