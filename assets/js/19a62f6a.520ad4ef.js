"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[1639],{12155:(e,n,i)=>{i.r(n),i.d(n,{assets:()=>l,contentTitle:()=>r,default:()=>a,frontMatter:()=>c,metadata:()=>h,toc:()=>o});var d=i(74848),s=i(28453);const c={sidebar_position:4},r="\u663e\u793a\u5c4f\u4f7f\u7528",h={id:"Advanced_development/hardware_development/rdk_x5/display",title:"\u663e\u793a\u5c4f\u4f7f\u7528",description:"RDK X5\u63d0\u4f9b\u4e00\u8defMIPI DSI\u63a5\u53e3\uff0c\u4e00\u8defHDMI\u63a5\u53e3\uff0cdisplay\u53ea\u80fd\u9009\u62e9\u5176\u4e2d\u7684\u4e00\u8def\uff0c\u9ed8\u8ba4\u662fHDMI\u3002",source:"@site/docs/07_Advanced_development/01_hardware_development/rdk_x5/display.md",sourceDirName:"07_Advanced_development/01_hardware_development/rdk_x5",slug:"/Advanced_development/hardware_development/rdk_x5/display",permalink:"/rdk_doc/Advanced_development/hardware_development/rdk_x5/display",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/07_Advanced_development/01_hardware_development/rdk_x5/display.md",tags:[],version:"current",sidebarPosition:4,frontMatter:{sidebar_position:4},sidebar:"tutorialSidebar",previous:{title:"CAN\u4f7f\u7528",permalink:"/rdk_doc/Advanced_development/hardware_development/rdk_x5/can"},next:{title:"7.2. Linux\u5f00\u53d1\u6307\u5357",permalink:"/rdk_doc/linux_development"}},l={},o=[{value:"HDMI",id:"hdmi",level:2},{value:"MIPI DSI",id:"mipi-dsi",level:2},{value:"\u652f\u6301\u5217\u8868",id:"\u652f\u6301\u5217\u8868",level:3},{value:"2.8inch DSI LCD",id:"28inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e",level:4},{value:"3.4inch DSI LCD",id:"34inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-1",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-1",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a",level:4},{value:"4.3inch DSI LCD",id:"43inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-2",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-2",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a-1",level:4},{value:"7inchC DSI LCD",id:"7inchc-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-3",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-3",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a-2",level:4},{value:"7.9inch DSI LCD",id:"79inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-4",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-4",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a-3",level:4},{value:"8inch DSI LCD",id:"8inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-5",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-5",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a-4",level:4},{value:"10.1inch DSI LCD",id:"101inch-dsi-lcd",level:3},{value:"\u786c\u4ef6\u8fde\u63a5",id:"\u786c\u4ef6\u8fde\u63a5-6",level:4},{value:"\u8f6f\u4ef6\u914d\u7f6e",id:"\u8f6f\u4ef6\u914d\u7f6e-6",level:4},{value:"\u6548\u679c\u6f14\u793a",id:"\u6548\u679c\u6f14\u793a-5",level:4}];function t(e){const n={a:"a",code:"code",h1:"h1",h2:"h2",h3:"h3",h4:"h4",img:"img",li:"li",p:"p",pre:"pre",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",ul:"ul",...(0,s.R)(),...e.components};return(0,d.jsxs)(d.Fragment,{children:[(0,d.jsx)(n.h1,{id:"\u663e\u793a\u5c4f\u4f7f\u7528",children:"\u663e\u793a\u5c4f\u4f7f\u7528"}),"\n",(0,d.jsx)(n.p,{children:"RDK X5\u63d0\u4f9b\u4e00\u8defMIPI DSI\u63a5\u53e3\uff0c\u4e00\u8defHDMI\u63a5\u53e3\uff0cdisplay\u53ea\u80fd\u9009\u62e9\u5176\u4e2d\u7684\u4e00\u8def\uff0c\u9ed8\u8ba4\u662fHDMI\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u53ef\u4ee5\u901a\u8fc7\u4ee5\u4e0b\u6307\u4ee4\u5207\u6362HDMI\u8f93\u51fa\uff0c\u548cMIPI DSI\u8f93\u51fa\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u5207\u6362\u6210MIPI DSI"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsx)(n.p,{children:"\u5207\u6362\u6210HMDI"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf\n\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsx)(n.h2,{id:"hdmi",children:"HDMI"}),"\n",(0,d.jsx)(n.p,{children:"RDK X5\u652f\u6301\u7684\u6700\u5927\u5206\u8fa8\u7387\u4e3a1080P60\uff0c\u7cfb\u7edf\u9ed8\u8ba4\u5206\u8fa8\u7387\u4e3a720P60\u3002"}),"\n",(0,d.jsxs)(n.p,{children:["\u9ed8\u8ba4\u5206\u8fa8\u7387\u53ef\u4ee5\u901a\u8fc7\u4fee\u6539",(0,d.jsx)(n.code,{children:"/etc/X11/xorg.conf.d/1-resolution.conf"}),"\u6587\u4ef6\u6765\u5b9e\u73b0\uff1a"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:'Section "Screen"\n    Identifier "Screen0"\n    Device "Device0"\n    Monitor "Monitor0"\n    DefaultDepth 24\n    SubSection "Display"\n        Depth 24\n        Modes "1280x720"\n    EndSubSection\nEndSection\n'})}),"\n",(0,d.jsxs)(n.ul,{children:["\n",(0,d.jsxs)(n.li,{children:["\u5bf9\u63a5\u6700\u5927\u5206\u8fa8\u7387\u5c0f\u4e8e1280x720\u7684\u5c4f\u5e55\uff0c\u53ef\u4ee5\u6839\u636e\u5b9e\u9645\u60c5\u51b5\uff0c\u8c03\u6574",(0,d.jsx)(n.code,{children:"Modes"}),"\uff0c\u53ef\u4ee5\u89e3\u51b3\u51fa\u4e0d\u4e86\u56fe\u7684\u95ee\u9898\u3002"]}),"\n",(0,d.jsxs)(n.li,{children:["\u5bf9\u63a52K\uff0c4K\u5c4f\u5e55\uff0c\u5efa\u8bae\u5c06",(0,d.jsx)(n.code,{children:"Modes"}),"\u8bbe\u7f6e\u4e3a",(0,d.jsx)(n.code,{children:"1920x1080"}),"\uff0c\u8fd9\u53d7\u9650\u4e8eRDK X5 \u7684\u89c6\u9891\u8f6c\u6362\u82af\u7247\u7684\u6027\u80fd\u3002"]}),"\n"]}),"\n",(0,d.jsx)(n.h2,{id:"mipi-dsi",children:"MIPI DSI"}),"\n",(0,d.jsx)(n.p,{children:"RDK X5\u63d0\u4f9b\u4e00\u8defMIPI DSI\u63a5\u53e3\uff0c\u652f\u6301\u591a\u79cdLCD\u5c4f\u5e55\u7684\u63a5\u5165\u3002"}),"\n",(0,d.jsx)(n.h3,{id:"\u652f\u6301\u5217\u8868",children:"\u652f\u6301\u5217\u8868"}),"\n",(0,d.jsxs)(n.table,{children:[(0,d.jsx)(n.thead,{children:(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.th,{children:"\u4f9b\u5e94\u5546"}),(0,d.jsx)(n.th,{children:"\u578b\u53f7"}),(0,d.jsx)(n.th,{children:"\u63cf\u8ff0"}),(0,d.jsx)(n.th,{children:"\u8d2d\u4e70\u94fe\u63a5"}),(0,d.jsx)(n.th,{children:"\u4f7f\u7528\u6307\u5357"})]})}),(0,d.jsxs)(n.tbody,{children:[(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"2.8inch DSI LCD"}),(0,d.jsx)(n.td,{children:"2.8\u5bf8IPS\u5168\u8d34\u5408\u7535\u5bb9\u89e6\u63a7\u5c4f\u5c0f\u5c4f\u5e55480\xd7640\u50cf\u7d20 DSI\u901a\u4fe1"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/2.8inch-DSI-LCD.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#28inch-dsi-lcd",children:"2.8inch DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"3.4inch DSI LCD (C)"}),(0,d.jsx)(n.td,{children:"3.4\u5bf8DSI\u5706\u5f62\u7535\u5bb9\u89e6\u63a7\u5c4f 800\xd7800\u50cf\u7d20 IPS\u663e\u793a\u9762\u677f \u5341\u70b9\u89e6\u63a7"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/3.4inch-DSI-LCD-C.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#34inch-dsi-lcd",children:"3.4inch DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"4.3inch DSI LCD"}),(0,d.jsx)(n.td,{children:"\u6811\u8393\u6d3e4.3\u5bf8\u7535\u5bb9\u89e6\u63a7\u5c4f 800\xd7480 IPS\u5e7f\u89c6\u89d2 MIPI DSI\u63a5\u53e3"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/4.3inch-DSI-LCD.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#43inch-dsi-lcd",children:"4.3inch DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"7inch DSI LCD (C)"}),(0,d.jsx)(n.td,{children:"7\u5bf8IPS\u7535\u5bb9\u89e6\u63a7\u5c4f1024\xd7600\u50cf\u7d20 DSI\u901a\u4fe1"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/7inch-DSI-LCD-C.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#7inchc-dsi-lcd",children:"7inchC DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"7.9inch DSI LCD"}),(0,d.jsx)(n.td,{children:"7.9\u5bf8IPS\u7535\u5bb9\u89e6\u63a7\u5c4f\u8d85\u957f\u5c4f\u5e55400\xd71280\u50cf\u7d20 DSI\u901a\u4fe1"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/7.9inch-DSI-LCD.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#79inch-dsi-lcd",children:"7.9inch DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"8inch DSI LCD (C)"}),(0,d.jsx)(n.td,{children:"\u6811\u8393\u6d3e\u7cfb\u5217\u663e\u793a\u5c4f 8\u5bf8IPS\u7535\u5bb9\u89e6\u63a7\u5c4f 1280\xd7800\u50cf\u7d20 DSI\u901a\u4fe1"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/8inch-DSI-LCD-C.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#8inch-dsi-lcd",children:"8inch DSI LCD"})})]}),(0,d.jsxs)(n.tr,{children:[(0,d.jsx)(n.td,{children:"\u5fae\u96ea"}),(0,d.jsx)(n.td,{children:"10.1inch DSI LCD (C)"}),(0,d.jsx)(n.td,{children:"\u6811\u8393\u6d3e dsi\u63a5\u53e3\u663e\u793a\u5c4f 10.1\u5bf8IPS\u7535\u5bb9\u89e6\u63a7\u5c4f 1280\xd7800\u50cf\u7d20 DSI\u901a\u4fe1"}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"https://www.waveshare.net/shop/10.1inch-DSI-LCD-C.htm",children:"\u8d2d\u4e70\u94fe\u63a5"})}),(0,d.jsx)(n.td,{children:(0,d.jsx)(n.a,{href:"display#101inch-dsi-lcd",children:"10.1inch DSI LCD"})})]})]})]}),"\n",(0,d.jsx)(n.h3,{id:"28inch-dsi-lcd",children:"2.8inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-161007",src:i(75583).A+"",width:"1114",height:"529"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c062.8inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-2_8_inch\n"})}),"\n",(0,d.jsx)(n.h3,{id:"34inch-dsi-lcd",children:"3.4inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-1",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-173409",src:i(9910).A+"",width:"1012",height:"827"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c063.4inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u901a\u8fc74PIN\u9876\u9488\u8fde\u901a5V\u4f9b\u7535\uff0c\u5305\u62ec5V\u548cGND\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-1",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-3_4_inch\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-173446",src:i(16701).A+"",width:"1169",height:"603"})}),"\n",(0,d.jsx)(n.h3,{id:"43inch-dsi-lcd",children:"4.3inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-2",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-144324",src:i(18456).A+"",width:"1110",height:"545"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c064.3inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-2",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-4_3_inch\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a-1",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-144429",src:i(8086).A+"",width:"1039",height:"627"})}),"\n",(0,d.jsx)(n.h3,{id:"7inchc-dsi-lcd",children:"7inchC DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-3",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsxs)(n.p,{children:[(0,d.jsx)(n.img,{alt:"screenshot-20241014-154448",src:i(65364).A+"",width:"970",height:"912"}),"\n",(0,d.jsx)(n.img,{alt:"screenshot-20241014-155331",src:i(82848).A+"",width:"620",height:"829"})]}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c067inchC DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u5e76\u901a\u8fc74PIN\u675c\u90a6\u7ebf\u8fde\u63a55V\u4f9b\u7535\u548cI2C5\u901a\u4fe1\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-3",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-7_0_inchC\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a-2",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-155439",src:i(65683).A+"",width:"1139",height:"680"})}),"\n",(0,d.jsx)(n.h3,{id:"79inch-dsi-lcd",children:"7.9inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-4",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-165628",src:i(18184).A+"",width:"599",height:"801"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c067.9inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u4f7f\u75285V/3A\u7684 type-C \u63a5\u53e3\u7535\u6e90\u4e3a\u5c4f\u5e55\u4f9b\u7535\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-4",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-7_9_inch\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a-3",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-165707",src:i(28932).A+"",width:"619",height:"676"})}),"\n",(0,d.jsx)(n.h3,{id:"8inch-dsi-lcd",children:"8inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-5",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-151754",src:i(63279).A+"",width:"1177",height:"516"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c068inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u4f7f\u75285V/3A\u7684 type-C \u63a5\u53e3\u7535\u6e90\u4e3a\u5c4f\u5e55\u4f9b\u7535\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-5",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-8_0_inch\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a-4",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-152537",src:i(84601).A+"",width:"1107",height:"466"})}),"\n",(0,d.jsx)(n.h3,{id:"101inch-dsi-lcd",children:"10.1inch DSI LCD"}),"\n",(0,d.jsx)(n.h4,{id:"\u786c\u4ef6\u8fde\u63a5-6",children:"\u786c\u4ef6\u8fde\u63a5"}),"\n",(0,d.jsx)(n.p,{children:"\u5c4f\u5e55\u8fde\u63a5\u65b9\u5f0f\u5982\u4e0b\u56fe\u6240\u793a\uff1a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-174402",src:i(29760).A+"",width:"1184",height:"559"})}),"\n",(0,d.jsx)(n.p,{children:"\u7528DSI-Cable-12cm\u7ebf\u6750\uff0c\u5c0610.1inch DSI LCD\u8fde\u63a5\u5230X5 rdk\u4e3b\u677f\u768422PIN DSI1\u63a5\u53e3\u3002"}),"\n",(0,d.jsx)(n.p,{children:"\u4f7f\u75285V/3A\u7684 type-C \u63a5\u53e3\u7535\u6e90\u4e3a\u5c4f\u5e55\u4f9b\u7535\u3002"}),"\n",(0,d.jsx)(n.h4,{id:"\u8f6f\u4ef6\u914d\u7f6e-6",children:"\u8f6f\u4ef6\u914d\u7f6e"}),"\n",(0,d.jsx)(n.p,{children:"1\uff0c\u7531\u4e8eRDK X5 \u7cfb\u7edf\u9ed8\u8ba4\u91c7\u7528HDMI\u8f93\u51fa\uff0c\u9700\u8981\u901a\u8fc7\u547d\u4ee4\u5207\u6362\u5230MIPI DSI\u663e\u793a\u65b9\u5f0f\u3002"}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"mv /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf /etc/X11/xorg.conf.d/xorg_dsi_ignore.conf.disable\nmv /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf.disable /etc/X11/xorg.conf.d/xorg_hdmi_ignore.conf\n"})}),"\n",(0,d.jsxs)(n.p,{children:["\u4e5f\u53ef\u4ee5\u901a\u8fc7srpi-config\u6765\u9009\u62e9\u8f93\u51fa\u65b9\u5f0f\uff0c\u53ef\u4ee5\u53c2\u8003 ",(0,d.jsx)(n.a,{href:"../../../System_configuration/srpi-config#display-options",children:"Dsiplay Chose DSI or HDMI"})," \u7ae0\u8282"]}),"\n",(0,d.jsxs)(n.p,{children:["2\uff0c\u6253\u5f00",(0,d.jsx)(n.code,{children:"/boot/config.txt"}),"\u6587\u4ef6\uff0c\u5728config.txt\u6700\u540e\u52a0\u5165\u4ee5\u4e0b\u4ee3\u7801\uff0c\u4fdd\u5b58\uff0c\u9000\u51fa\uff0c\u91cd\u542f\u7cfb\u7edf"]}),"\n",(0,d.jsx)(n.pre,{children:(0,d.jsx)(n.code,{className:"language-bash",children:"dtoverlay=dsi-waveshare-panel-overlay-10_1_inch\n"})}),"\n",(0,d.jsx)(n.h4,{id:"\u6548\u679c\u6f14\u793a-5",children:"\u6548\u679c\u6f14\u793a"}),"\n",(0,d.jsx)(n.p,{children:(0,d.jsx)(n.img,{alt:"screenshot-20241014-174925",src:i(75956).A+"",width:"1192",height:"530"})})]})}function a(e={}){const{wrapper:n}={...(0,s.R)(),...e.components};return n?(0,d.jsx)(n,{...e,children:(0,d.jsx)(t,{...e})}):t(e)}},18456:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-144324-1db73b4b4a892827faea3272208a7dd6.png"},8086:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-144429-e1f480413720dbdd7c87cf1d5207b4a3.png"},63279:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-151754-b6ebeb2b4f076660cd531762ec19e0ac.png"},84601:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-152537-4095be805bfa270ec11fceb36ab0e4a4.png"},65364:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-154448-eb5921511a5ea202abe813a4fd7b4241.png"},82848:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-155331-052996d8a2e85b9055b08b077d63f609.png"},65683:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-155439-5a1ae3b49f925c34e01fea4c601b644e.png"},75583:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-161007-24edad0779f5875c2931d29f75a9fba8.png"},18184:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-165628-2c1b9bf717d0640cda549246b9dd5327.png"},28932:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-165707-0a78f5b884fe90061854bb56afcf8309.png"},9910:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-173409-94ac00a8473cbb8789fa06db51ea983a.png"},16701:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-173446-dd14a72f19930ce4a8678ed267c2b7f5.png"},29760:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-174402-428819d52cfa91610c62c783975ce64a.png"},75956:(e,n,i)=>{i.d(n,{A:()=>d});const d=i.p+"assets/images/screenshot-20241014-174925-8020ebfe06ec972ea2f119189bdf5ea0.png"},28453:(e,n,i)=>{i.d(n,{R:()=>r,x:()=>h});var d=i(96540);const s={},c=d.createContext(s);function r(e){const n=d.useContext(c);return d.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function h(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(s):e.components||s:r(e.components),d.createElement(c.Provider,{value:n},e.children)}}}]);