"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[8471],{18160:(e,i,n)=>{n.r(i),n.d(i,{assets:()=>l,contentTitle:()=>c,default:()=>j,frontMatter:()=>d,metadata:()=>t,toc:()=>h});var r=n(74848),s=n(28453);const d={sidebar_position:4},c="1.4 \u8fdc\u7a0b\u767b\u5f55",t={id:"Quick_start/remote_login",title:"1.4 \u8fdc\u7a0b\u767b\u5f55",description:"\u672c\u7ae0\u8282\u65e8\u5728\u5411\u9700\u8981\u901a\u8fc7\u4e2a\u4eba\u7535\u8111(PC)\u8fdc\u7a0b\u8bbf\u95ee\u5f00\u53d1\u677f\u7684\u7528\u6237\u4ecb\u7ecd\u5982\u4f55\u901a\u8fc7\u4e32\u53e3\u3001\u7f51\u7edc(VNC\u3001SSH)\u65b9\u5f0f\u8fdb\u884c\u8fdc\u7a0b\u767b\u5f55\u3002",source:"@site/docs/01_Quick_start/remote_login.md",sourceDirName:"01_Quick_start",slug:"/Quick_start/remote_login",permalink:"/rdk_doc/Quick_start/remote_login",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/01_Quick_start/remote_login.md",tags:[],version:"current",sidebarPosition:4,frontMatter:{sidebar_position:4},sidebar:"tutorialSidebar",previous:{title:"1.3 \u5165\u95e8\u914d\u7f6e",permalink:"/rdk_doc/Quick_start/configuration_wizard"},next:{title:"1.5 \u7b97\u6cd5\u4f53\u9a8c",permalink:"/rdk_doc/Quick_start/classification"}},l={},h=[{value:"\u4e32\u53e3\u767b\u5f55",id:"login_uart",level:2},{value:"\u7f51\u7edc\u72b6\u6001\u786e\u8ba4",id:"network_config",level:2},{value:"VNC\u767b\u5f55",id:"vnc\u767b\u5f55",level:2},{value:"SSH\u767b\u5f55",id:"ssh",level:2},{value:"\u7ec8\u7aef\u8f6f\u4ef6",id:"\u7ec8\u7aef\u8f6f\u4ef6",level:3},{value:"\u7535\u8111\u547d\u4ee4\u884c",id:"\u7535\u8111\u547d\u4ee4\u884c",level:3},{value:"\u5c40\u57df\u7f51\u6784\u9020",id:"\u5c40\u57df\u7f51\u6784\u9020",level:2}];function o(e){const i={a:"a",admonition:"admonition",br:"br",code:"code",h1:"h1",h2:"h2",h3:"h3",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",ul:"ul",...(0,s.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(i.h1,{id:"14-\u8fdc\u7a0b\u767b\u5f55",children:"1.4 \u8fdc\u7a0b\u767b\u5f55"}),"\n",(0,r.jsx)(i.p,{children:"\u672c\u7ae0\u8282\u65e8\u5728\u5411\u9700\u8981\u901a\u8fc7\u4e2a\u4eba\u7535\u8111(PC)\u8fdc\u7a0b\u8bbf\u95ee\u5f00\u53d1\u677f\u7684\u7528\u6237\u4ecb\u7ecd\u5982\u4f55\u901a\u8fc7\u4e32\u53e3\u3001\u7f51\u7edc(VNC\u3001SSH)\u65b9\u5f0f\u8fdb\u884c\u8fdc\u7a0b\u767b\u5f55\u3002"}),"\n",(0,r.jsxs)(i.admonition,{type:"tip",children:[(0,r.jsx)(i.p,{children:"\u901a\u8fc7\u7f51\u7edc\u65b9\u5f0f\u8fdc\u7a0b\u767b\u5f55\u524d\uff0c\u5f00\u53d1\u677f\u9700\u8981\u901a\u8fc7\u6709\u7ebf\u4ee5\u592a\u7f51\u6216\u8005\u65e0\u7ebfWiFi\u65b9\u5f0f\u63a5\u5165\u7f51\u7edc\uff0c\u914d\u7f6e\u597d\u5f00\u53d1\u677fIP\u5730\u5740\u3002\u5bf9\u4e8e\u4e24\u79cd\u8fde\u63a5\u65b9\u5f0f\u4e0b\u7684IP\u5730\u5740\u4fe1\u606f\u53ef\u53c2\u8003\u5982\u4e0b\u63cf\u8ff0\uff1a"}),(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsxs)(i.li,{children:["\u6709\u7ebf\u4ee5\u592a\u7f51\uff1a\u5f00\u53d1\u677f\u9ed8\u8ba4\u91c7\u7528\u9759\u6001IP\u6a21\u5f0f\uff0cIP\u5730\u5740\u4e3a",(0,r.jsx)(i.code,{children:"192.168.127.10"}),"\uff0c\u63a9\u7801",(0,r.jsx)(i.code,{children:"255.255.255.0"}),"\uff0c\u7f51\u5173 ",(0,r.jsx)(i.code,{children:"192.168.127.1"})]}),"\n",(0,r.jsxs)(i.li,{children:["\u65e0\u7ebfWiFi\uff1a\u5f00\u53d1\u677fIP\u5730\u5740\u4e00\u822c\u7531\u8def\u7531\u5668\u5206\u914d\uff0c\u53ef\u5728\u8bbe\u5907\u547d\u4ee4\u884c\u4e2d\u901a\u8fc7",(0,r.jsx)(i.code,{children:"ifconfig"}),"\u547d\u4ee4\u67e5\u770bwlan0\u7f51\u7edc\u7684IP\u5730\u5740"]}),"\n"]})]}),"\n",(0,r.jsx)(i.h2,{id:"login_uart",children:"\u4e32\u53e3\u767b\u5f55"}),"\n",(0,r.jsxs)(i.p,{children:["Video: ",(0,r.jsx)(i.a,{href:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=2",children:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=2"})]}),"\n",(0,r.jsx)(i.p,{children:"\u5728\u4f7f\u7528\u4e32\u53e3\u767b\u5f55\u524d\uff0c\u9700\u8981\u786e\u8ba4\u5f00\u53d1\u677f\u4e32\u53e3\u7ebf\u8ddf\u7535\u8111\u6b63\u786e\u8fde\u63a5\uff0c\u8fde\u63a5\u65b9\u6cd5\u53ef\u53c2\u8003\u5bf9\u5e94\u5f00\u53d1\u677f\u7684\u8c03\u8bd5\u4e32\u53e3\u7ae0\u8282\uff1a"}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsx)(i.li,{children:(0,r.jsx)(i.a,{href:"/rdk_doc/Quick_start/hardware_introduction/rdk_ultra#debug_uart",children:"rdk_ultra \u8c03\u8bd5\u4e32\u53e3\u7ae0\u8282"})}),"\n",(0,r.jsx)(i.li,{children:(0,r.jsx)(i.a,{href:"/rdk_doc/Quick_start/hardware_introduction/rdk_x3#debug_uart",children:"rdk_x3 \u8c03\u8bd5\u4e32\u53e3\u7ae0\u8282"})}),"\n",(0,r.jsx)(i.li,{children:(0,r.jsx)(i.a,{href:"/rdk_doc/Quick_start/hardware_introduction/rdk_x5#debug_uart",children:"rdk_x5 \u8c03\u8bd5\u4e32\u53e3\u7ae0\u8282"})}),"\n"]}),"\n",(0,r.jsxs)(i.p,{children:["\u4e32\u53e3\u767b\u5f55\u9700\u8981\u501f\u52a9PC\u7ec8\u7aef\u5de5\u5177\uff0c\u76ee\u524d\u5e38\u7528\u7684\u5de5\u5177\u6709",(0,r.jsx)(i.code,{children:"Putty"}),"\u3001",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u7b49\uff0c\u7528\u6237\u53ef\u6839\u636e\u81ea\u8eab\u4f7f\u7528\u4e60\u60ef\u6765\u9009\u62e9\u3002\u4e0d\u540c\u5de5\u5177\u7684\u7aef\u53e3\u914d\u7f6e\u6d41\u7a0b\u57fa\u672c\u7c7b\u4f3c\uff0c\u4e0b\u9762\u4ee5",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u4e3a\u4f8b\uff0c\u4ecb\u7ecd\u65b0\u5efa\u4e32\u53e3\u8fde\u63a5\u8fc7\u7a0b\uff1a"]}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsxs)(i.li,{children:["\u5f53\u4e32\u53e3USB\u8f6c\u63a5\u677f\u9996\u6b21\u63d2\u5165\u7535\u8111\u65f6\uff0c\u9700\u8981\u5b89\u88c5\u4e32\u53e3\u9a71\u52a8\u3002\u9a71\u52a8\u7a0b\u5e8f\u53ef\u4ece\u8d44\u6e90\u4e2d\u5fc3\u7684",(0,r.jsx)(i.a,{href:"https://developer.d-robotics.cc/resource",children:"\u5de5\u5177\u5b50\u680f\u76ee"}),"\u83b7\u53d6\u3002\u9a71\u52a8\u5b89\u88c5\u5b8c\u6210\u540e\uff0c\u8bbe\u5907\u7ba1\u7406\u5668\u53ef\u6b63\u5e38\u8bc6\u522b\u4e32\u53e3\u677f\u7aef\u53e3\uff0c\u5982\u4e0b\u56fe\uff1a"]}),"\n"]}),"\n",(0,r.jsx)(i.p,{children:(0,r.jsx)(i.img,{alt:"image-20220416105939067",src:n(99330).A+"",width:"411",height:"62"})}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u6253\u5f00",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u5de5\u5177\uff0c\u70b9\u51fb",(0,r.jsx)(i.code,{children:"Session"}),"\uff0c\u7136\u540e\u9009\u62e9",(0,r.jsx)(i.code,{children:"Serial"})]}),"\n"]}),"\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u914d\u7f6e\u7aef\u53e3\u53f7\uff0c\u4f8b\u5982",(0,r.jsx)(i.code,{children:"COM3"}),"\uff0c\u5b9e\u9645\u4f7f\u7528\u7684\u4e32\u53e3\u53f7\u4ee5PC\u8bc6\u522b\u5230\u7684\u4e32\u53e3\u53f7\u4e3a\u51c6"]}),"\n"]}),"\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsx)(i.p,{children:"\u8bbe\u7f6e\u4e32\u53e3\u914d\u7f6e\u53c2\u6570\uff0c\u5982\u4e0b\uff1a"}),"\n",(0,r.jsxs)(i.table,{children:[(0,r.jsx)(i.thead,{children:(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.th,{children:"\u914d\u7f6e\u9879"}),(0,r.jsx)(i.th,{children:"\u53c2\u6570\u503c"})]})}),(0,r.jsxs)(i.tbody,{children:[(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"\u6ce2\u7279\u7387\uff08Baud rate\uff09"}),(0,r.jsx)(i.td,{children:"RDK X3 \uff08921600\uff09\uff0cRDK X5 \uff08115200\uff09"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"\u6570\u636e\u4f4d\uff08Data bits\uff09"}),(0,r.jsx)(i.td,{children:"8"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"\u5947\u5076\u6821\u9a8c\uff08Parity\uff09"}),(0,r.jsx)(i.td,{children:"None"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"\u505c\u6b62\u4f4d\uff08Stop bits\uff09"}),(0,r.jsx)(i.td,{children:"1"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"\u6d41\u63a7\uff08Flow Control\uff09"}),(0,r.jsx)(i.td,{children:"\u65e0"})]})]})]}),"\n"]}),"\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u70b9\u51fb",(0,r.jsx)(i.code,{children:"OK"}),"\uff0c\u8f93\u5165\u7528\u6237\u540d\uff1a",(0,r.jsx)(i.code,{children:"root"}),"\u3001\u5bc6\u7801\uff1a",(0,r.jsx)(i.code,{children:"root"}),"\u767b\u5f55\u8bbe\u5907",(0,r.jsx)(i.br,{}),"\n",(0,r.jsx)(i.img,{alt:"image-Uart-Login",src:n(32350).A+"",width:"1073",height:"621"})]}),"\n"]}),"\n"]}),"\n",(0,r.jsxs)(i.p,{children:["\u6b64\u65f6\uff0c\u53ef\u4f7f\u7528",(0,r.jsx)(i.code,{children:"ifconfig"}),"\u547d\u4ee4\u67e5\u8be2\u5f00\u53d1\u677fIP\u5730\u5740\uff0c\u5176\u4e2deth0\u3001wlan0\u5206\u522b\u4ee3\u8868\u6709\u7ebf\u3001\u65e0\u7ebf\u7f51\u7edc\uff1a"]}),"\n",(0,r.jsx)(i.pre,{children:(0,r.jsx)(i.code,{className:"language-bash",children:"root@ubuntu:~# ifconfig\neth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500\n        inet 192.168.127.10  netmask 255.255.255.0  broadcast 192.168.1.255\n        inet6 fe80::211:22ff:feaa:7637  prefixlen 64  scopeid 0x20<link>\n        ether 00:11:22:aa:76:37  txqueuelen 1000  (Ethernet)\n        RX packets 767  bytes 54006 (54.0 KB)\n        RX errors 0  dropped 0  overruns 0  frame 0\n        TX packets 5766  bytes 246466 (246.4 KB)\n        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0\n        device interrupt 43  base 0xa000  \n\nlo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536\n        inet 127.0.0.1  netmask 255.0.0.0\n        inet6 ::1  prefixlen 128  scopeid 0x10<host>\n        loop  txqueuelen 1000  (Local Loopback)\n        RX packets 3847  bytes 339115 (339.1 KB)\n        RX errors 0  dropped 0  overruns 0  frame 0\n        TX packets 3847  bytes 339115 (339.1 KB)\n        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0\n\nwlan0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500\n        ether 08:e9:f6:ae:f8:8a  txqueuelen 1000  (Ethernet)\n        RX packets 0  bytes 0 (0.0 B)\n        RX errors 0  dropped 0  overruns 0  frame 0\n        TX packets 0  bytes 0 (0.0 B)\n        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0\n"})}),"\n",(0,r.jsx)(i.h2,{id:"network_config",children:"\u7f51\u7edc\u72b6\u6001\u786e\u8ba4"}),"\n",(0,r.jsxs)(i.p,{children:["Video: ",(0,r.jsx)(i.a,{href:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=3",children:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=3"})]}),"\n",(0,r.jsxs)(i.p,{children:["\u5728\u4f7f\u7528\u8fdc\u7a0b\u767b\u5f55\u524d\uff0c\u9700\u8981\u786e\u4fdd\u7535\u8111\u3001\u5f00\u53d1\u677f\u7f51\u7edc\u901a\u4fe1\u6b63\u5e38\uff0c\u5982\u65e0\u6cd5",(0,r.jsx)(i.code,{children:"ping"}),"\u901a\uff0c\u9700\u6309\u5982\u4e0b\u6b65\u9aa4\u8fdb\u884c\u786e\u8ba4\uff1a"]}),"\n",(0,r.jsxs)(i.admonition,{type:"tip",children:[(0,r.jsx)(i.p,{children:"\u5404\u4e2a\u7248\u672c\u7684\u955c\u50cf\u5bf9\u5e94\u7684IP\u5730\u5740\u4e3a\uff1a"}),(0,r.jsxs)(i.table,{children:[(0,r.jsx)(i.thead,{children:(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.th,{children:"\u677f\u5361\u7cfb\u5217"}),(0,r.jsx)(i.th,{children:"\u955c\u50cf\u7248\u672c"}),(0,r.jsx)(i.th,{children:"\u7f51\u53e3IP\u5730\u5740"})]})}),(0,r.jsxs)(i.tbody,{children:[(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"X3"}),(0,r.jsx)(i.td,{children:"\u5c0f\u4e8e\u7b49\u4e8e2.0.0"}),(0,r.jsx)(i.td,{children:"192.168.1.10/24"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"X3"}),(0,r.jsx)(i.td,{children:"\u5927\u4e8e\u7b49\u4e8e2.1.0"}),(0,r.jsx)(i.td,{children:"192.168.127.10/24"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"X5"}),(0,r.jsx)(i.td,{children:"3.0.0"}),(0,r.jsx)(i.td,{children:"192.168.127.10/24"})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:(0,r.jsx)(i.strong,{children:"\u677f\u5361\u7cfb\u5217"})}),(0,r.jsx)(i.td,{children:(0,r.jsx)(i.strong,{children:"\u955c\u50cf\u7248\u672c"})}),(0,r.jsx)(i.td,{children:(0,r.jsx)(i.strong,{children:"\u95ea\u8fde\u53e3(USB Device)IP\u5730\u5740"})})]}),(0,r.jsxs)(i.tr,{children:[(0,r.jsx)(i.td,{children:"X5"}),(0,r.jsx)(i.td,{children:"3.0.0"}),(0,r.jsx)(i.td,{children:"192.168.128.10/24"})]})]})]})]}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsxs)(i.li,{children:["\u786e\u8ba4\u5f00\u53d1\u677f\u3001\u7535\u8111IP\u5730\u5740\u914d\u7f6e\uff0c\u4e00\u822c\u524d\u4e09\u6bb5\u9700\u8981\u662f\u4e00\u6837\u7684\uff0c\u4f8b\u5982\u5f00\u53d1\u677f\uff1a",(0,r.jsx)(i.code,{children:"192.168.127.10"}),"  \u7535\u8111\uff1a",(0,r.jsx)(i.code,{children:"192.168.127.100"})]}),"\n",(0,r.jsx)(i.li,{children:"\u786e\u8ba4\u5f00\u53d1\u677f\u3001\u7535\u8111\u7684\u5b50\u7f51\u63a9\u7801\u3001\u7f51\u5173\u914d\u7f6e\u662f\u5426\u4e00\u81f4"}),"\n",(0,r.jsx)(i.li,{children:"\u786e\u8ba4\u7535\u8111\u7f51\u7edc\u9632\u706b\u5899\u662f\u5426\u5904\u4e8e\u5173\u95ed\u72b6\u6001"}),"\n"]}),"\n",(0,r.jsxs)(i.p,{children:["\u5f00\u53d1\u677f\u6709\u7ebf\u4ee5\u592a\u7f51\u9ed8\u8ba4\u91c7\u7528\u9759\u6001IP\u6a21\u5f0f\uff0cIP\u5730\u5740\u4e3a",(0,r.jsx)(i.code,{children:"192.168.127.10"}),"\u3002\u5bf9\u4e8e\u5f00\u53d1\u677f\u3001\u7535\u8111\u7f51\u7edc\u76f4\u8fde\u7684\u60c5\u51b5\uff0c\u53ea\u9700\u8981\u5c06\u7535\u8111\u914d\u7f6e\u4e3a\u9759\u6001IP\uff0c\u4fdd\u8bc1\u8ddf\u5f00\u53d1\u677f\u5904\u4e8e\u540c\u4e00\u7f51\u6bb5\u5373\u53ef\u3002\u4ee5WIN10\u7cfb\u7edf\u4e3a\u4f8b\uff0c\u7535\u8111\u9759\u6001IP\u4fee\u6539\u65b9\u6cd5\u5982\u4e0b\uff1a"]}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsx)(i.li,{children:"\u5728\u7f51\u7edc\u8fde\u63a5\u4e2d\u627e\u5230\u5bf9\u5e94\u7684\u4ee5\u592a\u7f51\u8bbe\u5907\u5e76\u53cc\u51fb\u6253\u5f00"}),"\n",(0,r.jsx)(i.li,{children:"\u627e\u5230Internet\u534f\u8bae\u7248\u672c4\u9009\u9879\u5e76\u53cc\u51fb\u6253\u5f00"}),"\n",(0,r.jsx)(i.li,{children:"\u5728\u4e0b\u56fe\u7ea2\u6846\u4f4d\u7f6e\u586b\u5165\u5bf9\u5e94\u7684\u7f51\u7edc\u53c2\u6570\uff0c\u70b9\u51fb\u786e\u5b9a"}),"\n"]}),"\n",(0,r.jsx)(i.p,{children:(0,r.jsx)(i.img,{alt:"image-20220416110242445",src:n(35015).A+"",width:"1006",height:"779"})}),"\n",(0,r.jsxs)(i.p,{children:["\u5982\u9700\u5c06\u5f00\u53d1\u677f\u6709\u7ebf\u7f51\u7edc\u914d\u7f6e\u4e3a\u52a8\u6001\u83b7\u53d6DHCP\u6a21\u5f0f\uff0c\u53ef\u53c2\u8003",(0,r.jsx)(i.a,{href:"/rdk_doc/System_configuration/network_blueteeth",children:"\u6709\u7ebf\u7f51\u7edc"}),"\u7ae0\u8282\u8fdb\u884c\u914d\u7f6e\u3002"]}),"\n",(0,r.jsx)(i.h2,{id:"vnc\u767b\u5f55",children:"VNC\u767b\u5f55"}),"\n",(0,r.jsxs)(i.p,{children:["Video: ",(0,r.jsx)(i.a,{href:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=4",children:"https://www.bilibili.com/video/BV1rm4y1E73q/?p=4"})]}),"\n",(0,r.jsxs)(i.p,{children:["\u672c\u7ae0\u8282\u9762\u5411\u4f7f\u7528Ubuntu Desktop\u7cfb\u7edf\u7248\u672c\u7684\u7528\u6237\uff0c\u4ecb\u7ecd\u5982\u4f55\u901a\u8fc7",(0,r.jsx)(i.code,{children:"VNC Viewer"}),"\u5b9e\u73b0\u8fdc\u7a0b\u684c\u9762\u767b\u5f55\u529f\u80fd\u3002",(0,r.jsx)(i.code,{children:"VNC Viewer"}),"\u662f\u4e00\u4e2a\u56fe\u5f62\u684c\u9762\u5171\u4eab\u8f6f\u4ef6\uff0c\u53ef\u5728\u7535\u8111\u4e0a\u5b9e\u73b0\u8bbe\u5907\u8fdc\u7a0b\u767b\u5f55\u548c\u63a7\u5236\u684c\u9762\u3002\u8be5\u8f6f\u4ef6\u53ef\u4ee5\u901a\u8fc7\u7535\u8111\u663e\u793a\u5668\u9884\u89c8\u5f00\u53d1\u677f\u7cfb\u7edf\u684c\u9762\uff0c\u5e76\u4f7f\u7528\u7535\u8111\u7684\u9f20\u6807\u3001\u952e\u76d8\u8fdb\u884c\u8fdc\u7a0b\u64cd\u4f5c\u3002\u7528\u6237\u901a\u8fc7VNC Viewer\u64cd\u4f5c\uff0c\u53ef\u4ee5\u83b7\u5f97\u8ddf\u5f00\u53d1\u677f\u672c\u5730\u64cd\u4f5c\u76f8\u540c\u7684\u6548\u679c\uff0c\u4e0b\u8f7d\u94fe\u63a5",(0,r.jsx)(i.a,{href:"https://www.realvnc.com/en/connect/download/viewer/",children:"VNC Viewer"}),"\u3002"]}),"\n",(0,r.jsxs)(i.p,{children:[(0,r.jsx)(i.strong,{children:"\u8fde\u63a5\u5f00\u53d1\u677f"}),(0,r.jsx)(i.br,{}),"\n","\u76ee\u524dVNC\u652f\u6301\u76f4\u63a5\u3001\u4e91\u7aef\u4e24\u79cd\u8fde\u63a5\u65b9\u5f0f\uff0c\u7528\u6237\u53ef\u4ee5\u6839\u636e\u81ea\u8eab\u60c5\u51b5\u9009\u62e9\u3002\u672c\u6587\u63a8\u8350\u4f7f\u7528\u76f4\u63a5\u8fde\u63a5\u65b9\u5f0f\uff0c\u8fde\u63a5\u6b65\u9aa4\u5982\u4e0b\uff1a"]}),"\n",(0,r.jsxs)(i.ul,{children:["\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u8f93\u5165\u8bbe\u5907ip\u5730\u5740\uff0c\u4f8b\u5982\uff1a192.168.127.10",(0,r.jsx)(i.br,{}),"\n",(0,r.jsx)(i.img,{alt:"image-20220610160658103",src:n(63798).A+"",width:"753",height:"107"})]}),"\n"]}),"\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u8f93\u5165IP\u5730\u5740\u540e\u56de\u8f66\uff0c\u5f39\u51fa\u94fe\u63a5\u672a\u52a0\u5bc6\u7684\u63d0\u793a\uff0c\u70b9\u51fb ",(0,r.jsx)(i.code,{children:"Continue"}),(0,r.jsx)(i.br,{}),"\n",(0,r.jsx)(i.img,{alt:"image-20220610160715916",src:n(9520).A+"",width:"609",height:"442"})]}),"\n"]}),"\n",(0,r.jsxs)(i.li,{children:["\n",(0,r.jsxs)(i.p,{children:["\u8f93\u5165\u5bc6\u7801 ",(0,r.jsx)(i.code,{children:"sunrise"}),"\uff0c\u52fe\u9009 ",(0,r.jsx)(i.code,{children:"Remember password"}),", \u70b9\u51fb ",(0,r.jsx)(i.code,{children:"OK"}),"\u8fde\u63a5",(0,r.jsx)(i.br,{}),"\n",(0,r.jsx)(i.img,{alt:"image-20220610160928136",src:n(85868).A+"",width:"609",height:"442"})]}),"\n"]}),"\n"]}),"\n",(0,r.jsx)(i.h2,{id:"ssh",children:"SSH\u767b\u5f55"}),"\n",(0,r.jsx)(i.p,{children:"\u9664\u4e86VNC\u767b\u5f55\u8fdc\u7a0b\u684c\u9762\u5916\uff0c\u8fd8\u53ef\u4ee5\u901a\u8fc7SSH\u8fde\u63a5\u767b\u5f55\u5f00\u53d1\u677f\u3002\u4e0b\u9762\u5206\u522b\u4ecb\u7ecd\u7ec8\u7aef\u8f6f\u4ef6\u3001\u7ec8\u7aef\u547d\u4ee4\u884c\u4e24\u79cd\u65b9\u6cd5\u7684\u521b\u5efa\u6b65\u9aa4\u3002"}),"\n",(0,r.jsx)(i.h3,{id:"\u7ec8\u7aef\u8f6f\u4ef6",children:"\u7ec8\u7aef\u8f6f\u4ef6"}),"\n",(0,r.jsxs)(i.p,{children:["\u76ee\u524d\u5e38\u7528\u7ec8\u7aef\u5de5\u5177\u6709",(0,r.jsx)(i.code,{children:"Putty"}),"\u3001",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u7b49\uff0c\u7528\u6237\u53ef\u6839\u636e\u81ea\u8eab\u4f7f\u7528\u4e60\u60ef\u6765\u9009\u62e9\u3002\u4e0d\u540c\u5de5\u5177\u7684\u7aef\u53e3\u914d\u7f6e\u6d41\u7a0b\u57fa\u672c\u7c7b\u4f3c\uff0c\u4e0b\u9762\u4ee5",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u4e3a\u4f8b\uff0c\u4ecb\u7ecd\u65b0\u5efaSSH\u8fde\u63a5\u8fc7\u7a0b\uff1a"]}),"\n",(0,r.jsxs)(i.ol,{children:["\n",(0,r.jsxs)(i.li,{children:["\u6253\u5f00",(0,r.jsx)(i.code,{children:"MobaXterm"}),"\u5de5\u5177\uff0c\u70b9\u51fb",(0,r.jsx)(i.code,{children:"Session"}),"\uff0c\u7136\u540e\u9009\u62e9",(0,r.jsx)(i.code,{children:"SSH"})]}),"\n",(0,r.jsxs)(i.li,{children:["\u8f93\u5165\u5f00\u53d1\u677fIP\u5730\u5740\uff0c\u4f8b\u5982",(0,r.jsx)(i.code,{children:"192.168.127.10"})]}),"\n",(0,r.jsxs)(i.li,{children:["\u9009\u4e2d",(0,r.jsx)(i.code,{children:"specify username"}),"\uff0c\u8f93\u5165",(0,r.jsx)(i.code,{children:"sunrise"})]}),"\n",(0,r.jsx)(i.li,{children:"\u70b9\u51fbOK\u540e\uff0c\u8f93\u5165\u7528\u6237\u540d\uff08sunrise\uff09\u3001\u5bc6\u7801\uff08sunrise\uff09\u5373\u53ef\u5b8c\u6210\u767b\u5f55"}),"\n"]}),"\n",(0,r.jsx)(i.p,{children:(0,r.jsx)(i.img,{alt:"image-Network-Login",src:n(43280).A+"",width:"1073",height:"621"})}),"\n",(0,r.jsx)(i.h3,{id:"\u7535\u8111\u547d\u4ee4\u884c",children:"\u7535\u8111\u547d\u4ee4\u884c"}),"\n",(0,r.jsx)(i.p,{children:"\u7528\u6237\u4e5f\u53ef\u901a\u8fc7\u547d\u4ee4\u884c\u65b9\u5f0f\u8fdb\u884cSSH\u767b\u5f55\uff0c\u6b65\u9aa4\u5982\u4e0b\uff1a"}),"\n",(0,r.jsxs)(i.ol,{children:["\n",(0,r.jsxs)(i.li,{children:["\u6253\u5f00\u7ec8\u7aef\u7a97\u53e3\uff0c\u8f93\u5165SSH\u767b\u5f55\u547d\u4ee4\uff0c\u4f8b\u5982",(0,r.jsx)(i.code,{children:"ssh sunrise@192.168.127.10"})]}),"\n",(0,r.jsx)(i.li,{children:"\u5f39\u51fa\u8fde\u63a5\u786e\u8ba4\u63d0\u793a\uff0c\u8f93\u5165YES"}),"\n",(0,r.jsx)(i.li,{children:"\u8f93\u5165\u5bc6\u7801\uff08sunrise\uff09\u5373\u53ef\u5b8c\u6210\u767b\u5f55"}),"\n"]}),"\n",(0,r.jsx)(i.p,{children:(0,r.jsx)(i.img,{alt:"image-Cmdline-Linux",src:n(16956).A+"",width:"734",height:"482"})}),"\n",(0,r.jsx)(i.h2,{id:"\u5c40\u57df\u7f51\u6784\u9020",children:"\u5c40\u57df\u7f51\u6784\u9020"}),"\n",(0,r.jsx)(i.p,{children:"\u5bf9\u4e8e\u4e0a\u8ff0\u5404\u6b65\u9aa4\u767b\u5f55\uff0c\u59cb\u7ec8\u9700\u8981\u4fdd\u6301\u4e32\u53e3\u7ebf\u94fe\u63a5\uff0c\u4f7f\u7528\u4e0b\u9762\u547d\u4ee4\u5b9e\u73b0\u5c40\u57df\u7f51\u8bbf\u95ee"}),"\n",(0,r.jsx)(i.pre,{children:(0,r.jsx)(i.code,{className:"language-bash",children:'sudo nmcli device wifi rescan # \u626b\u63cfwifi\u2f79\u7edc\nsudo nmcli device wifi list # \u5217\u51fa\u627e\u5230\u7684wifi\nsudo wifi_connect "SSID" "PASSWD" # \u8fde\u63a5\u6307\u5b9awifi\n'})}),"\n",(0,r.jsxs)(i.p,{children:["\u4e0a\u8ff0\u547d\u4ee4\u6210\u529f\u540e\uff0c\u4f1a\u51fa\u73b0",(0,r.jsx)(i.code,{children:"successfully xxx"})]}),"\n",(0,r.jsxs)(i.p,{children:["\u6700\u540e\u677f\u5361\u7aef\u4f7f\u7528",(0,r.jsx)(i.code,{children:"ifconifg"}),"\u4fbf\u53ef\u83b7\u5f97\u677f\u5361IP\u5730\u5740\uff0c\u4fbf\u53ef\u62d4\u6389\u4e32\u53e3\u7ebf\uff0c\u4f7f\u7528\u524d\u6587SSH\u767b\u5f55\u8fdb\u884c\u8fdc\u7a0b\u94fe\u63a5"]})]})}function j(e={}){const{wrapper:i}={...(0,s.R)(),...e.components};return i?(0,r.jsx)(i,{...e,children:(0,r.jsx)(o,{...e})}):o(e)}},99330:(e,i,n)=>{n.d(i,{A:()=>r});const r="data:image/png;base64,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"},35015:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/image-20220416110242445-115ab0e8ff800aca7f33f669ccded7f4.png"},63798:(e,i,n)=>{n.d(i,{A:()=>r});const r="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAvEAAABrCAYAAAD+Wt3tAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAADsQAAA7EAZUrDhsAACRzSURBVHhe7d15cBzXnR/wbw8Gg/sgcRLgBXIGvCCalmTZBO316ljZALVeOClDcawVa2sdILteBZCzTNYpKhWXuJZ3qViAEzshkqo1Xc5WCYnXsE0SXlvWRQHULR4gRRIjUuJNgOCB+5rpvN/rbmAGmBkApEhpoO+H1Z4+X/eM9ce3H3792jCV1U8P4GYc++s0e46IiIiIiG41l/2pvfFYKu73JthLk35fY63/XkWSDuwtj6ZgdX7YofHD34hNxiY0+u1lIiIiIqI4E5bEf+cP4J6l4SFewnpmErAmPwFr81x49NlhHO0O4kdVyfYekbXWGjBqW+2lSf7GTTA2NcKv/jVuUvtECtSttfY+oVpRa8j+9hShbTHzeYmIiIiI4ltYiP995zge8LrtJctXy9x47UwQ9yxx4TkV8l8/E8BTz4+iONOw94isoqoGaGpR0TuUH3ua21GzrQ5ee01NTRnqt8wQriXUG5XAXhOmaU9VLdgUoTt9xvN669BmtqHOuQAiIiIiojgTFuJfV2FdwnloqcxnFyfocB+qb8S052KoqEINmtASmqb9e9DcXoOqCntZVG1FA+qxJWp9SytqK5tQowL8ztDjKnaiLVISn+15iYiIiIjiVFiIl3Auve33qOAunEAv697tDuLc9aBeL6RHPrYKWJ3ik2nav6cZ7TVVaksoL+p2NQD1PkSskGltQVN5A7bOOoDPcF5dE18b0lMfXqbj9O5LWc5kT7+1z8T1hZX7RD5enUiXC9W22tvV/kREREREH4awEC9ePx3AvyizSmru97pxrCuow72U0IifPpysp9mo2NqA8onSFrukJVJ3uLcOuxrK0VQZGq5v3KzPqwP4dqztdMp0OlHdbN1M+NaWo/1op71bCzrKy9Fxwo7tLU0or96sbj+iH+9oqmxBlWxrq7PXEBERERHdnGkh/vf+cd37npFk6Idc/6nDKqX56jo3/tN9Hl1y40wz8m5Gdbld2jJDSYu3bhca1L6VEbvj52i255VefrSj3uf0pPtQ3w4d1r2bqyduBFpbOlC9axvKmveoWwI/TnSUo3qzN+bxjpq9O6f85YGIiIiI6OZMC/Hnek3d+y6h/QFvAl4/a5XNyAOuu94ax39vH52YZubF5upyXdoSuZQmlFVWU95UidoWe5WQGvf2ZuyZzMWzMIfzljeg03lY1p50rb2+EejACX8rWjqqsdnrw1qo62hVNwWQ5RmOJyIiIiK6RaaFeCE18I9tSpwI9EJ63iXUz3V8eKtHezu2NAMNMxW2O2U1TU32ClGBrbpkfspQlK21EUencczqvPoGoR47Qjr/W2udkh65EQCat2xHU1mpWrKXt6sGdSmNEvN4IiIiIqJbI2Ii/0XHmP05OSrNrrfGcFaFennRk7zwadZvadU92u1oD+29jsEqq7EXbN66Nph7y0LKVtTUUhW7x3tW563Azs4GdFROtttSNVn+IjcCaJ+sp7eWJcM7DcY+noiIiIjoVjBMZfXTA/bijZl1oCciIiIiops2t9oYIiIiIiL6yOkQLw+t3qibOZaIiIiIiOZOl9PY80REREREFAdYTkNEREREFGcY4omIiIiI4gxDPBERERFRnGGIJyIiIiKKMwzxRERERERxhiGeiIiIiCjOMMQTEREREcUZhngiIiIiojjDEE9EREREFGcY4omIiIiI4gxDPBERERFRnIke4v0NKDfK0eCXhVbUGrXqfz8iYdcSrz7i35CIiIiI5g0V4v1oKDdgGCFTeYNae/u01qpz1k6Pt/6G8tt+LdFZv9O0y9Q3GAznRERERHT7TPTE1+w1YZr21F4Pr7ce7WY76r32DrdQRVUN0NQyJQj7sbt5P2qeuL3XQkRERET0cffxqImvqEINmtASmuL9u9G8vwZVFfYyERERERFpM9TERysTkfruyfKb8psuVq+A1Rk/eTb/7mbsr6lSW2Rh6rVEPr+U5Uxei7XPRPlLa60uzbn1ZvPbOKU54ftGqCgiIiIiIppmIsQ3Vc4UPB0SPJ/Emk6n/KYT1c2+mw6gFVufwcaJkhq7lCZiN3z08/vWbMT+dzvt3VpweONGHD5uB/yWJmysfkjP34zQ30lPvsex394219+mqbIFVU4J094atczaeiIiIiKaWcSa+PZYxecqHDep2Pq4zwmyPjyuUqwTlm+Y9yFUb7RLamKV0sQ4v/eh6okbgdaWw6j+6RO4o3m3uiXw4/jhjah+6OaL6sOeHZCpU9182Nvm+tvU7N1p/aVBVGzFM873JyIiIiKK4cZq4jc+g87QIKummMF/Vrx4qHqjLqkJK6WJJNr59Y3AYRz3t6LlcDUe8vqwBs3Y3apuCiDL9vG30i35bYiIiIiIJs09xMtDqPsfx46QHuPW2g+nDMTqSX8SjzYDz2yNEuFjnl9uBIDmR59E0x2r1JK9/KRqsPohtXSLzfG3CXsGoOFRPM4HeYmIiIhoFm6gJ74COzufweGQ2vCWqpCykJuhe9L3Y3/MXvPY55cbAeyfrKe3liXD347e8Ln9NjVomdjP9/gd2Gt+SL8jEREREc1rhin1HnSbyeg0Prz7hImdTO1ERERENEc3VhNPREREREQfGYZ4IiIiIqI4w3IaIiIiIqI4Y/T29jLEExERERHFEePatWsM8UREREREcYQ18UREREREcYYhnoiIiIgozjDEExERERHFGYZ4IiIiIqI4wxBPRERERBRnGOKJiIiIiOKMcfL9s+bC7HR7cdKFrmvovtKL/NwsFKppbDyA0+cvY2BwBIYB5C7IRGFelppXC1MMDo/izIUejIyM2WssBaodmSK5PmJg/2ngfH94e2vyTGxcPDkK5oGLBvZ2Gni320BQLZfmmKj0mriryIRryqUE1GEHL7pw+JI1vyjdxB8sN5GWaO8Q4oPrwGtnDfSPGmq7iXsWAyXZ00ffHBkHfvueCy9+YOCMOiYjCbiz0MSfrAlicYa9Uwg57+vnrOsN5Va3T39cGsQvjkW/j/Kp7/aFpdOvgYiIiIg+2Yxf/+5V8/N3r5oWxn/53Ju43jeEB7+wHonuBLz02rvoHxy2t6oD1e6L8rKx6a5VSEn26HXy8tcjnWfRceIsxsbGMTV+rl+9FBvWLLOXJv3G78IPX3Ph6pBqw17n+No6E/9+YwCjKg03veXG/z1q6CAdKkHl4K+sCqLusyaS3VYLl4cMfH9fAtrPAEG7UfmGi1TQ/pvPB3BPsbVyXN0JPNvhwv96W7UbMNR3sL5bosvEo58y8WcbAqp967fpGjDwxAsJOHRx+nWme0zVrokHVsitheXUVQPffcmFEz3qhmPKAclu4B/+ZBxf/7maiWKzL4gnvjjZXizZ379gzxERERHRfGf86Kd7zK99+R54PJPd09ID/9tXDiM9NRn3fm6tnh8cGkGqCutLinIQVIlUeuVHRsexKD8b929cB5fLpQP8O0c/UNuDyM5MQ35OJtySsG2yb3HBQnvJ8tIHLnz3RQODYwYK0kx8phjITJpMvJ8qMPHF5SZ2HZCgnaACtok/WGbij1RYlqbfPO/Cr45BB/CvlwXxrXuCGAsA/+VF6S1PQIoK9XJ8hrrPkJ526XHPSzXx3yoDWJ4N/Oq4C3/f5tIh+478INbmAad7Xdivwr+0/1efCeJh1e7AKLD1dwl4+4KBhSnAN9YH9V8BegaBX7xr4OAlF1ITTez4owDuKrIC/1/sTsC5PgnsJj63GCjKmPxeiQnAw+uC+Nkh6/e5oPZ74X2XuhmwbkjE2jy5KZh6uxCZhHjzqdX2EhERERHNZ8YP/+HX5uc2eFFassheBbxx6CTefe+c7jUfDwRVOD+DrIxUPLCpTAX5JN3jfvX6AH7X1oHRsXHcX74Omekp2PviAQyPjGHNyiJ17HK43a6I5TaOgTHg3+52o7MHuK/ExNbyALKSMa0s5nSvgW/tNnB12IU//3RAB2hPgrWTujw8d8qF770sIRr48UMBXFHB+vF/TkCSCsrfvXccn1UBWqLylSFThfsEvKGC/9fWBvHNu0z8u9YE+K8Y+NP1AfzZhiCS3MDouIlnj7jwozcSsDQL+N9fGceL7yfgqX0GctQNQGNFECvsUpuLA8Dv1fl/8o4L/Sro360CfGPFOJ7Zn4D/d9TAonTg71SwX7nA1DcF0cgNRt1vElCsgv7PH1Z3IXPEEE9ERET0yaFj5ckzXbp3XQTU56mzXSpwurB8cR4u9VzXJSbrfIt1gBcSzBdkpWHFknwd6C9dvq4nCfDSA79+1VIkJkqvefQAL6ROXOrKc1JM/OXdASxImR7gxdEuE5cHXfDlSHkNJgK8kGB83/IA7i6GCvnAm+ekXRXEVQ6+u9jqAZfd5VJyUg11A2B9T+k5v9gnJS9Wz/y/XGPqAC88bgMVPhNLVIC/rG4I/FdduixH6nG+utrUNwu/POHSNwqP/DwBP37dCvByHlP961bBXnryVTOouSuge+xjBXgiIiIiorlwJah0KeUzfQNDeoU8kCq96/m5mUhPS9a17WJBZvjDrxLQM9R2IWU1vf3W8bnZ6WGlObFcHwGGVfOLM00VsO2VEbx3xaVr0FdkB3Tt+VSJKj2vUkFZXOw3dIAXS1S7U28KpKdfDI2b6FPBW/aVB1lz08LbTU80dLiXm5pr6qtd7LfWS6//ll8k4O9fceHVs1JH78LqXBN/fqeJn3w1gKcfDKibDOCCug5Pgok7i6zjiIiIiIg+LK5lxbm6p/3U2W7dq37K7pVfuaRABeDJBBypUz20p11uBsToeEC1M7uHMSVgyzQaNPQoLtFk2sG7f8yly2cikRsC4fSmi0i9+qHkewv5GlN3lXXO8QmGqQO5/p2uqutQ4X9Zlqlr2n+8eVzX13/z0wH4FppIUeeX4xLVzyGXKvsSEREREX2YXCuXFqjAauiSGhl95kLXVSR53CguXGDvMjt5CzN1Cc6F7qu42jtgr41tSSaQ4TFx8qoL71yMXm+yKkdKaKCHivRfsVeGONsLvHrGUOc3UKr2/bClJAKrc635hSkmnvlyAP/zoQDqPhfE+gITqSq4t51x4a/2JuCJ5xP0dUjv/Mi4oR96tSuViIiIiIg+FK6CnEykpSahf2BYP9AqD7IuLcqDJzGkS3sWZCSa3IUZGBsL4JU3T+D9c5d1ic3wyKiulZcpMKUbvWSBifIlUtJi4u9eMfBPKvBKT/eVIau+XaYhFYTX5ZtYkxtU6w08vT8Bh7sMXeYivffHewzsaEvQ48uXLpRQPbu/AsyFdMhX+kykeYDeEUPfMMi5pcddRsKRAC/X/9Z5A3lpMhKOiarVQd2b/+sTLjS+auhhJi8PysO51ve6pqbboxW16kI2NfrtZZu/EZvUermBM4xNCNvcWmuvj7Btmijti7B2atWetrBzG6id2BBJlPajXKO/cVPI+skp+jmiXb+13jp+pt+AiIiI6PZyydCQJYvz9cLZi1Y3tzywKuFlLqQXXka5yc5M1eG9/a0T+O2+w9j74kG0vnRATyfPdtl7W+RB0G/eGcC6PBM9KqD/VxXQv7XXjW/+Sk2/tKb/c8h6+dJffMZEQZr0xhv4698moHZPIv5ytxv1rQl4XYXn7GQTtXcH9fCPt4J3QRCPrrduEJqPuFCrzv34bxLwb34tve8GulVAv6MAeMTe5/4Vph7yUkJ+85EEPKau85u/ck18Lxl+8tbyo3GTBNAWoMZeNUEFVF89yvZKiZCa9pahfkujOsLetn0tOmW9s80XEsAnxGpfkZAd2o65ExXWhinnrkFT5Vzbj36N3ro2+3z21NmActVAlXXyELHal22V6GjonNY+ERER0ceBrmEpXV6oe9GTkxLhU/N5CycfYpUe+SS1XsL+VDIGvGxzxoKXYShlXPnVK4qQlupB0JQQG9B18jINDU8vEC/OBL53fxDfuMPUD6JKD7sMPdlvT6MB62biU4UmnrwvqF/SNKzWd1ySEWasfTYUBPG39jYhL1KSB1jlcyq5VNkm48bLW1NlPj3Cc7hyDyNvds1MsvZzqzuOr98RxNZNQSzOAs71Gth/1sCxy4Z+sFZezPS39wcmbiKk/KfmblPtH9BvnZVee/mrgvO9BsbCb5Kca5Hr+nB4UdcmQXYnquw1E1pb0FTegK1OsK3YigY0Y49O8RXY2VanjrZVVKmM24ET03qiY7QvIXh7Bxp2hbQTJiRU31D7s71G9VV3yA2DcwMRKkb7/j1obq/Btjr7DPL7lDehhSmeiIiIPiaMa9eu6eQrI8yMjI4hLSVp4iFVIWU2gWBQl9y4E8J7j2UUGwnm8kbXVHWcI2iaGBkZ0+2F9ugneRL1jUIkUjcuZTR98oBqyDGZnvCRawZVAJY3oZ7ptR5MLVbBf8UCeWOqvYMipSpXhw1kJZnTeuZHAsD5PgMe9RWl3Qv9VuCW8dlDyXeQFzbJ6DmF6cbEm2DlnPJXg/euAJcGrPOWZFvDUUoQj0QebpWhKs2Qx2cNtSQvm3IMq4Av15JomPomYa5ijRPfWmtg+9pOtDmh1OklnwjC0vPsw9FtJnZOTbuybyWwd6Infbpp7Uu5zBaguqwe9U3WqvKGye1S8rIFu6xlab+lCua0E0+a1v5U0a5x2veMLOLvM+WaZrwGIiIiottoInbKw6zywqbQAC9kmEnpYZ8a4IX00su20AAvZFSblGSPHjNetjtTtAAvpKc6V4XqEhXIJRQ709ShJ2WMdqmR/7LX1GO5ry8ID/AiO9lqI1JpjbwASrZJ+JdgruenBHgh36EwHTpoOwFeyP1FbqqJzy428ZVVpn5JlVxztAAv5PqkHec7yRQa4IVzLTcS4OfMtxbl7fXY4fQst+5Afbs9H0rCeGUTaiL2ZMfQeRTtqv2jVdLTrabOBqB+S1hdeXu9T9/gGar98rU+e+0NiHqN8tcAtX5b7AAfif9Ehz1HRERE9PEUI3rSvOWtQ5uuRZeacDVtX4uGmnKEZmn9gKivGdWdEXrnZyO0XEedb1tNO5qlXkeF7i31QINq16pb34syFehjP9waWcxrtEtiptfCz8xbWmbPEREREX08McR/UlXstEO0mtpKcbSpDKUT1SQGfEe3qW1tuKHqEenpt2en8u9pRnvNtpB2K1BVA3REKmiPYaZrlFp4NGyd218QQnWcwOQV+SGd82XOD0RERET0EZuXIV6GsuwbmPsYjuPjAT1JsL0RA0MjEYfSlNZknbxEa+q28YD9elmbnHtMrmPKfrdSa62MxGIHXn8jtjfVYG+k7ncpXZnNcIvezahGPbZMjvuo2ixH9Wav2lSN8qbtIW20oqXJDsizbT/WNWrSpnW+MLNtXz/oO6XcSK2Z+MsCERER0UdsXoZ4CfBvHj5pL82OPKT76kE/9r15HG1vncCBdz/A6Oi4vXV2Dh8/o1+aJeeWYTYd3T29OHT8NM5c6MFrB/wqzFsBXR6e3f/OZKK8cq0f7W934tV3OrHvjXfx3ulL9pYPmzO8ojWFPbAp9exoQqW9zZnmVu4iI79YZTL6eD2kpN1jrkt5ZMhGp239ROrcSnZmukb/CXRg8i8Lc6euf1cDOpxyI7nEGR6OJSIiIrqdJkanmU+u9Q7inaPvY+OnfRgbH9ej4siDuRLU5QFeCWbSKz48OobUZOupWOlBf/G1o1hXugRJiW51/Cn90qs1K4v06DxDw2NwuQwkq7bkU3rUZfQd6bR3htl8VQX0BZlpKsAP6od91/kW6/O8fug95C7I0L38bx85hbvXr0Dp8kW63V8//w6qHrgLg8Oj+N0rh1FWuhiL8hZgaGRUP2ScnTHlyd4oYo1OQ0RERETzy7ytib/eN4j975zQverSu903MIQ3Dp/U68W5S1dwUAX9UBLuM9KSkbcwE/kLszA4NKLLXaRn/ZU3j+mQ3/nBRb3v++e6dWjfp9a/oUK6QzWBZcW5+OD8Zd3TLsdfunwdxYUL9PaSJfk49t4FXXoT6sz5HhTkZmHl0gJ1A+BBTnb6rAM8EREREX2yzNsQL73ld60rwQOb7kBSkhuXeq4jKy0FZy5e0b3jp1XIXpRvBWuH9IyfVmH68PHT6LrSi6VFObh8pQ891/rxhbtXY8PqZTjaeRbj40EV1PPw+btX6d7+y1f7dC+/Y0FWOlyGC1evD6Crp1e/xTYlyerxl6E2S0sK9V8KQmvvB4dHdG89EREREdFM5m2IlzHsk5M9uswlJzsDff3DWFqcg4td13C9b0D3hOfnZtp7Wwx5BZMZxJXr/cjPydKhundgWNfYSy/8W0dOIdHt1iU6/g8u4vVDJ9Fx4gzGVYAPfWBVXn5VoNqW3v6z6qZheXGuvcUib8UdHh7FB+d67DXWTcdYyI0AEREREVE08zbE61KW8aD+lBp1qXPPTE9VwT4Rx06e1y+iSk2e8pIqFaSlh/2ushJcunxNP5wqxxXkZKLyDzfgj++7E5X3btBtHvWfw2fuWIH1q5dZNTRTSNnMqTPd+oagqGChvdaS4HLhznUrcFjdAATt8C8lPKfOduuRaRzz7mEFIiIiIvpQzMsQL7XtIyNjerSZ/W+fwPlL11BcuFCH9OWL8/H+uR4sKwrvHRd6JBI1paelYKnafqTzHBblZetgLXXvHZ1n8FbHKd27n5KUiEPHTuuHVl3222wly8vxIj01WT9QK7Xt0jPvcLYvzE5DyeJc/XCtkPNI2c1zbYd1Db60KzcSRERERERTzcvRacbGAugdGNKBua9/CAuy0pCZnqK3SX37Wx0ncd/GMh3GHfIA6uWr/ROhW5Z7+4Z0j72Uz3T19Opa+qyMFL1OativXBvQYV32lVFpZFQcj8etH44V/YPDcKlrkJFqhPTsSx281MULKcGRcht5EFbItp6rfeq4EbjdLuQtzNKj6cyGjE5DRERERJ8M8zLERyJBWwL88ZMXkJ+TidUriuwt88MP/8dP7DkiIiIimu8+OSF+PID3znTpceGL8hfoMdjnEwnx//n6l+wlIiIiIprP5u2DrVO53QlYVbIISxblzLsAT0RERESfLEyzRERERERxhiGeiIiIiCjOMMQTEREREcUZhngiIiIiojjziRmdZr6LNDrNp197AqkjFzAyOgJD/SvOWwL3ik+jbyiAK/1DGB4dt/e0DH7wJka631N7zvy22NB31Dr7Lvd5MT48gr7eXngSE5GSkoLhEXVuw9Bj4CeqdaOjowgabuR85WkYian4fV+6fTQRERERzZbx2GOPMcTPA3lFK6eF+J+ufQMul/yxJRNDgxeR6PGoQD8GT1ISktUkJFzLlDE8iMHUNCQkJEysExLEhYTvoaEhvX0unADvzAtZMl2Jqi03/tWbq/S66TKx96ki+F44Bd9vR+x1SmEeOuty4NULI2hsPIX6i3oB2FAM8+EMe2HKtmmitC/C2ulD7XfOoUlmw84NtD57DJUH7IVporQf5RrLHyxB273W/yehop8j2vVb6yv0/Ey/AREREcUrQwUshvh54MnvN04L8c89cE6HeMNIx8jIFSSoeXn7rEeFeekVd8h/AsljowioEC9BW6ZxtZ8E9iQ77EuIHx4etm8KZmcitE8J8cJaZ+KLrQXWiglJaPh2Cery+tB6JAO+rtCQagVUOMFWAvF9o9j0g260y7ZvJ2G7nre3PYzJAD4hVvtKWJuhIpx7zu3P9hoVfcPgwY45tW9t29xhr4vVPhEREcU11sTPYwUFBSgsLEROTrKeLyouRklJCYrVp5AgLUFdgvlYcgoCgQD6+vowODiot0nQDwaDenK73UhPT0dqauqsJgnsaWlpyMvLQ35+vm5rwYIFer3cRMh8Vla2vo5wI6j/wTEYKni22GsmbMhARXcPtjs90wcuoxHpqC6UhV5UhgbvAyrkwoO1eluoGO1LCL7Pg8Z/nBrgHX1omTj3jbQ/22sEah7IQeezkcJ3jPYLM7FZhfsdTqiX36c7A1UbrEUiIiKaPxji57Hc3FwdliVIS5hfuHAhsrOz9SRhfsWKFVi6dCmWL1+OZcuW6c81a9bA5/PpeQn+Nzo5xzvnk+WcnBysXLkSixcv1tck229eEkojhGAd+DGKo3MpJZEQjH7ggdUwn7KmzgedEpdebH/Bg63OsrR/pOfmSlWiXeOGYuzMD7lZma1CD7xH+kKC/wiOdgG+/OllOkRERBTfGOLnMQnu0aYlS5bc8kl6/J3zLVq0aGIKXT8nF0fhz8vBNqdneUMu6vLs+VBSivJwBloj9mTHICFYtV/aIT3damrsAe4tQkPIZXrvLbECvmrf3xVShjNXUa9R/hqg1j8f7a8B0ZXne+w5IiIimu8Y4il+XOyG79k+VDxs95TfN4rGIyM4EdKTLQ+ImnXp2NMY66HTGELLddT5dhxJwub1STp077oXaFTt6oD/nfPoVIF+7w2UqsS8RrskZqJsZw7au0btOSIiIprvGOIpvhw4Z4doNf1gBKXrJstRah5Zjbb8HrXtBkdkkZ5+e3aq8vXp8IaVz/Si5cjcS1VmukaphccLl2/8QVR1PeX2rPTqr80HOm/mLwZERET0scQQT3Gr5hEZYtEOvIV52LquD7U/69XbwkjpylMlYWUxEV3sxR7kYJdT967bHMGeQyNoP9QP/7qckDYyUbXODsizbT/WNWrSpnW+MLNtXz/oO6XcCDdQW09EREQfewzxFEdkCMXJh063hg6vKPXsyMBOe5szza3cRUZ+scpk9PF1MkKM3WOuS3lGUVfntB0y3ORszXSNhUnwzfVh3DDq+v+xBz6n3EiGl5w2VCYRERHNBxwnfp6INE68BLnZMM2g+l9rDHcZAtJZnhzX3Xr5k2G47M/J8d4dMgylyyXHho8HPxMpiyEiIiKiuWGInyduJsT/pvWfcfLkKQRVeH/wwQdw7NhxXL1yDUnJHj3c5Ntvv4Oenh586UtfwoEDBzE2OopFRYuQlZmJ7u7LuHrtqn4ZlIw3Ly+EeuSRbyA3N8duPTaGeCIiIqK5Y4ifJ24mxEsvuvxnIJ/Siy5hXJZlknlnu+jt7UWmCu+hb251joPsoj5kW+j2WBjiiYiIiOaONfGkSeiWt6rKm1mdIO+8zVXWyTaZ5IVN8inbnMk5zp2oJvU52wBPRERERDeGaYtw4MABvPzyPnR0dGDfvldw8OAh7N69Ry0fsfcgIiIioo8TltPMEzdTTiMlMuPj43o+EAjo3nXhPKAqy7JdluWf/AeTkpKC5OSbf50/y2mIiIiI5o498YSkpCS43YnweDx6kvm0tDS9PjFRlt0qyLutbYluWKPX8N6PiIiI6KPCnvh54mZ64mVEGRmdZmBgQAd1CfAFRUtQ+cMrWFuUiPEAMDgaRKrHhdFxE9eHgvjaZ1LxpxvT7RZuHHviiYiIiOaOPfGky2WWL1+GNWtWw+tdiaKiRfC4Xdi8PgXePDdWFbqxYYlHf5YVJ6Lcm4TSgkT76Fuj/MESmN/OQ7m9LPQ6dWMiU+hLnGoemXxxUqfzttUpIrUX/vKoyTeihp5nLu1Fuo7QdVO3hYp4fRuKYx5DREREn1wM8aRHl0lNTUV6erruhZf5JI8bf1OZha0VWfgPavqPat75lPWfXXGrQmUm9qrQuit/FH57jaYCbVtZPzZ95xiM75wHHi5GjawvzMPaDllnre+8t2gijFuitKfUPFKCzR2n7GPtN7Oq9nZNnOcU9pTNsr0o19H0M2ed1V5jdx92OG+Z1aK31ylvXJ3SHhEREZFgiKePmV5UquDqe27UXraU53vg7+hFu17qxfYXPKiS3viL3ag/oFcqvWiZNqBO5PYkJG/N78GWsEBty/OgzJ6dLkp7M16H9LYXqZuGy2iyly3Rrs8DvODsK+0loZQhnoiIiGwM8RQX2rtG4S3LtMtNklBdFuEvATqY96NZetRnIiG5A9j2lF3m4pSyqDDuexbYqddLT/15q4d+LiJeRya23Ts6pRc+Orlp6eya3LdDzfvyWVJDREREFoZ4ig8HzqG2KwdtOlwXoTQk4GpSP17nwY4fdNu99TPz3utBiy5XOabb3iV151LGct9ojHKaGUS5jvIHc+Cb6FknIiIiujkM8fPYvn378OKLL+Lll1/GSy+9pCeZl/Xy6Wx7++23cejQIT0dPHgQly9fRldX16wnGWf+dpisLz+FFrV8wu7p1g+PlvWp9efmFJL9IaG6qaNPf5avTwcmynZGUP/8KDavn10PePTrmFsvvCO0571MzYf2zBMREdEnG4eYnCciDTEZ16RH/F8DWyL1rEtvt/SWyzaZl+D8sxluJKa1Jw+UZqDFDtwSwKs6jqESIW3r0WtKUPq8Wj9R726b2l6s65jNNU5tT5alR19fX/i1EhEREbEnnuKENYqLrl+XUVvssCu141hXZK23JxmOMfKQkqF6UTlR+74aO3HeCuphZTslqOuy1s/UXrTrcLb5p/Siz3h9F7ux5QWPfX1FwLMM8ERERDSJPfHzxLzriSciIiKiqNgTT0REREQUV4D/D2e8hcYfleeHAAAAAElFTkSuQmCC"},9520:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/image-20220610160715916-424308bc7cacbe86ed409b88714694cd.png"},85868:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/image-20220610160928136-46502263809d9f36aabdad4d0229147c.png"},43280:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/image-Network-Login-76df2b4924681803a180ab42035a4ecf.gif"},32350:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/image-Uart-Login-cb4e30ecfc90a5beb73624b4fbb8a6c3.gif"},16956:(e,i,n)=>{n.d(i,{A:()=>r});const r=n.p+"assets/images/linux_login_01-5f7ffb3dfa3ddf2b347194dc06bb0108.gif"},28453:(e,i,n)=>{n.d(i,{R:()=>c,x:()=>t});var r=n(96540);const s={},d=r.createContext(s);function c(e){const i=r.useContext(d);return r.useMemo((function(){return"function"==typeof e?e(i):{...i,...e}}),[i,e])}function t(e){let i;return i=e.disableParentContext?"function"==typeof e.components?e.components(s):e.components||s:c(e.components),r.createElement(d.Provider,{value:i},e.children)}}}]);