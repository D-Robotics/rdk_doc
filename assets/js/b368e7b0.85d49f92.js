"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[4258],{23587:(e,n,r)=>{r.r(n),r.d(n,{assets:()=>o,contentTitle:()=>d,default:()=>h,frontMatter:()=>t,metadata:()=>s,toc:()=>c});var l=r(74848),i=r(28453);const t={sidebar_position:9},d="Thermal \u7cfb\u7edf",s={id:"Advanced_development/linux_development/driver_development_x5/driver_thermal_dev",title:"Thermal \u7cfb\u7edf",description:"\u6e29\u5ea6\u4f20\u611f\u5668",source:"@site/docs/07_Advanced_development/02_linux_development/driver_development_x5/driver_thermal_dev.md",sourceDirName:"07_Advanced_development/02_linux_development/driver_development_x5",slug:"/Advanced_development/linux_development/driver_development_x5/driver_thermal_dev",permalink:"/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_thermal_dev",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/07_Advanced_development/02_linux_development/driver_development_x5/driver_thermal_dev.md",tags:[],version:"current",sidebarPosition:9,frontMatter:{sidebar_position:9},sidebar:"tutorialSidebar",previous:{title:"PWM \u9a71\u52a8\u8c03\u8bd5\u6307\u5357",permalink:"/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_pwm"},next:{title:"\u4f4e\u529f\u8017\u6a21\u5f0f\u8c03\u8bd5\u6307\u5357",permalink:"/rdk_doc/Advanced_development/linux_development/driver_development_x5/driver_lowpower"}},o={},c=[{value:"\u6e29\u5ea6\u4f20\u611f\u5668",id:"\u6e29\u5ea6\u4f20\u611f\u5668",level:2},{value:"Thermal",id:"thermal",level:2},{value:"thermal\u53c2\u8003\u6587\u6863",id:"thermal\u53c2\u8003\u6587\u6863",level:2}];function p(e){const n={code:"code",h1:"h1",h2:"h2",li:"li",p:"p",pre:"pre",ul:"ul",...(0,i.R)(),...e.components};return(0,l.jsxs)(l.Fragment,{children:[(0,l.jsx)(n.h1,{id:"thermal-\u7cfb\u7edf",children:"Thermal \u7cfb\u7edf"}),"\n",(0,l.jsx)(n.h2,{id:"\u6e29\u5ea6\u4f20\u611f\u5668",children:"\u6e29\u5ea6\u4f20\u611f\u5668"}),"\n",(0,l.jsx)(n.p,{children:"\u5728X5\u4e0a\u6709\u4e09\u4e2a\u6e29\u5ea6\u4f20\u611f\u5668\uff0c\u7528\u4e8e\u663e\u793aDDR/BPU/CPU\u7684\u6e29\u5ea6 \u5728/sys/class/hwmon/\u4e0b\u6709hwmon0\u76ee\u5f55\u4e0b\u5305\u542b\u6e29\u5ea6\u4f20\u611f\u5668\u7684\u76f8\u5173\u53c2\u6570 temp1_input\u662fDDR\u7684\u6e29\u5ea6\uff0ctemp2_input\u662fBPU\u7684\u6e29\u5ea6\uff0ctemp3_input\u662fCPU\u7684\u6e29\u5ea6 \u6e29\u5ea6\u7684\u7cbe\u5ea6\u4e3a0.001\u6444\u6c0f\u5ea6"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"cat /sys/class/hwmon/hwmon0/temp1_input\n46643\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u6ce8\uff1aBPU\u7684\u6e29\u5ea6\u4f20\u611f\u5668\u4f4d\u4e8ebpu subsytem\uff0cbpu subsystem\u53ea\u6709\u5728bpu\u8fd0\u884c\u65f6\u624d\u4f1a\u4e0a\u7535\uff0c\u6240\u4ee5\u53ea\u6709bpu\u8fd0\u884c\u65f6\uff0cbpu\u7684\u6e29\u5ea6\u624d\u53ef\u4ee5\u67e5\u770b\u3002"}),"\n",(0,l.jsx)(n.h2,{id:"thermal",children:"Thermal"}),"\n",(0,l.jsx)(n.p,{children:"Linux Thermal \u662f Linux \u7cfb\u7edf\u4e0b\u6e29\u5ea6\u63a7\u5236\u76f8\u5173\u7684\u6a21\u5757\uff0c\u4e3b\u8981\u7528\u6765\u63a7\u5236\u7cfb\u7edf\u8fd0\u884c\u8fc7\u7a0b\u4e2d\u82af\u7247\u4ea7\u751f\u7684\u70ed\u91cf\uff0c\u4f7f\u82af\u7247\u6e29\u5ea6\u548c\u8bbe\u5907\u5916\u58f3\u6e29\u5ea6\u7ef4\u6301\u5728\u4e00\u4e2a\u5b89\u5168\u3001\u8212\u9002\u7684\u8303\u56f4\u3002"}),"\n",(0,l.jsx)(n.p,{children:"\u8981\u60f3\u8fbe\u5230\u5408\u7406\u63a7\u5236\u8bbe\u5907\u6e29\u5ea6\uff0c\u6211\u4eec\u9700\u8981\u4e86\u89e3\u4ee5\u4e0b\u4e09\u4e2a\u6a21\u5757\uff1a"}),"\n",(0,l.jsxs)(n.ul,{children:["\n",(0,l.jsx)(n.li,{children:"\u83b7\u53d6\u6e29\u5ea6\u7684\u8bbe\u5907\uff1a\u5728 Thermal \u6846\u67b6\u4e2d\u88ab\u62bd\u8c61\u4e3a Thermal Zone Device\uff0cX5\u4e0a\u6709\u4e24\u4e2athermal zone\uff0c\u5206\u522b\u662fthermal_zone0\u548cthermal_zone1\uff1b"}),"\n",(0,l.jsx)(n.li,{children:"\u9700\u8981\u964d\u6e29\u7684\u8bbe\u5907\uff1a\u5728 Thermal \u6846\u67b6\u4e2d\u88ab\u62bd\u8c61\u4e3a Thermal Cooling Device\uff0c\u6709CPU\u3001BPU\u3001GPU\u548cDDR\uff1b"}),"\n",(0,l.jsx)(n.li,{children:"\u63a7\u5236\u6e29\u5ea6\u7b56\u7565\uff1a\u5728 Thermal \u6846\u67b6\u4e2d\u88ab\u62bd\u8c61\u4e3a Thermal Governor;"}),"\n"]}),"\n",(0,l.jsx)(n.p,{children:"\u4ee5\u4e0a\u6a21\u5757\u7684\u4fe1\u606f\u548c\u63a7\u5236\u90fd\u53ef\u4ee5\u5728 /sys/class/thermal \u76ee\u5f55\u4e0b\u83b7\u53d6\u3002"}),"\n",(0,l.jsx)(n.p,{children:"\u5728X5\u91cc\u9762\u4e00\u5171\u6709\u56db\u4e2acooling(\u964d\u6e29)\u8bbe\u5907\uff1a"}),"\n",(0,l.jsxs)(n.ul,{children:["\n",(0,l.jsx)(n.li,{children:"cooling_device0: cpu"}),"\n",(0,l.jsx)(n.li,{children:"cooling_device1: bpu"}),"\n",(0,l.jsx)(n.li,{children:"cooling_device2: gpu"}),"\n",(0,l.jsx)(n.li,{children:"cooling_device3: ddr"}),"\n"]}),"\n",(0,l.jsx)(n.p,{children:"\u5176\u4e2d\uff0ccooling\u8bbe\u5907DDR\u4e0ethermal_zone0\u5173\u8054\uff0ccooling \u8bbe\u5907CPU/BPU/GPU\u4e0ethermal_zone1\u5173\u8054\u3002 \u76ee\u524d\u9ed8\u8ba4\u7684\u7b56\u7565\u901a\u8fc7\u4ee5\u4e0b\u547d\u4ee4\u53ef\u77e5\u662f\u4f7f\u7528\u7684 step_wise\u3002"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"cat /sys/class/thermal/thermal_zone0/policy\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u901a\u8fc7\u4ee5\u4e0b\u547d\u4ee4\u53ef\u770b\u5230\u652f\u6301\u7684\u7b56\u7565\uff1auser_space\u3001step_wise\u4e00\u5171\u4e24\u79cd\u3002"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"cat /sys/class/thermal/thermal_zone0/available_policies\n"})}),"\n",(0,l.jsxs)(n.ul,{children:["\n",(0,l.jsx)(n.li,{children:"user_space \u662f\u901a\u8fc7uevent\u5c06\u6e29\u533a\u5f53\u524d\u6e29\u5ea6\uff0c\u6e29\u63a7\u89e6\u53d1\u70b9\u7b49\u4fe1\u606f\u4e0a\u62a5\u5230\u7528\u6237\u7a7a\u95f4\uff0c\u7531\u7528\u6237\u7a7a\u95f4\u8f6f\u4ef6\u5236\u5b9a\u6e29\u63a7\u7684\u7b56\u7565\u3002"}),"\n",(0,l.jsx)(n.li,{children:"step_wise \u662f\u6bcf\u4e2a\u8f6e\u8be2\u5468\u671f\u9010\u7ea7\u63d0\u9ad8\u51b7\u5374\u72b6\u6001\uff0c\u662f\u4e00\u79cd\u76f8\u5bf9\u6e29\u548c\u7684\u6e29\u63a7\u7b56\u7565"}),"\n"]}),"\n",(0,l.jsx)(n.p,{children:"\u5177\u4f53\u9009\u62e9\u54ea\u79cd\u7b56\u7565\u662f\u6839\u636e\u4ea7\u54c1\u9700\u8981\u81ea\u5df1\u9009\u62e9\u3002\u53ef\u5728\u7f16\u8bd1\u7684\u65f6\u5019\u6307\u5b9a\u6216\u8005\u901a\u8fc7sysfs\u52a8\u6001\u5207\u6362\u3002 \u4f8b\u5982\uff1a\u52a8\u6001\u5207\u6362thermal_zone0\u7684\u7b56\u7565\u4e3a user_space\u6a21\u5f0f"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"echo user_space > /sys/class/thermal/thermal_zone0/policy\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u5728thermal_zone0\u4e2d\u67091\u4e2atrip_point\uff0c\u7528\u4e8e\u63a7\u5236cooling\u8bbe\u5907DDR\u7684\u8c03\u9891\u6e29\u5ea6"}),"\n",(0,l.jsx)(n.p,{children:"\u53ef\u901a\u8fc7sysfs\u67e5\u770bDDR\u7684\u8c03\u9891\u6e29\u5ea6\uff0c\u5f53\u524d\u914d\u7f6e\u7684\u4e3a95\u5ea6"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"cat /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u82e5\u60f3\u8c03\u6574DDR\u7684\u8c03\u9891\u6e29\u5ea6\uff0c\u598285\u5ea6\uff0c\u53ef\u901a\u8fc7\u5982\u4e0b\u547d\u4ee4\uff1a"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"echo 85000 > /sys/devices/virtual/thermal/thermal_zone0/trip_point_0_temp\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u5728thermal_zone1\u4e2d\u67093\u4e2atrip_point\uff0c\u5176\u4e2dtrip_point_0_temp\u4e3a\u9884\u7559\u4f5c\u7528\uff1btrip_point_1_temp\u662f\u8be5thermal zone\u7684\u8c03\u9891\u6e29\u5ea6\uff0c\u53ef\u63a7\u5236CPU/BPU/GPU\u7684\u9891\u7387\uff0c\u5f53\u524d\u8bbe\u7f6e\u4e3a95\u5ea6\u3002trip_point_2_temp\u4e3a\u5173\u673a\u6e29\u5ea6\uff0c\u5f53\u524d\u8bbe\u7f6e\u4e3a105\u5ea6 \u4f8b\u5982\u60f3\u8981\u7ed3\u6e29\u523085\u6444\u6c0f\u5ea6\uff0cCPU/BPU/GPU\u5f00\u59cb\u8c03\u9891\uff1a"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"echo 85000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_1_temp\n"})}),"\n",(0,l.jsx)(n.p,{children:"\u5982\u679c\u60f3\u8981\u8c03\u6574\u5173\u673a\u6e29\u5ea6\u4e3a105\u6444\u6c0f\u5ea6\uff1a"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"echo 105000 > /sys/devices/virtual/thermal/thermal_zone1/trip_point_2_temp\n"})}),"\n",(0,l.jsxs)(n.p,{children:[(0,l.jsx)("font",{color:"red",children:"\u6ce8\u610f\uff1a"}),"\u4ee5\u4e0a\u8bbe\u7f6e\u65ad\u7535\u91cd\u542f\u540e\u9700\u8981\u91cd\u65b0\u8bbe\u7f6e"]}),"\n",(0,l.jsx)(n.h2,{id:"thermal\u53c2\u8003\u6587\u6863",children:"thermal\u53c2\u8003\u6587\u6863"}),"\n",(0,l.jsx)(n.p,{children:"\u4ee5\u4e0b\u8def\u5f84\u4ee5kernel\u4ee3\u7801\u76ee\u5f55\u4e3a\u6839\u76ee\u5f55\u3002"}),"\n",(0,l.jsx)(n.pre,{children:(0,l.jsx)(n.code,{children:"./Documentation/devicetree/bindings/thermal\n./Documentation/driver-api/thermal\n"})})]})}function h(e={}){const{wrapper:n}={...(0,i.R)(),...e.components};return n?(0,l.jsx)(n,{...e,children:(0,l.jsx)(p,{...e})}):p(e)}},28453:(e,n,r)=>{r.d(n,{R:()=>d,x:()=>s});var l=r(96540);const i={},t=l.createContext(i);function d(e){const n=l.useContext(t);return l.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function s(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:d(e.components),l.createElement(t.Provider,{value:n},e.children)}}}]);