"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[9814],{97017:(e,r,n)=>{n.r(r),n.d(r,{assets:()=>c,contentTitle:()=>a,default:()=>u,frontMatter:()=>o,metadata:()=>d,toc:()=>g});var t=n(74848),i=n(28453),s=n(93859),l=n(19365);const o={sidebar_position:7},a="5.2.7 \u5de5\u5177",d={id:"Robot_development/quick_demo/demo_tool",title:"5.2.7 \u5de5\u5177",description:"\u56fe\u50cf\u53d1\u5e03\u5de5\u5177",source:"@site/docs/05_Robot_development/02_quick_demo/demo_tool.md",sourceDirName:"05_Robot_development/02_quick_demo",slug:"/Robot_development/quick_demo/demo_tool",permalink:"/rdk_doc/Robot_development/quick_demo/demo_tool",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/05_Robot_development/02_quick_demo/demo_tool.md",tags:[],version:"current",sidebarPosition:7,frontMatter:{sidebar_position:7},sidebar:"tutorialSidebar",previous:{title:"5.2.6 \u6a21\u578b\u63a8\u7406",permalink:"/rdk_doc/Robot_development/quick_demo/ai_predict"},next:{title:"5.2.8 \u6587\u672c\u8f6c\u8bed\u97f3",permalink:"/rdk_doc/Robot_development/quick_demo/hobot_tts"}},c={},g=[{value:"\u56fe\u50cf\u53d1\u5e03\u5de5\u5177",id:"\u56fe\u50cf\u53d1\u5e03\u5de5\u5177",level:2},{value:"\u529f\u80fd\u4ecb\u7ecd",id:"\u529f\u80fd\u4ecb\u7ecd",level:3},{value:"\u652f\u6301\u5e73\u53f0",id:"\u652f\u6301\u5e73\u53f0",level:3},{value:"\u51c6\u5907\u5de5\u4f5c",id:"\u51c6\u5907\u5de5\u4f5c",level:3},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0",level:4},{value:"X86\u5e73\u53f0",id:"x86\u5e73\u53f0",level:4},{value:"\u56fe\u7247\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",id:"\u56fe\u7247\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",level:3},{value:"RDK/X86\u5e73\u53f0",id:"rdkx86\u5e73\u53f0",level:4},{value:"\u56fe\u7247\u53d1\u5e03\u7ed3\u679c\u5206\u6790",id:"\u56fe\u7247\u53d1\u5e03\u7ed3\u679c\u5206\u6790",level:3},{value:"\u89c6\u9891\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",id:"\u89c6\u9891\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",level:3},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0-1",level:4},{value:"X86\u5e73\u53f0",id:"x86\u5e73\u53f0-1",level:4},{value:"\u89c6\u9891\u53d1\u5e03\u7ed3\u679c\u5206\u6790",id:"\u89c6\u9891\u53d1\u5e03\u7ed3\u679c\u5206\u6790",level:3},{value:"Trigger\u8bb0\u5f55\u5de5\u5177",id:"trigger\u8bb0\u5f55\u5de5\u5177",level:2},{value:"\u529f\u80fd\u4ecb\u7ecd",id:"\u529f\u80fd\u4ecb\u7ecd-1",level:3},{value:"\u652f\u6301\u5e73\u53f0",id:"\u652f\u6301\u5e73\u53f0-1",level:3},{value:"\u4f7f\u7528\u8bf4\u660e",id:"\u4f7f\u7528\u8bf4\u660e",level:3},{value:"Trigger\u521d\u59cb\u5316\u914d\u7f6e\u8bf4\u660e",id:"trigger\u521d\u59cb\u5316\u914d\u7f6e\u8bf4\u660e",level:4},{value:"Trigger\u4e8b\u4ef6\u89e6\u53d1\u914d\u7f6e\u8bf4\u660e",id:"trigger\u4e8b\u4ef6\u89e6\u53d1\u914d\u7f6e\u8bf4\u660e",level:4},{value:"\u51c6\u5907\u5de5\u4f5c",id:"\u51c6\u5907\u5de5\u4f5c-1",level:3},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0-2",level:4},{value:"\u4f7f\u7528\u4ecb\u7ecd",id:"\u4f7f\u7528\u4ecb\u7ecd",level:3},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0-3",level:4},{value:"\u7ed3\u679c\u5206\u6790",id:"\u7ed3\u679c\u5206\u6790",level:3},{value:"\u62d3\u5c55\u529f\u80fd",id:"\u62d3\u5c55\u529f\u80fd",level:3},{value:"\u7ed9Trigger\u6a21\u5757\u4e0b\u53d1\u4efb\u52a1",id:"\u7ed9trigger\u6a21\u5757\u4e0b\u53d1\u4efb\u52a1",level:4},{value:"Trigger\u4efb\u52a1\u534f\u8bae",id:"trigger\u4efb\u52a1\u534f\u8bae",level:5},{value:"\u8fd0\u884c",id:"\u8fd0\u884c",level:5},{value:"\u65e5\u5fd7\u4fe1\u606f",id:"\u65e5\u5fd7\u4fe1\u606f",level:5}];function h(e){const r={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",h3:"h3",h4:"h4",h5:"h5",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",...(0,i.R)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsx)(r.h1,{id:"527-\u5de5\u5177",children:"5.2.7 \u5de5\u5177"}),"\n","\n",(0,t.jsx)(r.h2,{id:"\u56fe\u50cf\u53d1\u5e03\u5de5\u5177",children:"\u56fe\u50cf\u53d1\u5e03\u5de5\u5177"}),"\n",(0,t.jsx)(r.h3,{id:"\u529f\u80fd\u4ecb\u7ecd",children:"\u529f\u80fd\u4ecb\u7ecd"}),"\n",(0,t.jsx)(r.p,{children:"\u56fe\u7247\u53d1\u5e03\u5de5\u5177\u652f\u6301\u6279\u91cf\u8bfb\u53d6\u672c\u5730\u56fe\u7247\u6216\u89c6\u9891\u6587\u4ef6\uff0c\u5e76\u6309\u7167ROS\u6d88\u606f\u683c\u5f0f\u53d1\u5e03\uff0c\u4ece\u800c\u63d0\u9ad8\u7b97\u6cd5\u8c03\u8bd5\u548c\u90e8\u7f72\u6548\u7387\u3002"}),"\n",(0,t.jsx)(r.p,{children:"\u5bf9\u4e8e\u56fe\u7247\u53d1\u5e03\uff0c\u652f\u6301\u8bfb\u53d6JPEG/JPG/PNG/NV12\u683c\u5f0f\u7684\u56fe\u7247\uff0c\u53d1\u5e03\u538b\u7f29\u56fe\u7247\u6216\u8005\u5c06\u538b\u7f29\u56fe\u7247\u8f6c\u6362\u4e3aNV12\u683c\u5f0f\u8fdb\u884c\u53d1\u5e03\u3002"}),"\n",(0,t.jsx)(r.p,{children:"\u5bf9\u4e8e\u89c6\u9891\u53d1\u5e03\uff0c\u652f\u6301H264/H265/MP4\u683c\u5f0f\uff0c\u8bfb\u53d6\u89c6\u9891\u6587\u4ef6\u540e\u63d0\u53d6\u76f8\u5173\u7684\u89c6\u9891\u6d41\u8fdb\u884c\u53d1\u5e03\u3002"}),"\n",(0,t.jsxs)(r.p,{children:["\u4ee3\u7801\u4ed3\u5e93: (",(0,t.jsx)(r.a,{href:"https://github.com/D-Robotics/hobot_image_publisher.git",children:"https://github.com/D-Robotics/hobot_image_publisher.git"}),")"]}),"\n",(0,t.jsx)(r.h3,{id:"\u652f\u6301\u5e73\u53f0",children:"\u652f\u6301\u5e73\u53f0"}),"\n",(0,t.jsxs)(r.table,{children:[(0,t.jsx)(r.thead,{children:(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.th,{children:"\u5e73\u53f0"}),(0,t.jsx)(r.th,{children:"\u8fd0\u884c\u65b9\u5f0f"})]})}),(0,t.jsxs)(r.tbody,{children:[(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.td,{children:"RDK X3, RDK X3 Module"}),(0,t.jsx)(r.td,{children:"Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)"})]}),(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.td,{children:"RDK X5"}),(0,t.jsx)(r.td,{children:"Ubuntu 22.04 (Humble)"})]}),(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.td,{children:"RDK Ultra"}),(0,t.jsx)(r.td,{children:"Ubuntu 20.04 (Foxy)"})]}),(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.td,{children:"X86"}),(0,t.jsx)(r.td,{children:"Ubuntu 20.04 (Foxy)"})]})]})]}),"\n",(0,t.jsx)(r.admonition,{type:"caution",children:(0,t.jsx)(r.p,{children:"X86\u5e73\u53f0\u4e0d\u652f\u6301\u5c06H.264\u3001H.265\u89c6\u9891\u89e3\u7801\u4e3aNV12\u683c\u5f0f\uff0c\u56e0\u6b64H.264\u3001H.265\u89c6\u9891\u53d1\u5e03\u529f\u80fd\u65e0\u6cd5\u5728X86\u5e73\u53f0\u5c55\u793a\u3002\nRDK Ultra\u4e0d\u652f\u6301\u5c06H.264\u89c6\u9891\u89e3\u7801\u4e3aNV12\u683c\u5f0f\uff0c\u56e0\u6b64H.264\u89c6\u9891\u53d1\u5e03\u529f\u80fd\u65e0\u6cd5\u5728RDK Ultra\u5e73\u53f0\u5c55\u793a\u3002"})}),"\n",(0,t.jsx)(r.h3,{id:"\u51c6\u5907\u5de5\u4f5c",children:"\u51c6\u5907\u5de5\u4f5c"}),"\n",(0,t.jsx)(r.h4,{id:"rdk\u5e73\u53f0",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsxs)(r.ol,{children:["\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"RDK\u5df2\u70e7\u5f55\u597dUbuntu 20.04/Ubuntu 22.04\u7cfb\u7edf\u955c\u50cf"}),"\n"]}),"\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"RDK\u5df2\u6210\u529f\u5b89\u88c5tros.b"}),"\n"]}),"\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"\u53ef\u4ee5\u901a\u8fc7\u7f51\u7edc\u8bbf\u95eeRDK\u7684PC"}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(r.h4,{id:"x86\u5e73\u53f0",children:"X86\u5e73\u53f0"}),"\n",(0,t.jsxs)(r.ol,{children:["\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"X86\u73af\u5883\u5df2\u914d\u7f6eUbuntu 20.04\u7cfb\u7edf\u955c\u50cf"}),"\n"]}),"\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"X86\u73af\u5883\u5df2\u5b89\u88c5X86\u7248\u672c tros.b"}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(r.h3,{id:"\u56fe\u7247\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",children:"\u56fe\u7247\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd"}),"\n",(0,t.jsx)(r.p,{children:"\u5faa\u73af\u8bfb\u53d6\u672c\u5730\u7684\u4e00\u5f20NV12\u683c\u5f0f\u56fe\u7247\u5e76\u53d1\u5e03\uff0c\u4f7f\u7528\u56fe\u50cf\u7f16\u89e3\u7801\u6a21\u5757\u5c06\u56fe\u7247\u538b\u7f29\u7f16\u7801\u6210JPEG\u683c\u5f0f\uff0c\u5728PC\u7684Web\u7aef\u5c55\u793a\u56fe\u7247\u3002"}),"\n",(0,t.jsx)(r.h4,{id:"rdkx86\u5e73\u53f0",children:"RDK/X86\u5e73\u53f0"}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:"# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u56fe\u7247\u6587\u4ef6\ncp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch hobot_image_publisher hobot_image_publisher_demo.launch.py\n"})}),"\n",(0,t.jsx)(r.h3,{id:"\u56fe\u7247\u53d1\u5e03\u7ed3\u679c\u5206\u6790",children:"\u56fe\u7247\u53d1\u5e03\u7ed3\u679c\u5206\u6790"}),"\n",(0,t.jsx)(r.p,{children:"\u5728\u8fd0\u884c\u7ec8\u7aef\u8f93\u51fa\u5982\u4e0b\u4fe1\u606f\uff1a"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-text",children:"[INFO] [launch]: All log files can be found below /root/.ros/log/2022-08-19-12-58-02-288516-ubuntu-24492\n[INFO] [launch]: Default logging verbosity is set to INFO\nwebserver has launch\n[INFO] [hobot_image_pub-1]: process started with pid [24511]\n[INFO] [hobot_codec_republish-2]: process started with pid [24513]\n[INFO] [websocket-3]: process started with pid [24519]\n"})}),"\n",(0,t.jsx)(r.p,{children:"\u8f93\u51falog\u663e\u793a\u51fawebserver\u5df2\u542f\u52a8\uff0chobot_image_pub\u3001hobot_codec_republish\u3001websocket\u90fd\u6b63\u5e38\u8fd0\u884c"}),"\n",(0,t.jsxs)(r.p,{children:["\u5728PC\u7aef\u7684\u6d4f\u89c8\u5668\u8f93\u5165 ",(0,t.jsx)(r.code,{children:"http://IP:8000"})," \u5373\u53ef\u67e5\u770b\u56fe\u50cf\u5c55\u793a\u6548\u679c\uff08IP\u4e3aRDK/X86\u8bbe\u5907\u7684IP\u5730\u5740\uff09\uff1a"]}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.img,{alt:"hobot_img_pub",src:n(8519).A+"",width:"1862",height:"941"})}),"\n",(0,t.jsx)(r.h3,{id:"\u89c6\u9891\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd",children:"\u89c6\u9891\u53d1\u5e03\u4f7f\u7528\u4ecb\u7ecd"}),"\n",(0,t.jsx)(r.p,{children:"\u8bfb\u53d6\u672c\u5730video.list\u6587\u4ef6\uff0c\u83b7\u53d6list\u6587\u4ef6\u4e2d\u7684\u89c6\u9891\u6587\u4ef6\u8def\u5f84\uff0c\u5faa\u73af\u8bfb\u53d6\u89c6\u9891\u6587\u4ef6\u5e76\u53d1\u5e03\uff0c\u5148\u4f7f\u7528\u56fe\u50cf\u7f16\u89e3\u7801\u6a21\u5757\u5c06\u89c6\u9891\u6d41\u89e3\u7801\u6210NV12\u683c\u5f0f\u56fe\u7247\uff0c\u518d\u4f7f\u7528\u56fe\u50cf\u7f16\u89e3\u7801\u6a21\u5757\u5c06\u56fe\u7247\u538b\u7f29\u7f16\u7801\u6210JPEG\u683c\u5f0f\uff0c\u5728PC\u7684Web\u7aef\u5c55\u793a\u56fe\u7247\u3002"}),"\n",(0,t.jsx)(r.h4,{id:"rdk\u5e73\u53f0-1",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:"# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u56fe\u7247\u6587\u4ef6\ncp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch hobot_image_publisher hobot_image_publisher_videolist_demo.launch.py\n"})}),"\n",(0,t.jsx)(r.h4,{id:"x86\u5e73\u53f0-1",children:"X86\u5e73\u53f0"}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:"# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u56fe\u7247\u6587\u4ef6\ncp -r /opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/config/ .\n\n# \u542f\u52a8\u56fe\u7247\u53d1\u5e03\u8282\u70b9\uff0c\u4f7f\u7528\u672c\u5730MP4\u683c\u5f0f\u89c6\u9891\u6587\u4ef6\u8fdb\u884c\u53d1\u5e03\uff08\u53ef\u4ee5\u6839\u636e\u81ea\u5df1\u7684\u9700\u6c42\u8fdb\u884c\u53c2\u6570\u914d\u7f6e\uff09\uff0c\u6682\u4e0d\u652f\u6301Web\u7aef\u663e\u793a\n/opt/tros/${TROS_DISTRO}/lib/hobot_image_publisher/hobot_image_pub --ros-args -p image_source:=./config/video.list -p fps:=30 -p image_format:=mp4\n"})}),"\n",(0,t.jsx)(r.h3,{id:"\u89c6\u9891\u53d1\u5e03\u7ed3\u679c\u5206\u6790",children:"\u89c6\u9891\u53d1\u5e03\u7ed3\u679c\u5206\u6790"}),"\n",(0,t.jsx)(r.p,{children:"\u5728\u8fd0\u884c\u7ec8\u7aef\u8f93\u51fa\u5982\u4e0b\u4fe1\u606f\uff1a"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-text",children:"[INFO] [launch]: All log files can be found below /root/.ros/log/2022-10-22-21-44-03-663907-ubuntu-702475\n[INFO] [launch]: Default logging verbosity is set to INFO\nwebserver has launch\n[INFO] [hobot_image_pub-1]: process started with pid [702597]\n[INFO] [hobot_codec_republish-2]: process started with pid [702599]\n[INFO] [hobot_codec_republish-3]: process started with pid [702601]\n[INFO] [websocket-4]: process started with pid [702603]\n"})}),"\n",(0,t.jsx)(r.p,{children:"\u8f93\u51falog\u663e\u793a\u51fawebserver\u5df2\u542f\u52a8\uff0chobot_image_pub\u3001hobot_codec_republish\u3001websocket\u90fd\u6b63\u5e38\u8fd0\u884c"}),"\n",(0,t.jsxs)(r.p,{children:["\u5728PC\u7aef\u7684\u6d4f\u89c8\u5668\u8f93\u5165 ",(0,t.jsx)(r.code,{children:"http://IP:8000"})," \u5373\u53ef\u67e5\u770b\u56fe\u50cf\u5c55\u793a\u6548\u679c\uff08IP\u4e3aRDK/X86\u8bbe\u5907\u7684IP\u5730\u5740\uff09\uff1a"]}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.img,{alt:"hobot_img_pub",src:n(49528).A+"",width:"1670",height:"934"})}),"\n",(0,t.jsx)(r.h2,{id:"trigger\u8bb0\u5f55\u5de5\u5177",children:"Trigger\u8bb0\u5f55\u5de5\u5177"}),"\n",(0,t.jsx)(r.h3,{id:"\u529f\u80fd\u4ecb\u7ecd-1",children:"\u529f\u80fd\u4ecb\u7ecd"}),"\n",(0,t.jsx)(r.p,{children:"\u6240\u8c13Trigger\uff0c\u662f\u5728\u8bbe\u5b9a\u597d\u5df2\u6709Trigger\u673a\u5236\u57fa\u7840\u4e0a\uff0c\u76d1\u6d4bTrigger\u6a21\u5757\u8ba2\u9605\u7684\u6d88\u606f\u53d8\u5316\uff0c\u4f8b\u5982\u68c0\u6d4b\u6846\u7ed3\u679c\u6570\u91cf\u53d8\u5316\uff0c\u5c0f\u8f66\u63a7\u5236\u4fe1\u606f\u53d8\u5316\u7b49\uff0c\u89e6\u53d1\u5bf9\u5e94Trigger\u4e8b\u4ef6\uff0c\u8bb0\u5f55\u6307\u5b9a\u65f6\u95f4\u533a\u95f4\u5185\u7684ROS2\u6d88\u606f\uff0c\u4ece\u800c\u5e2e\u52a9\u5f00\u53d1\u4eba\u5458\u5b9a\u4f4d\u548c\u590d\u73b0\u673a\u5668\u4eba\u573a\u666f\u4e2d\u7684\u611f\u77e5\u3001\u89c4\u63a7\u7b49\u95ee\u9898\u3002"}),"\n",(0,t.jsx)(r.p,{children:"trigger_node package \u662fD-Robotics\u57fa\u4e8eROS2\u5f00\u53d1\u7684Trigger\u57fa\u7840\u6a21\u5757\uff0c\u7528\u4e8e\u5728\u89e6\u53d1Trigger\u4e8b\u4ef6\u540e\uff0c\u83b7\u53d6\u6307\u5b9arosbag\u6570\u636e\u7684\u529f\u80fd\u5305\u3002package\u652f\u6301\u76f4\u63a5\u8ba2\u9605ai_msg/msg/PerceptionTargets\u7c7b\u578b\u7684\u8bdd\u9898\uff0c\u5728\u8bdd\u9898\u56de\u8c03\u51fd\u6570\u4e2d\uff0c\u5224\u65ad\u662f\u5426\u89e6\u53d1Trigger\u4e8b\u4ef6\uff0c\u5e76\u8bb0\u5f55Trigger\u4e8b\u4ef6\u76f8\u5173\u7684rosbag\u5305\uff0c\u6700\u540e\u5c06Trigger\u4e8b\u4ef6\u4fe1\u606f\u4fdd\u5b58\uff0c\u5e76\u53d1\u5e03std_msg/msg/String\u7c7b\u578b\u7684Trigger\u4e8b\u4ef6\u8bdd\u9898\u3002"}),"\n",(0,t.jsx)(r.p,{children:"\u672c\u7ae0\u8282\u5c55\u793a\u7684\u793a\u4f8b\uff0c\u662fD-Robotics\u5728\u81ea\u5b9a\u4e49trigger\u57fa\u7840\u6a21\u5757\u57fa\u7840\u4e0a\uff0c\u5f00\u53d1\u7684Trigger\u6a21\u5757\u4f7f\u7528\u793a\u4f8b\u3002\u672c\u793a\u4f8b\u5c55\u793a\u7684\u529f\u80fd\uff0c\u662f\u8ba2\u9605\u5783\u573e\u68c0\u6d4b\u6846\u4fe1\u606f\uff0c\u6839\u636e\u5783\u573e\u68c0\u6d4b\u6846\u7684\u6570\u91cf\u662f\u5426\u5927\u4e8e\u7b49\u4e8e3\uff0c\u5224\u65ad\u662f\u5426\u89e6\u53d1Trigger\u4e8b\u4ef6\u3002\u82e5\u68c0\u6d4b\u6846\u6570\u91cf\u5927\u4e8e\u7b49\u4e8e3\uff0c\u5219\u89e6\u53d1Trigger\u4e8b\u4ef6\u3002"}),"\n",(0,t.jsxs)(r.p,{children:["\u4ee3\u7801\u4ed3\u5e93\uff1a(",(0,t.jsx)(r.a,{href:"https://github.com/D-Robotics/hobot_trigger.git",children:"https://github.com/D-Robotics/hobot_trigger.git"}),")"]}),"\n",(0,t.jsx)(r.p,{children:"\u5e94\u7528\u573a\u666f\uff1a\u673a\u5668\u4eba\u6570\u636e\u95ed\u73af\u94fe\u8def\uff0c\u673a\u5668\u4ebaTrigger\u4e8b\u4ef6\u4e0a\u62a5\u573a\u666f\uff0c\u53ef\u914d\u5408\u611f\u77e5\u3001\u89c4\u63a7\u7b49\u4efb\u52a1\uff0c\u8bb0\u5f55Trigger\u4e8b\u4ef6\u53d1\u751f\u65f6\u7684rosbag\u6570\u636e\u3002"}),"\n",(0,t.jsx)(r.h3,{id:"\u652f\u6301\u5e73\u53f0-1",children:"\u652f\u6301\u5e73\u53f0"}),"\n",(0,t.jsxs)(r.table,{children:[(0,t.jsx)(r.thead,{children:(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.th,{children:"\u5e73\u53f0"}),(0,t.jsx)(r.th,{children:"\u8fd0\u884c\u65b9\u5f0f"}),(0,t.jsx)(r.th,{children:"\u793a\u4f8b\u529f\u80fd"})]})}),(0,t.jsx)(r.tbody,{children:(0,t.jsxs)(r.tr,{children:[(0,t.jsx)(r.td,{children:"RDK X3, RDK X3 Module"}),(0,t.jsx)(r.td,{children:"Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)"}),(0,t.jsx)(r.td,{children:"\xb7 \u542f\u52a8MIPI/USB\u6444\u50cf\u5934\uff0c\u89e6\u53d1\u8bb0\u5f55\u7684rosbag\u6570\u636e\u8bb0\u5f55\u5728\u672c\u5730"})]})})]}),"\n",(0,t.jsx)(r.h3,{id:"\u4f7f\u7528\u8bf4\u660e",children:"\u4f7f\u7528\u8bf4\u660e"}),"\n",(0,t.jsx)(r.h4,{id:"trigger\u521d\u59cb\u5316\u914d\u7f6e\u8bf4\u660e",children:"Trigger\u521d\u59cb\u5316\u914d\u7f6e\u8bf4\u660e"}),"\n",(0,t.jsx)(r.p,{children:"Trigger\u57fa\u7840\u6a21\u5757\uff0c\u5b9a\u4e49\u4e86\u521d\u59cb\u5316\u914d\u7f6e\u9700\u8981\u7684\u53c2\u6570\u3002"}),"\n",(0,t.jsx)(r.p,{children:"config_file\u914d\u7f6e\u6587\u4ef6\u683c\u5f0f\u4e3ajson\u683c\u5f0f\uff0c\u5177\u4f53\u914d\u7f6e\u5982\u4e0b\uff1a"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:'{ \n  "domain": Trigger\u4e8b\u4ef6domain\u3002\u5982\u626b\u5730\u673a\u3001\u4eba\u578b\u673a\u7b49\uff0cTrigger\u7c7b\u578b\u4e0d\u540c\uff0c\u901a\u8fc7domain\u533a\u5206\u4e0d\u540c\u9886\u57df\u7c7b\u578b\u673a\u5668\u4ebaTrigger\u3002\n\n  "desc": Trigger\u6a21\u5757\u63cf\u8ff0\u4fe1\u606f\u3002\n\n  "duration_ts_back": \u5f55\u5236Trigger\u53d1\u751f\u540e\u6301\u7eed\u65f6\u957f\u3002\n\n  "duration_ts_front": \u5f55\u5236Tirgger\u53d1\u751f\u524d\u6301\u7eed\u65f6\u957f\u3002\n  \n  "level": Trigger\u4e8b\u4ef6\u7684\u4f18\u5148\u7ea7, \u591a\u4e2a\u4e0d\u540cTrigger\u53d1\u751f\u65f6, \u53ef\u5229\u7528\u4e00\u4e2a\u603b\u8282\u70b9\uff0c\u7b5b\u9009\u4e00\u4e9b\u9ad8\u4f18\u6216\u4f4e\u4f18\u7684Trigger\u4e8b\u4ef6\u3002\n  \n  "src_module_id": \u53d1\u751fTrigger\u7684\u6a21\u5757ID, \u7528\u4e8e\u7ba1\u7406\u4e0d\u540c\u7684Trigger\u6a21\u5757, \u6ee1\u8db3\u4e1a\u52a1\u4e0d\u540cTrigger\u6a21\u5757\u7ba1\u7406\u9700\u6c42\u3002\n  \n  "status": Trigger\u72b6\u6001, \'0\': \u5173\u95ed, \'1\': \u6253\u5f00\u3002\n  \n  "strategy_version": Trigger\u6a21\u5757\u7b56\u7565\u7684\u7248\u672c\u53f7\u3002\n  \n  "topics": \u9700\u8981\u8bb0\u5f55\u7684\u8bdd\u9898list\uff0c\u5305\u542b\u8bdd\u9898\u540d\u3002\n  \n  "trigger_type": Trigger\u7c7b\u578bID\u3002\u6bcf\u4e2aTrigger\u6a21\u5757\u5e76\u4e0d\u662f\u53ea\u6709\u4e00\u79cd\u89e6\u53d1\u60c5\u51b5\uff0c\u6bd4\u5982\u68c0\u6d4b\u52302\u4e2a\u5783\u573e\u89e6\u53d1\u662f\u4e00\u79cd\u7c7b\u578b\uff0c\u68c0\u6d4b\u52303\u4e2a\u5783\u573e\u662f\u4e00\u79cd\u7c7b\u578b\u3002\n  \n  "unique_id": \u8bbe\u5907\u552f\u4e00\u6807\u8bc6\u3002\n  \n  "version": Trigger\u6a21\u5757\u7248\u672c\u4fe1\u606f\u3002\n  \n  "extra_kv": \u5176\u4ed6\u5197\u4f59\u6269\u5c55\u4fe1\u606f\u53ef\u8bb0\u5f55\u5728\u6b64\u3002\n}\n'})}),"\n",(0,t.jsx)(r.h4,{id:"trigger\u4e8b\u4ef6\u89e6\u53d1\u914d\u7f6e\u8bf4\u660e",children:"Trigger\u4e8b\u4ef6\u89e6\u53d1\u914d\u7f6e\u8bf4\u660e"}),"\n",(0,t.jsx)(r.p,{children:"\u5728trigger_node\u57fa\u7c7b\u4e2d\uff0c\u5b9a\u4e49\u4e86Config\u7ed3\u6784\u4f53\uff0c\u5176\u4e2d\u90e8\u5206\u914d\u7f6e\u4e0e\u521d\u59cb\u5316\u65f6Trigger\u914d\u7f6e\u4fdd\u6301\u4e00\u81f4\uff0c\u5269\u4e0b\u5185\u5bb9\u9700\u7531Trigger\u89e6\u53d1\u65f6\u6839\u636e\u5b9e\u9645\u60c5\u51b5\u586b\u5145\u3002"}),"\n",(0,t.jsx)(r.p,{children:'\u7528\u6237\u57fa\u4e8eTrigger_node\u8fdb\u884c\u4e8c\u6b21\u5f00\u53d1\u65f6\uff0c\u4ec5\u9700\u8981\u5728\u6bcf\u6b21Trigger\u53d1\u751f\u65f6\uff0c\u5b9e\u4f8b\u5316\u4e00\u4e2a\u7ed3\u6784\u4f53\u53d8\u91cf\uff0c\u5c06Trigger\u53d1\u751f\u65f6\u7684\u76f8\u5173\u4fe1\u606f\u586b\u5165\u7ed3\u6784\u4f53\u53d8\u91cf\uff0c\u5982 "timestamp"\u3001"gps_pos"\u7b49\uff0c\u9001\u5165Trigger\u4e8b\u4ef6\u8bb0\u5f55\u961f\u5217 "requests_"\u4e2d\u3002'}),"\n",(0,t.jsx)(r.p,{children:"\u5728\u6b64\u57fa\u7840\u4e0a\uff0c\u7528\u6237\u5c31\u53ef\u4ee5\u5f00\u53d1\u81ea\u5b9a\u4e49\u7684Trigger\u6a21\u5757\uff0c\u66f4\u591a\u4fe1\u606f\u8bf7\u5728\u4ee3\u7801\u4ed3\u5e93\u4e2d\u53c2\u8003 trigger_node_example \u7684\u5b9e\u73b0\u65b9\u5f0f\u3002"}),"\n",(0,t.jsxs)(r.p,{children:["\u4ee3\u7801\u4ed3\u5e93\uff1a(",(0,t.jsx)(r.a,{href:"https://github.com/D-Robotics/hobot_trigger.git",children:"https://github.com/D-Robotics/hobot_trigger.git"}),")"]}),"\n",(0,t.jsx)(r.p,{children:"\u7ed3\u6784\u4f53\u4fe1\u606f\u5982\u4e0b\uff1a"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-c++",children:"struct Config {\n  std::string domain;       // Trigger\u4e8b\u4ef6domain\n  std::string desc;         // Trigger\u63cf\u8ff0\u4fe1\u606f\n  long duration_ts_back;    // \u5f55\u5236Trigger \u53d1\u751f\u540e\u6301\u7eed\u65f6\u957f\n  long duration_ts_front;   // \u5f55\u5236tirgger \u53d1\u751f\u524d\u6301\u7eed\u65f6\u957f\n  GPS_POS gps_pos;          // GPS\u5b9a\u4f4d\n  int level;                // \u4f18\u5148\u7ea7\n  std::string rosbag_path;  // Trigger\u53d1\u751f\u540erosbag\u672c\u5730\u6587\u4ef6\u8def\u5f84\n  int src_module_id;        // \u53d1\u751fTrigger\u7684\u6a21\u5757\n  int status;               // Trigger\u72b6\u6001\n  std::string strategy_version; // \u7b56\u7565\u7248\u672c\u53f7\n  long timestamp;           // Trigger\u53d1\u751f\u65f6\u95f4\u6233\n  std::vector<std::string> topics;    // \u9700\u8981\u8bb0\u5f55\u7684\u8bdd\u9898list\uff0c\u5305\u542b\u8bdd\u9898\u540d\u548c\u8bdd\u9898\u7c7b\u578b\n  int trigger_type;         // Trigger\u7c7b\u578b\n  std::string unique_id;    // \u8bbe\u5907\u552f\u4e00\u6807\u8bc6\n  std::string version;      // Trigger\u7248\u672c\u4fe1\u606f\n  std::vector<EXTRA_KV> extra_kv;   // \u989d\u5916\u4fe1\u606f\n};\n"})}),"\n",(0,t.jsx)(r.h3,{id:"\u51c6\u5907\u5de5\u4f5c-1",children:"\u51c6\u5907\u5de5\u4f5c"}),"\n",(0,t.jsx)(r.h4,{id:"rdk\u5e73\u53f0-2",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsxs)(r.ol,{children:["\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"RDK\u5df2\u70e7\u5f55\u597dUbuntu 20.04/Ubuntu 22.04\u7cfb\u7edf\u955c\u50cf\u3002"}),"\n"]}),"\n",(0,t.jsxs)(r.li,{children:["\n",(0,t.jsx)(r.p,{children:"RDK\u5df2\u6210\u529f\u5b89\u88c5TogetheROS.Bot\u3002"}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(r.h3,{id:"\u4f7f\u7528\u4ecb\u7ecd",children:"\u4f7f\u7528\u4ecb\u7ecd"}),"\n",(0,t.jsx)(r.h4,{id:"rdk\u5e73\u53f0-3",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.strong,{children:"\u4f7f\u7528MIPI\u6444\u50cf\u5934\u53d1\u5e03\u56fe\u7247"})}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u5b89\u88c5mcap\u5305\napt install ros-humble-rosbag2-storage-mcap\n\n# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:"# \u4ecetros\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_trash_detection/config/ .\ncp -r /opt/tros/${TROS_DISTRO}/lib/trigger_node_example/config/ .\n\n# \u914d\u7f6eMIPI\u6444\u50cf\u5934\nexport CAM_TYPE=mipi\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch trigger_node_example hobot_trigger_example.launch.py\n"})}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.strong,{children:"\u4f7f\u7528usb\u6444\u50cf\u5934\u53d1\u5e03\u56fe\u7247"})}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:"# \u4ecetros\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_trash_detection/config/ .\ncp -r /opt/tros/${TROS_DISTRO}/lib/trigger_node_example/config/ .\n\n# \u914d\u7f6eUSB\u6444\u50cf\u5934\nexport CAM_TYPE=usb\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch trigger_node_example hobot_trigger_example.launch.py\n"})}),"\n",(0,t.jsx)(r.h3,{id:"\u7ed3\u679c\u5206\u6790",children:"\u7ed3\u679c\u5206\u6790"}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.strong,{children:"\u4f7f\u7528mipi\u6444\u50cf\u5934\u53d1\u5e03\u56fe\u7247"})}),"\n",(0,t.jsx)(r.p,{children:"package\u521d\u59cb\u5316\u540e\uff0c\u5728\u7ec8\u7aef\u8f93\u51fa\u5982\u4e0b\u4fe1\u606f\uff1a"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:'  [INFO] [launch]: All log files can be found below /root/.ros/log/2023-05-13-17-31-53-158704-ubuntu-2981490\n   [INFO] [launch]: Default logging verbosity is set to INFO\n   [INFO] [trigger_node_example-1]: process started with pid [2981766]\n   [trigger_node_example-1] [WARN] [1683970314.850652382] [hobot_trigger]: Parameter:\n   [trigger_node_example-1]  cache_path: /home/hobot/recorder/\n   [trigger_node_example-1]  config_file: config/trigger_config.json\n   [trigger_node_example-1]  format: mcap\n   [trigger_node_example-1]  isRecord(1:record, 0:norecord): 1\n   [trigger_node_example-1]  agent_msg_sub_topic_name: /hobot_agent\n   [trigger_node_example-1]  event_msg_sub_topic_name: /ai_msg_mono2d_trash_detection\n   [trigger_node_example-1]  msg_pub_topic_name: /hobot_trigger\n   [trigger_node_example-1]  config detail: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"","src_module_id":203,"timestamp":-1,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"v1.0.0\\n","version":"v1.0.0\\n"}\n   [trigger_node_example-1] [WARN] [1683970314.893573769] [hobot_trigger]: TriggerNode Init Succeed!\n   [trigger_node_example-1] [WARN] [1683970314.898132256] [example]: TriggerExampleNode Init.\n   [trigger_node_example-1] [WARN] [1683970315.931225440] [example]: Trigger Event!\n   [trigger_node_example-1] [WARN] [1683970322.178604839] [rosbag2_storage_mcap]: no message indices found, falling back to reading in file order\n   [trigger_node_example-1] [WARN] [1683970323.007470033] [hobot_trigger]: Trigger Event Report. Trigger moudle id: 203, type id: 1110\n   [trigger_node_example-1]  Report message: {"domain":"","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":5000,"level":1,"rosbag_path":"trigger/OriginBot002_20230513-173155-931/OriginBot002_20230513-173155-931_0.mcap","src_module_id":203,"timestamp":1683970315931,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection"],"trigger_type":1110,"unique_id":"bot","version":"v1.0.0"}\n\n'})}),"\n",(0,t.jsx)(r.p,{children:'\u8fd0\u884c\u540eTrigger\u89e6\u53d1\u4ea7\u751f\u7684rosbag\u6570\u636e\uff0c\u5c06\u8bb0\u5f55\u5728\u5f53\u524d\u8fd0\u884c\u76ee\u5f55 "trigger" \u76ee\u5f55\u4e0b\u3002\u8bb0\u5f55\u7684rosbag\u6570\u636e\uff0c\u53ef\u4ee5\u5728foxglove\u4e2d\u64ad\u653e\u3002\u5728foxglove\u4e2d\u64ad\u653erosbag\u6587\u4ef6\u7684\u65b9\u6cd5\uff0c\u53ef\u4ee5\u53c2\u8003\u624b\u518c 2.2 \u6570\u636e\u5c55\u793a\u2014\u2014foxglove\u5c55\u793a\u3002'}),"\n",(0,t.jsx)(r.p,{children:"foxglove\u4e2d\u64ad\u653e\u6548\u679c\uff1a"}),"\n",(0,t.jsx)(r.p,{children:(0,t.jsx)(r.img,{src:n(34543).A+"",width:"600",height:"249"})}),"\n",(0,t.jsx)(r.p,{children:"\u8bf4\u660e\uff1a\u8be5Trigger\u793a\u4f8b\u8bb0\u5f55\u4e86\u4e8b\u4ef6\u53d1\u751f\u524d5s\u548c\u4e8b\u4ef6\u53d1\u751f\u540e5s\u7684\u6570\u636e\u3002\u540c\u65f6\u770b\u5230\u5728\u4e8b\u4ef6\u4e2d\u95f4\u65f6\u523b\uff0c\u8bb0\u5f55\u4e86Trigger\u4e8b\u4ef6\u53d1\u751f\u7684\u539f\u56e0\uff1a\u5373\u5728\u573a\u666f\u4e2d\u4e22\u5165\u4e86\u4e00\u4e2a\u5783\u573e,\u4f7f\u5f97\u573a\u666f\u4e2d\u5783\u573e\u8fbe\u5230\u4e09\u4e2a\uff0c\u89e6\u53d1Trigger\u3002"}),"\n",(0,t.jsx)(r.h3,{id:"\u62d3\u5c55\u529f\u80fd",children:"\u62d3\u5c55\u529f\u80fd"}),"\n",(0,t.jsx)(r.h4,{id:"\u7ed9trigger\u6a21\u5757\u4e0b\u53d1\u4efb\u52a1",children:"\u7ed9Trigger\u6a21\u5757\u4e0b\u53d1\u4efb\u52a1"}),"\n",(0,t.jsx)(r.p,{children:"Trigger\u6a21\u5757\u652f\u6301\u7531\u5176\u4ed6\u8282\u70b9\u4e0b\u53d1Trigger\u4efb\u52a1,\u63a7\u5236Trigger\u914d\u7f6e\u3002\u4e0b\u53d1\u65b9\u5f0f,\u901a\u8fc7\u53d1\u5e03std_msg\u7684\u8bdd\u9898\u6d88\u606f,\u6d88\u606f\u6570\u636e\u4e3ajson\u683c\u5f0f\u7684String\u6570\u636e\u3002\u5c06\u4efb\u52a1\u534f\u8bae\u53d1\u9001\u5230Trigger\u6a21\u5757\u3002"}),"\n",(0,t.jsx)(r.h5,{id:"trigger\u4efb\u52a1\u534f\u8bae",children:"Trigger\u4efb\u52a1\u534f\u8bae"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-json",children:'{\n   "version": "v0.0.1_20230421",       // Trigger\u6a21\u5757\u7248\u672c\u4fe1\u606f\u3002\n   "trigger_status": true,             // Trigger\u72b6\u6001, \'false\': \u5173\u95ed, \'true\': \u6253\u5f00\u3002\n   "strategy": [\n      {\n            "src_module_id": 203,      // \u53d1\u751fTrigger\u7684\u6a21\u5757ID\n            "trigger_type": 1110,      // Trigger\u7c7b\u578bID\u3002\n            "level": 1,                // Trigger\u4e8b\u4ef6\u7684\u4f18\u5148\u7ea7\n            "desc": "",                // Trigger\u6a21\u5757\u63cf\u8ff0\u4fe1\u606f\u3002\n            "duration_ts_back": 5000,  // \u5f55\u5236Trigger\u53d1\u751f\u540e\u6301\u7eed\u65f6\u957f\n            "duration_ts_front": 3000  // \u5f55\u5236Tirgger \u53d1\u751f\u524d\u6301\u7eed\u65f6\u957f\n      }\n   ]\n}\n'})}),"\n",(0,t.jsx)(r.h5,{id:"\u8fd0\u884c",children:"\u8fd0\u884c"}),"\n",(0,t.jsx)(r.p,{children:'\u5728\u524d\u9762\u542f\u52a8Trigger\u8282\u70b9\u57fa\u7840\u4e0a,\u5728\u53e6\u4e00\u4e2a\u7ec8\u7aef,\u53d1\u5e03\u8bdd\u9898\u540d\u4e3a"/hobot_agent"\u7684std_msg\u8bdd\u9898\u6d88\u606f\u3002'}),"\n",(0,t.jsxs)(s.A,{groupId:"tros-distro",children:[(0,t.jsx)(l.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(l.A,{value:"humble",label:"Humble",children:(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:'# \u53d1\u5e03\u8bdd\u9898\u540d\u4e3a"/hobot_agent"\u7684std_msg\u8bdd\u9898\u6d88\u606f\nros2 topic pub /hobot_agent std_msgs/String "data: \'{\\"version\\":\\"v0.0.1_20230421\\",\\"trigger_status\\":true,\\"strategy\\":[{\\"src_module_id\\":203,\\"trigger_type\\":1110,\\"status\\":true,\\"level\\":1,\\"desc\\":\\"test\\",\\"duration_ts_back\\":5000,\\"duration_ts_front\\":3000}]}\'"\n'})}),"\n",(0,t.jsx)(r.h5,{id:"\u65e5\u5fd7\u4fe1\u606f",children:"\u65e5\u5fd7\u4fe1\u606f"}),"\n",(0,t.jsx)(r.pre,{children:(0,t.jsx)(r.code,{className:"language-shell",children:'   [WARN] [1691670626.026737642] [hobot_trigger]: TriggerNode Init Succeed!\n   [WARN] [1691670626.026859316] [example]: TriggerExampleNode Init.\n   [INFO] [1691670626.517232775] [TriggerNode]: Updated Trigger Config: {"domain":"robot","desc":"trigger lane","duration_ts_back":5000,"duration_ts_front":3000,"gps_pos":{"latitude":-1,"longitude":-1},"level":1,"rosbag_path":"","src_module_id":203,"strategy_version":"Robot_sweeper_V1.0_20230526","timestamp":0,"topic":["/image_raw/compressed","/ai_msg_mono2d_trash_detection","/hobot_visualization"],"trigger_type":1110,"unique_id":"OriginBot002","version":"v0.0.1_20230421","extra_kv":[]}\n'})}),"\n",(0,t.jsx)(r.p,{children:"\u5206\u6790: \u5bf9Trigger\u6a21\u5757\u4e0b\u53d1\u914d\u7f6e\u4efb\u52a1\u7684\u65f6\u5019,\u53ef\u4ee5\u6210\u529f\u66f4\u65b0Trigger\u8282\u70b9\u7684\u914d\u7f6e\u3002\uff08Trigger\u8282\u70b9Log\u65e5\u5fd7\u4e3aINFO\u65f6\u53ef\u770b\u5230\u65e5\u5fd7\u66f4\u65b0\uff09"})]})}function u(e={}){const{wrapper:r}={...(0,i.R)(),...e.components};return r?(0,t.jsx)(r,{...e,children:(0,t.jsx)(h,{...e})}):h(e)}},19365:(e,r,n)=>{n.d(r,{A:()=>l});n(96540);var t=n(34164);const i={tabItem:"tabItem_Ymn6"};var s=n(74848);function l(e){let{children:r,hidden:n,className:l}=e;return(0,s.jsx)("div",{role:"tabpanel",className:(0,t.A)(i.tabItem,l),hidden:n,children:r})}},93859:(e,r,n)=>{n.d(r,{A:()=>T});var t=n(96540),i=n(34164),s=n(86641),l=n(56347),o=n(205),a=n(38874),d=n(24035),c=n(82993);function g(e){return t.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,t.isValidElement)(e)&&function(e){const{props:r}=e;return!!r&&"object"==typeof r&&"value"in r}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function h(e){const{values:r,children:n}=e;return(0,t.useMemo)((()=>{const e=r??function(e){return g(e).map((e=>{let{props:{value:r,label:n,attributes:t,default:i}}=e;return{value:r,label:n,attributes:t,default:i}}))}(n);return function(e){const r=(0,d.X)(e,((e,r)=>e.value===r.value));if(r.length>0)throw new Error(`Docusaurus error: Duplicate values "${r.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[r,n])}function u(e){let{value:r,tabValues:n}=e;return n.some((e=>e.value===r))}function p(e){let{queryString:r=!1,groupId:n}=e;const i=(0,l.W6)(),s=function(e){let{queryString:r=!1,groupId:n}=e;if("string"==typeof r)return r;if(!1===r)return null;if(!0===r&&!n)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return n??null}({queryString:r,groupId:n});return[(0,a.aZ)(s),(0,t.useCallback)((e=>{if(!s)return;const r=new URLSearchParams(i.location.search);r.set(s,e),i.replace({...i.location,search:r.toString()})}),[s,i])]}function b(e){const{defaultValue:r,queryString:n=!1,groupId:i}=e,s=h(e),[l,a]=(0,t.useState)((()=>function(e){let{defaultValue:r,tabValues:n}=e;if(0===n.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(r){if(!u({value:r,tabValues:n}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${r}" but none of its children has the corresponding value. Available values are: ${n.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return r}const t=n.find((e=>e.default))??n[0];if(!t)throw new Error("Unexpected error: 0 tabValues");return t.value}({defaultValue:r,tabValues:s}))),[d,g]=p({queryString:n,groupId:i}),[b,_]=function(e){let{groupId:r}=e;const n=function(e){return e?`docusaurus.tab.${e}`:null}(r),[i,s]=(0,c.Dv)(n);return[i,(0,t.useCallback)((e=>{n&&s.set(e)}),[n,s])]}({groupId:i}),x=(()=>{const e=d??b;return u({value:e,tabValues:s})?e:null})();(0,o.A)((()=>{x&&a(x)}),[x]);return{selectedValue:l,selectValue:(0,t.useCallback)((e=>{if(!u({value:e,tabValues:s}))throw new Error(`Can't select invalid tab value=${e}`);a(e),g(e),_(e)}),[g,_,s]),tabValues:s}}var _=n(92303);const x={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var m=n(74848);function j(e){let{className:r,block:n,selectedValue:t,selectValue:l,tabValues:o}=e;const a=[],{blockElementScrollPositionUntilNextRender:d}=(0,s.a_)(),c=e=>{const r=e.currentTarget,n=a.indexOf(r),i=o[n].value;i!==t&&(d(r),l(i))},g=e=>{let r=null;switch(e.key){case"Enter":c(e);break;case"ArrowRight":{const n=a.indexOf(e.currentTarget)+1;r=a[n]??a[0];break}case"ArrowLeft":{const n=a.indexOf(e.currentTarget)-1;r=a[n]??a[a.length-1];break}}r?.focus()};return(0,m.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,i.A)("tabs",{"tabs--block":n},r),children:o.map((e=>{let{value:r,label:n,attributes:s}=e;return(0,m.jsx)("li",{role:"tab",tabIndex:t===r?0:-1,"aria-selected":t===r,ref:e=>a.push(e),onKeyDown:g,onClick:c,...s,className:(0,i.A)("tabs__item",x.tabItem,s?.className,{"tabs__item--active":t===r}),children:n??r},r)}))})}function v(e){let{lazy:r,children:n,selectedValue:i}=e;const s=(Array.isArray(n)?n:[n]).filter(Boolean);if(r){const e=s.find((e=>e.props.value===i));return e?(0,t.cloneElement)(e,{className:"margin-top--md"}):null}return(0,m.jsx)("div",{className:"margin-top--md",children:s.map(((e,r)=>(0,t.cloneElement)(e,{key:r,hidden:e.props.value!==i})))})}function f(e){const r=b(e);return(0,m.jsxs)("div",{className:(0,i.A)("tabs-container",x.tabList),children:[(0,m.jsx)(j,{...r,...e}),(0,m.jsx)(v,{...r,...e})]})}function T(e){const r=(0,_.A)();return(0,m.jsx)(f,{...e,children:g(e.children)},String(r))}},49528:(e,r,n)=>{n.d(r,{A:()=>t});const t=n.p+"assets/images/mp4show-06ab0a5df586133012e0272b7c137b95.jpg"},8519:(e,r,n)=>{n.d(r,{A:()=>t});const t=n.p+"assets/images/show-7bef9ef64c1ec1b4df6bd6161d9a0507.png"},34543:(e,r,n)=>{n.d(r,{A:()=>t});const t=n.p+"assets/images/trigger_example_trash_det-0074a2df9db738abb373e1b02798ce72.gif"},28453:(e,r,n)=>{n.d(r,{R:()=>l,x:()=>o});var t=n(96540);const i={},s=t.createContext(i);function l(e){const r=t.useContext(s);return t.useMemo((function(){return"function"==typeof e?e(r):{...r,...e}}),[r,e])}function o(e){let r;return r=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:l(e.components),t.createElement(s.Provider,{value:r},e.children)}}}]);