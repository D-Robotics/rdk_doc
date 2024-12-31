"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[1083],{60814:(e,n,o)=>{o.r(n),o.d(n,{assets:()=>a,contentTitle:()=>l,default:()=>b,frontMatter:()=>i,metadata:()=>c,toc:()=>u});var t=o(74848),d=o(28453),r=o(93859),s=o(19365);const i={sidebar_position:1},l="\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a",c={id:"Robot_development/boxs/function/mono2d_body_detection",title:"\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a",description:"\u529f\u80fd\u4ecb\u7ecd",source:"@site/docs/05_Robot_development/03_boxs/function/mono2d_body_detection.md",sourceDirName:"05_Robot_development/03_boxs/function",slug:"/Robot_development/boxs/function/mono2d_body_detection",permalink:"/rdk_doc/Robot_development/boxs/function/mono2d_body_detection",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/05_Robot_development/03_boxs/function/mono2d_body_detection.md",tags:[],version:"current",sidebarPosition:1,frontMatter:{sidebar_position:1},sidebar:"tutorialSidebar",previous:{title:"YOLOv8-Seg",permalink:"/rdk_doc/Robot_development/boxs/segmentation/yolov8_seg"},next:{title:"\u4eba\u624b\u5173\u952e\u70b9\u68c0\u6d4b",permalink:"/rdk_doc/Robot_development/boxs/function/hand_lmk_detection"}},a={},u=[{value:"\u529f\u80fd\u4ecb\u7ecd",id:"\u529f\u80fd\u4ecb\u7ecd",level:2},{value:"\u652f\u6301\u5e73\u53f0",id:"\u652f\u6301\u5e73\u53f0",level:2},{value:"\u51c6\u5907\u5de5\u4f5c",id:"\u51c6\u5907\u5de5\u4f5c",level:2},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0",level:3},{value:"X86\u5e73\u53f0",id:"x86\u5e73\u53f0",level:3},{value:"\u4f7f\u7528\u4ecb\u7ecd",id:"\u4f7f\u7528\u4ecb\u7ecd",level:2},{value:"RDK\u5e73\u53f0",id:"rdk\u5e73\u53f0-1",level:3},{value:"X86\u5e73\u53f0",id:"x86\u5e73\u53f0-1",level:3},{value:"\u7ed3\u679c\u5206\u6790",id:"\u7ed3\u679c\u5206\u6790",level:2}];function h(e){const n={a:"a",br:"br",code:"code",h1:"h1",h2:"h2",h3:"h3",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",...(0,d.R)(),...e.components};return(0,t.jsxs)(t.Fragment,{children:[(0,t.jsx)(n.h1,{id:"\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a",children:"\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a"}),"\n","\n",(0,t.jsx)(n.h2,{id:"\u529f\u80fd\u4ecb\u7ecd",children:"\u529f\u80fd\u4ecb\u7ecd"}),"\n",(0,t.jsx)(n.p,{children:"\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a\u7b97\u6cd5\u793a\u4f8b\u8ba2\u9605\u56fe\u7247\uff0c\u5229\u7528BPU\u8fdb\u884c\u7b97\u6cd5\u63a8\u7406\uff0c\u53d1\u5e03\u5305\u542b\u4eba\u4f53\u3001\u4eba\u5934\u3001\u4eba\u8138\u3001\u4eba\u624b\u6846\u548c\u4eba\u4f53\u5173\u952e\u70b9\u68c0\u6d4b\u7ed3\u679cmsg\uff0c\u5e76\u901a\u8fc7\u591a\u76ee\u6807\u8ddf\u8e2a\uff08multi-target tracking\uff0c\u5373MOT\uff09\u529f\u80fd\uff0c\u5b9e\u73b0\u68c0\u6d4b\u6846\u7684\u8ddf\u8e2a\u3002X86\u7248\u672c\u6682\u4e0d\u652f\u6301\u591a\u76ee\u6807\u8ddf\u8e2a\u4ee5\u53caWeb\u7aef\u5c55\u793a\u529f\u80fd\u3002"}),"\n",(0,t.jsx)(n.p,{children:"\u7b97\u6cd5\u652f\u6301\u7684\u68c0\u6d4b\u7c7b\u522b\uff0c\u4ee5\u53ca\u4e0d\u540c\u7c7b\u522b\u5728\u7b97\u6cd5msg\u4e2d\u5bf9\u5e94\u7684\u6570\u636e\u7c7b\u578b\u5982\u4e0b\uff1a"}),"\n",(0,t.jsxs)(n.table,{children:[(0,t.jsx)(n.thead,{children:(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.th,{children:"\u7c7b\u522b"}),(0,t.jsx)(n.th,{children:"\u8bf4\u660e"}),(0,t.jsx)(n.th,{children:"\u6570\u636e\u7c7b\u578b"})]})}),(0,t.jsxs)(n.tbody,{children:[(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"body"}),(0,t.jsx)(n.td,{children:"\u4eba\u4f53\u6846"}),(0,t.jsx)(n.td,{children:"Roi"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"head"}),(0,t.jsx)(n.td,{children:"\u4eba\u5934\u6846"}),(0,t.jsx)(n.td,{children:"Roi"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"face"}),(0,t.jsx)(n.td,{children:"\u4eba\u8138\u6846"}),(0,t.jsx)(n.td,{children:"Roi"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"hand"}),(0,t.jsx)(n.td,{children:"\u4eba\u624b\u6846"}),(0,t.jsx)(n.td,{children:"Roi"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"body_kps"}),(0,t.jsx)(n.td,{children:"\u4eba\u4f53\u5173\u952e\u70b9"}),(0,t.jsx)(n.td,{children:"Point"})]})]})]}),"\n",(0,t.jsx)(n.p,{children:"\u4eba\u4f53\u5173\u952e\u70b9\u7b97\u6cd5\u7ed3\u679c\u7d22\u5f15\u5982\u4e0b\u56fe\uff1a"}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.img,{src:o(52310).A+"",width:"749",height:"1124"})}),"\n",(0,t.jsxs)(n.p,{children:["\u4ee3\u7801\u4ed3\u5e93\uff1a (",(0,t.jsx)(n.a,{href:"https://github.com/D-Robotics/mono2d_body_detection",children:"https://github.com/D-Robotics/mono2d_body_detection"}),")"]}),"\n",(0,t.jsx)(n.p,{children:"\u5e94\u7528\u573a\u666f\uff1a\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a\u7b97\u6cd5\u662f\u4eba\u4f53\u8fd0\u52a8\u89c6\u89c9\u5206\u6790\u7684\u91cd\u8981\u7ec4\u6210\u90e8\u5206\uff0c\u53ef\u5b9e\u73b0\u4eba\u4f53\u59ff\u6001\u5206\u6790\u4ee5\u53ca\u4eba\u6d41\u91cf\u7edf\u8ba1\u7b49\u529f\u80fd\uff0c\u4e3b\u8981\u5e94\u7528\u4e8e\u4eba\u673a\u4ea4\u4e92\u3001\u6e38\u620f\u5a31\u4e50\u7b49\u9886\u57df\u3002"}),"\n",(0,t.jsxs)(n.p,{children:["\u59ff\u6001\u68c0\u6d4b\u6848\u4f8b\uff1a",(0,t.jsx)(n.a,{href:"../../apps/fall_detection",children:"4.3. \u59ff\u6001\u68c0\u6d4b"}),(0,t.jsx)(n.br,{}),"\n","\u5c0f\u8f66\u4eba\u4f53\u8ddf\u968f\u6848\u4f8b\uff1a",(0,t.jsx)(n.a,{href:"../../apps/car_tracking",children:"4.4. \u5c0f\u8f66\u4eba\u4f53\u8ddf\u968f"}),(0,t.jsx)(n.br,{}),"\n","\u57fa\u4e8e\u4eba\u4f53\u59ff\u6001\u5206\u6790\u4ee5\u53ca\u624b\u52bf\u8bc6\u522b\u5b9e\u73b0\u6e38\u620f\u4eba\u7269\u63a7\u5236\u6848\u4f8b\uff1a",(0,t.jsx)(n.a,{href:"https://developer.d-robotics.cc/forumDetail/112555512834430487",children:"\u73a9\u8f6cX3\u6d3e\uff0c\u5065\u8eab\u6e38\u620f\u4e24\u4e0d\u8bef"})]}),"\n",(0,t.jsx)(n.h2,{id:"\u652f\u6301\u5e73\u53f0",children:"\u652f\u6301\u5e73\u53f0"}),"\n",(0,t.jsxs)(n.table,{children:[(0,t.jsx)(n.thead,{children:(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.th,{children:"\u5e73\u53f0"}),(0,t.jsx)(n.th,{children:"\u8fd0\u884c\u65b9\u5f0f"}),(0,t.jsx)(n.th,{children:"\u793a\u4f8b\u529f\u80fd"})]})}),(0,t.jsxs)(n.tbody,{children:[(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"RDK X3, RDK X3 Module"}),(0,t.jsx)(n.td,{children:"Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)"}),(0,t.jsx)(n.td,{children:"\u542f\u52a8MIPI/USB\u6444\u50cf\u5934\uff0c\u5e76\u901a\u8fc7Web\u5c55\u793a\u63a8\u7406\u6e32\u67d3\u7ed3\u679c"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"RDK X5"}),(0,t.jsx)(n.td,{children:"Ubuntu 22.04 (Humble)"}),(0,t.jsx)(n.td,{children:"\u542f\u52a8MIPI/USB\u6444\u50cf\u5934\uff0c\u5e76\u901a\u8fc7Web\u5c55\u793a\u63a8\u7406\u6e32\u67d3\u7ed3\u679c"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"RDK Ultra"}),(0,t.jsx)(n.td,{children:"Ubuntu 20.04 (Foxy)"}),(0,t.jsx)(n.td,{children:"\u542f\u52a8MIPI/USB\u6444\u50cf\u5934/\u672c\u5730\u56de\u704c\uff0c\u5e76\u901a\u8fc7Web\u5c55\u793a\u63a8\u7406\u6e32\u67d3\u7ed3\u679c"})]}),(0,t.jsxs)(n.tr,{children:[(0,t.jsx)(n.td,{children:"X86"}),(0,t.jsx)(n.td,{children:"Ubuntu 20.04 (Foxy)"}),(0,t.jsx)(n.td,{children:"\u542f\u52a8\u672c\u5730\u56de\u704c\uff0c\u5e76\u901a\u8fc7Web\u5c55\u793a\u63a8\u7406\u6e32\u67d3\u7ed3\u679c"})]})]})]}),"\n",(0,t.jsx)(n.h2,{id:"\u51c6\u5907\u5de5\u4f5c",children:"\u51c6\u5907\u5de5\u4f5c"}),"\n",(0,t.jsx)(n.h3,{id:"rdk\u5e73\u53f0",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsxs)(n.ol,{children:["\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"RDK\u5df2\u70e7\u5f55\u597dUbuntu 20.04/Ubuntu 22.04\u7cfb\u7edf\u955c\u50cf\u3002"}),"\n"]}),"\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"RDK\u5df2\u6210\u529f\u5b89\u88c5TogetheROS.Bot\u3002"}),"\n"]}),"\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"RDK\u5df2\u5b89\u88c5MIPI\u6216\u8005USB\u6444\u50cf\u5934\u3002"}),"\n"]}),"\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"\u786e\u8ba4PC\u673a\u80fd\u591f\u901a\u8fc7\u7f51\u7edc\u8bbf\u95eeRDK\u3002"}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(n.h3,{id:"x86\u5e73\u53f0",children:"X86\u5e73\u53f0"}),"\n",(0,t.jsxs)(n.ol,{children:["\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"X86\u73af\u5883\u5df2\u914d\u7f6eUbuntu 20.04\u7cfb\u7edf\u955c\u50cf\u3002"}),"\n"]}),"\n",(0,t.jsxs)(n.li,{children:["\n",(0,t.jsx)(n.p,{children:"X86\u73af\u5883\u5df2\u6210\u529f\u5b89\u88c5tros.b\u3002"}),"\n"]}),"\n"]}),"\n",(0,t.jsx)(n.h2,{id:"\u4f7f\u7528\u4ecb\u7ecd",children:"\u4f7f\u7528\u4ecb\u7ecd"}),"\n",(0,t.jsx)(n.p,{children:"\u4eba\u4f53\u68c0\u6d4b\u548c\u8ddf\u8e2a(mono2d_body_detection)package\u8ba2\u9605sensor package\u53d1\u5e03\u7684\u56fe\u7247\uff0c\u7ecf\u8fc7\u63a8\u7406\u540e\u53d1\u5e03\u7b97\u6cd5msg\uff0c\u901a\u8fc7websocket package\u5b9e\u73b0\u5728PC\u7aef\u6d4f\u89c8\u5668\u4e0a\u6e32\u67d3\u663e\u793asensor\u53d1\u5e03\u7684\u56fe\u7247\u548c\u5bf9\u5e94\u7684\u7b97\u6cd5\u7ed3\u679c\u3002"}),"\n",(0,t.jsx)(n.h3,{id:"rdk\u5e73\u53f0-1",children:"RDK\u5e73\u53f0"}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.strong,{children:"\u4f7f\u7528MIPI\u6444\u50cf\u5934\u53d1\u5e03\u56fe\u7247"})}),"\n",(0,t.jsxs)(r.A,{groupId:"tros-distro",children:[(0,t.jsx)(s.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(s.A,{value:"humble",label:"Humble",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .\n\n# \u914d\u7f6eMIPI\u6444\u50cf\u5934\nexport CAM_TYPE=mipi\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch mono2d_body_detection mono2d_body_detection.launch.py\n"})}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.strong,{children:"\u4f7f\u7528USB\u6444\u50cf\u5934\u53d1\u5e03\u56fe\u7247"})}),"\n",(0,t.jsxs)(r.A,{groupId:"tros-distro",children:[(0,t.jsx)(s.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(s.A,{value:"humble",label:"Humble",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"\n# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .\n\n# \u914d\u7f6eUSB\u6444\u50cf\u5934\nexport CAM_TYPE=usb\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch mono2d_body_detection mono2d_body_detection.launch.py\n"})}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.strong,{children:"\u4f7f\u7528\u672c\u5730\u56de\u704c\u56fe\u7247"})}),"\n",(0,t.jsxs)(r.A,{groupId:"tros-distro",children:[(0,t.jsx)(s.A,{value:"foxy",label:"Foxy",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n"})})}),(0,t.jsx)(s.A,{value:"humble",label:"Humble",children:(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .\ncp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .\n\n# \u914d\u7f6e\u672c\u5730\u56de\u704c\u56fe\u7247\nexport CAM_TYPE=fb\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch mono2d_body_detection mono2d_body_detection.launch.py publish_image_source:=config/person_body.jpg publish_image_format:=jpg publish_output_image_w:=960 publish_output_image_h:=544\n\n# RDK Ultra\u5e73\u53f0\u9700\u8981\u6307\u5b9a\u56de\u704c\u56fe\u7247\uff0c\u4f8b\u5982\uff1a\n# ros2 launch mono2d_body_detection mono2d_body_detection.launch.py picture:=./config/target.jpg\n"})}),"\n",(0,t.jsx)(n.h3,{id:"x86\u5e73\u53f0-1",children:"X86\u5e73\u53f0"}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.strong,{children:"\u4f7f\u7528\u672c\u5730\u56de\u704c\u56fe\u7247"})}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-bash",children:"# \u914d\u7f6etros.b\u73af\u5883\nsource /opt/tros/setup.bash\n\n# \u4ecetros.b\u7684\u5b89\u88c5\u8def\u5f84\u4e2d\u62f7\u8d1d\u51fa\u8fd0\u884c\u793a\u4f8b\u9700\u8981\u7684\u914d\u7f6e\u6587\u4ef6\u3002\ncp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .\ncp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .\n\n# \u914d\u7f6e\u672c\u5730\u56de\u704c\u56fe\u7247\nexport CAM_TYPE=fb\n\n# \u542f\u52a8launch\u6587\u4ef6\nros2 launch mono2d_body_detection mono2d_body_detection.launch.py\n"})}),"\n",(0,t.jsx)(n.h2,{id:"\u7ed3\u679c\u5206\u6790",children:"\u7ed3\u679c\u5206\u6790"}),"\n",(0,t.jsx)(n.p,{children:"\u5728\u8fd0\u884c\u7ec8\u7aef\u8f93\u51fa\u5982\u4e0b\u4fe1\u606f\uff1a"}),"\n",(0,t.jsx)(n.pre,{children:(0,t.jsx)(n.code,{className:"language-shell",children:"[mono2d_body_detection-3] [WARN] [1660219823.214730286] [example]: This is mono2d body det example!\n[mono2d_body_detection-3] [WARN] [1660219823.417856952] [mono2d_body_det]: Parameter:\n[mono2d_body_detection-3]  is_sync_mode_: 0\n[mono2d_body_detection-3]  model_file_name_: config/multitask_body_head_face_hand_kps_960x544.hbm\n[mono2d_body_detection-3]  is_shared_mem_sub: 1\n[mono2d_body_detection-3]  ai_msg_pub_topic_name: /hobot_mono2d_body_detection\n[mono2d_body_detection-3] [C][31082][08-11][20:10:23:425][configuration.cpp:49][EasyDNN]EasyDNN version: 0.4.11\n[mono2d_body_detection-3] [BPU_PLAT]BPU Platform Version(1.3.1)!\n[mono2d_body_detection-3] [HBRT] set log level as 0. version = 3.14.5\n[mono2d_body_detection-3] [DNN] Runtime version = 1.9.7_(3.14.5 HBRT)\n[mono2d_body_detection-3] [WARN] [1660219823.545293244] [mono2d_body_det]: Create hbmem_subscription with topic_name: /hbmem_img\n[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_euclid_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_euclid_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (MOTMethod.cpp:39): MOTMethod::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] (IOU2.cpp:34): IOU2 Mot::Init config/iou2_method_param.json\n[mono2d_body_detection-3] \n[mono2d_body_detection-3] [WARN] [1660219824.895102286] [mono2d_body_det]: input fps: 31.34, out fps: 31.22\n[mono2d_body_detection-3] [WARN] [1660219825.921873870] [mono2d_body_det]: input fps: 30.16, out fps: 30.21\n[mono2d_body_detection-3] [WARN] [1660219826.922075496] [mono2d_body_det]: input fps: 30.16, out fps: 30.00\n[mono2d_body_detection-3] [WARN] [1660219827.955463330] [mono2d_body_det]: input fps: 30.01, out fps: 30.01\n[mono2d_body_detection-3] [WARN] [1660219828.955764872] [mono2d_body_det]: input fps: 30.01, out fps: 30.00\n"})}),"\n",(0,t.jsx)(n.p,{children:"\u8f93\u51falog\u663e\u793a\uff0c\u7a0b\u5e8f\u8fd0\u884c\u6210\u529f\uff0c\u63a8\u7406\u65f6\u7b97\u6cd5\u8f93\u5165\u548c\u8f93\u51fa\u5e27\u7387\u4e3a30fps\uff0c\u6bcf\u79d2\u949f\u5237\u65b0\u4e00\u6b21\u7edf\u8ba1\u5e27\u7387\u3002"}),"\n",(0,t.jsxs)(n.p,{children:["\u5728PC\u7aef\u7684\u6d4f\u89c8\u5668\u8f93\u5165",(0,t.jsx)(n.a,{href:"http://IP:8000",children:"http://IP:8000"})," \u5373\u53ef\u67e5\u770b\u56fe\u50cf\u548c\u7b97\u6cd5\uff08\u4eba\u4f53\u3001\u4eba\u5934\u3001\u4eba\u8138\u3001\u4eba\u624b\u68c0\u6d4b\u6846\uff0c\u68c0\u6d4b\u6846\u7c7b\u578b\u548c\u76ee\u6807\u8ddf\u8e2aID\uff0c\u4eba\u4f53\u5173\u952e\u70b9\uff09\u6e32\u67d3\u6548\u679c\uff08IP\u4e3aRDK/X86\u8bbe\u5907\u7684IP\u5730\u5740\uff09\uff1a"]}),"\n",(0,t.jsx)(n.p,{children:(0,t.jsx)(n.img,{src:o(92688).A+"",width:"1920",height:"908"})})]})}function b(e={}){const{wrapper:n}={...(0,d.R)(),...e.components};return n?(0,t.jsx)(n,{...e,children:(0,t.jsx)(h,{...e})}):h(e)}},19365:(e,n,o)=>{o.d(n,{A:()=>s});o(96540);var t=o(34164);const d={tabItem:"tabItem_Ymn6"};var r=o(74848);function s(e){let{children:n,hidden:o,className:s}=e;return(0,r.jsx)("div",{role:"tabpanel",className:(0,t.A)(d.tabItem,s),hidden:o,children:n})}},93859:(e,n,o)=>{o.d(n,{A:()=>v});var t=o(96540),d=o(34164),r=o(86641),s=o(56347),i=o(205),l=o(38874),c=o(24035),a=o(82993);function u(e){return t.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,t.isValidElement)(e)&&function(e){const{props:n}=e;return!!n&&"object"==typeof n&&"value"in n}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function h(e){const{values:n,children:o}=e;return(0,t.useMemo)((()=>{const e=n??function(e){return u(e).map((e=>{let{props:{value:n,label:o,attributes:t,default:d}}=e;return{value:n,label:o,attributes:t,default:d}}))}(o);return function(e){const n=(0,c.X)(e,((e,n)=>e.value===n.value));if(n.length>0)throw new Error(`Docusaurus error: Duplicate values "${n.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[n,o])}function b(e){let{value:n,tabValues:o}=e;return o.some((e=>e.value===n))}function p(e){let{queryString:n=!1,groupId:o}=e;const d=(0,s.W6)(),r=function(e){let{queryString:n=!1,groupId:o}=e;if("string"==typeof n)return n;if(!1===n)return null;if(!0===n&&!o)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return o??null}({queryString:n,groupId:o});return[(0,l.aZ)(r),(0,t.useCallback)((e=>{if(!r)return;const n=new URLSearchParams(d.location.search);n.set(r,e),d.replace({...d.location,search:n.toString()})}),[r,d])]}function _(e){const{defaultValue:n,queryString:o=!1,groupId:d}=e,r=h(e),[s,l]=(0,t.useState)((()=>function(e){let{defaultValue:n,tabValues:o}=e;if(0===o.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(n){if(!b({value:n,tabValues:o}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${n}" but none of its children has the corresponding value. Available values are: ${o.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return n}const t=o.find((e=>e.default))??o[0];if(!t)throw new Error("Unexpected error: 0 tabValues");return t.value}({defaultValue:n,tabValues:r}))),[c,u]=p({queryString:o,groupId:d}),[_,m]=function(e){let{groupId:n}=e;const o=function(e){return e?`docusaurus.tab.${e}`:null}(n),[d,r]=(0,a.Dv)(o);return[d,(0,t.useCallback)((e=>{o&&r.set(e)}),[o,r])]}({groupId:d}),x=(()=>{const e=c??_;return b({value:e,tabValues:r})?e:null})();(0,i.A)((()=>{x&&l(x)}),[x]);return{selectedValue:s,selectValue:(0,t.useCallback)((e=>{if(!b({value:e,tabValues:r}))throw new Error(`Can't select invalid tab value=${e}`);l(e),u(e),m(e)}),[u,m,r]),tabValues:r}}var m=o(92303);const x={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var j=o(74848);function f(e){let{className:n,block:o,selectedValue:t,selectValue:s,tabValues:i}=e;const l=[],{blockElementScrollPositionUntilNextRender:c}=(0,r.a_)(),a=e=>{const n=e.currentTarget,o=l.indexOf(n),d=i[o].value;d!==t&&(c(n),s(d))},u=e=>{let n=null;switch(e.key){case"Enter":a(e);break;case"ArrowRight":{const o=l.indexOf(e.currentTarget)+1;n=l[o]??l[0];break}case"ArrowLeft":{const o=l.indexOf(e.currentTarget)-1;n=l[o]??l[l.length-1];break}}n?.focus()};return(0,j.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,d.A)("tabs",{"tabs--block":o},n),children:i.map((e=>{let{value:n,label:o,attributes:r}=e;return(0,j.jsx)("li",{role:"tab",tabIndex:t===n?0:-1,"aria-selected":t===n,ref:e=>l.push(e),onKeyDown:u,onClick:a,...r,className:(0,d.A)("tabs__item",x.tabItem,r?.className,{"tabs__item--active":t===n}),children:o??n},n)}))})}function g(e){let{lazy:n,children:o,selectedValue:d}=e;const r=(Array.isArray(o)?o:[o]).filter(Boolean);if(n){const e=r.find((e=>e.props.value===d));return e?(0,t.cloneElement)(e,{className:"margin-top--md"}):null}return(0,j.jsx)("div",{className:"margin-top--md",children:r.map(((e,n)=>(0,t.cloneElement)(e,{key:n,hidden:e.props.value!==d})))})}function y(e){const n=_(e);return(0,j.jsxs)("div",{className:(0,d.A)("tabs-container",x.tabList),children:[(0,j.jsx)(f,{...n,...e}),(0,j.jsx)(g,{...n,...e})]})}function v(e){const n=(0,m.A)();return(0,j.jsx)(y,{...e,children:u(e.children)},String(n))}},92688:(e,n,o)=>{o.d(n,{A:()=>t});const t=o.p+"assets/images/body_render-bfbd47112fb160cdb8aeeb62a8f9fd2a.jpeg"},52310:(e,n,o)=>{o.d(n,{A:()=>t});const t=o.p+"assets/images/kps_index-6cecb8977d692cc8db31148e7c9ada54.jpeg"},28453:(e,n,o)=>{o.d(n,{R:()=>s,x:()=>i});var t=o(96540);const d={},r=t.createContext(d);function s(e){const n=t.useContext(r);return t.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function i(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(d):e.components||d:s(e.components),t.createElement(r.Provider,{value:n},e.children)}}}]);