"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[5115],{52966:(e,i,d)=>{d.r(i),d.d(i,{assets:()=>r,contentTitle:()=>t,default:()=>h,frontMatter:()=>c,metadata:()=>l,toc:()=>o});var n=d(74848),s=d(28453);const c={sidebar_position:6},t="SYS\uff08\u6a21\u5757\u7ed1\u5b9a\uff09API",l={id:"Basic_Application/multi_media/multi_media_api/cdev_multimedia_api_ultra/sys_api",title:"SYS\uff08\u6a21\u5757\u7ed1\u5b9a\uff09API",description:"SYS API\u63d0\u4f9b\u4e86\u4ee5\u4e0b\u7684\u63a5\u53e3\uff1a",source:"@site/docs/03_Basic_Application/04_multi_media/multi_media_api/cdev_multimedia_api_ultra/sys_api.md",sourceDirName:"03_Basic_Application/04_multi_media/multi_media_api/cdev_multimedia_api_ultra",slug:"/Basic_Application/multi_media/multi_media_api/cdev_multimedia_api_ultra/sys_api",permalink:"/rdk_doc/en/Basic_Application/multi_media/multi_media_api/cdev_multimedia_api_ultra/sys_api",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/03_Basic_Application/04_multi_media/multi_media_api/cdev_multimedia_api_ultra/sys_api.md",tags:[],version:"current",sidebarPosition:6,frontMatter:{sidebar_position:6},sidebar:"tutorialSidebar",previous:{title:"BPU\uff08\u7b97\u6cd5\u63a8\u7406\u6a21\u5757\uff09API",permalink:"/rdk_doc/en/Basic_Application/multi_media/multi_media_api/cdev_multimedia_api_ultra/bpu_api"},next:{title:"4. \u7b97\u6cd5\u5e94\u7528\u5f00\u53d1",permalink:"/rdk_doc/en/Basic_Development"}},r={},o=[{value:"sp_module_bind",id:"sp_module_bind",level:3},{value:"sp_module_unbind",id:"sp_module_unbind",level:3}];function _(e){const i={code:"code",h1:"h1",h3:"h3",li:"li",p:"p",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",ul:"ul",...(0,s.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsx)(i.h1,{id:"sys\u6a21\u5757\u7ed1\u5b9aapi",children:"SYS\uff08\u6a21\u5757\u7ed1\u5b9a\uff09API"}),"\n",(0,n.jsxs)(i.p,{children:[(0,n.jsx)(i.code,{children:"SYS"})," API\u63d0\u4f9b\u4e86\u4ee5\u4e0b\u7684\u63a5\u53e3\uff1a"]}),"\n",(0,n.jsxs)(i.table,{children:[(0,n.jsx)(i.thead,{children:(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.th,{children:"\u51fd\u6570"}),(0,n.jsx)(i.th,{children:"\u529f\u80fd"})]})}),(0,n.jsxs)(i.tbody,{children:[(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"sp_module_bind"}),(0,n.jsx)(i.td,{children:(0,n.jsx)(i.strong,{children:"\u7ed1\u5b9a\u6570\u636e\u6e90\u3001\u76ee\u6807\u6a21\u5757"})})]}),(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"sp_module_unbind"}),(0,n.jsx)(i.td,{children:(0,n.jsx)(i.strong,{children:"\u89e3\u9664\u6a21\u5757\u95f4\u7684\u7ed1\u5b9a"})})]})]})]}),"\n",(0,n.jsx)(i.h3,{id:"sp_module_bind",children:"sp_module_bind"}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u51fd\u6570\u539f\u578b\u3011"})}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.code,{children:"int32_t sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)"})}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"})}),"\n",(0,n.jsxs)(i.p,{children:["\u672c\u63a5\u53e3\u53ef\u4ee5\u628a ",(0,n.jsx)(i.code,{children:"VIO"}),"\uff0c",(0,n.jsx)(i.code,{children:"ENCODER"}),"\uff0c",(0,n.jsx)(i.code,{children:"DECODER"}),"\uff0c",(0,n.jsx)(i.code,{children:"DISPLAY"}),", \u8fd9\u56db\u4e2a\u6a21\u5757\u7684\u8f93\u51fa\u4e0e\u8f93\u5165\u8fdb\u884c\u5185\u90e8\u7ed1\u5b9a\uff0c\u7ed1\u5b9a\u540e\u7684\u4e24\u4e2a\u6a21\u5757\u7684\u6570\u636e\u4f1a\u5728\u5185\u90e8\u81ea\u52a8\u6d41\u8f6c\uff0c\u65e0\u9700\u7528\u6237\u64cd\u4f5c\u3002\u6bd4\u5982\u7ed1\u5b9a ",(0,n.jsx)(i.code,{children:"VIO"})," \u548c ",(0,n.jsx)(i.code,{children:"DISPLAY"})," \u540e\uff0c\u6253\u5f00\u7684mipi\u6444\u50cf\u5934\u7684\u6570\u636e\u4f1a\u76f4\u63a5\u663e\u793a\u5230\u663e\u793a\u5c4f\u4e0a\uff0c\u4e0d\u9700\u8981\u8c03\u7528",(0,n.jsx)(i.code,{children:"VIO"}),"\u7684",(0,n.jsx)(i.code,{children:"sp_vio_get_frame"}),"\u63a5\u53e3\u83b7\u53d6\u6570\u636e\uff0c\u4e4b\u540e\u518d\u8c03\u7528",(0,n.jsx)(i.code,{children:"DISPLAY"}),"\u7684",(0,n.jsx)(i.code,{children:"sp_display_set_image"}),"\u63a5\u53e3\u8fdb\u884c\u663e\u793a\u3002"]}),"\n",(0,n.jsx)(i.p,{children:"\u652f\u6301\u7ed1\u5b9a\u7684\u6a21\u5757\u5173\u7cfb\u5982\u4e0b\uff1a"}),"\n",(0,n.jsxs)(i.table,{children:[(0,n.jsx)(i.thead,{children:(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.th,{children:"\u6e90\u6570\u636e\u6a21\u5757"}),(0,n.jsx)(i.th,{children:"\u76ee\u6807\u6570\u636e\u6a21\u5757"})]})}),(0,n.jsxs)(i.tbody,{children:[(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"VIO"}),(0,n.jsx)(i.td,{children:"ENCODER"})]}),(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"VIO"}),(0,n.jsx)(i.td,{children:"DISPLAY"})]}),(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"DECODER"}),(0,n.jsx)(i.td,{children:"ENCODER"})]}),(0,n.jsxs)(i.tr,{children:[(0,n.jsx)(i.td,{children:"DECODER"}),(0,n.jsx)(i.td,{children:"DISPLAY"})]})]})]}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u53c2\u6570\u3011"})}),"\n",(0,n.jsxs)(i.ul,{children:["\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"src"}),"\uff1a \u6570\u636e\u6e90\u6a21\u5757\u7684\u5bf9\u8c61\u6307\u9488\uff08\u8c03\u7528\u5404\u6a21\u5757\u521d\u59cb\u5316\u63a5\u53e3\u5f97\u5230\uff09"]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"src_type"}),"\uff1a\u6e90\u6570\u636e\u6a21\u5757\u7c7b\u578b\uff0c\u652f\u6301 ",(0,n.jsx)(i.code,{children:"SP_MTYPE_VIO"})," \u548c ",(0,n.jsx)(i.code,{children:"SP_MTYPE_DECODER"})]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"dst"}),"\uff1a \u76ee\u6807\u6a21\u5757\u7684\u5bf9\u8c61\u6307\u9488\uff08\u8c03\u7528\u5404\u6a21\u5757\u521d\u59cb\u5316\u63a5\u53e3\u5f97\u5230\uff09"]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"dst_type"}),"\uff1a\u76ee\u6807\u6570\u636e\u6a21\u5757\u7c7b\u578b\uff0c\u652f\u6301 ",(0,n.jsx)(i.code,{children:"SP_MTYPE_ENCODER"})," \u548c ",(0,n.jsx)(i.code,{children:"SP_MTYPE_DISPLAY"})]}),"\n"]}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u8fd4\u56de\u7c7b\u578b\u3011"})}),"\n",(0,n.jsx)(i.p,{children:"\u6210\u529f\u8fd4\u56de 0\uff0c\u5931\u8d25\u8fd4\u56de\u5176\u4ed6\u503c\u3002"}),"\n",(0,n.jsx)(i.h3,{id:"sp_module_unbind",children:"sp_module_unbind"}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u51fd\u6570\u539f\u578b\u3011"})}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.code,{children:"int32_t sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)"})}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"})}),"\n",(0,n.jsx)(i.p,{children:"\u672c\u63a5\u53e3\u5b8c\u6210\u5df2\u7ecf\u7ed1\u5b9a\u7684\u4e24\u4e2a\u6a21\u5757\u7684\u89e3\u7ed1\uff0c\u6a21\u5757\u9000\u51fa\u524d\u9700\u8981\u5148\u5b8c\u6210\u89e3\u7ed1\u3002"}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u53c2\u6570\u3011"})}),"\n",(0,n.jsxs)(i.ul,{children:["\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"src"}),"\uff1a \u6570\u636e\u6e90\u6a21\u5757\u7684\u5bf9\u8c61\u6307\u9488\uff08\u8c03\u7528\u5404\u6a21\u5757\u521d\u59cb\u5316\u63a5\u53e3\u5f97\u5230\uff09"]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"src_type"}),"\uff1a\u6e90\u6570\u636e\u6a21\u5757\u7c7b\u578b\uff0c\u652f\u6301 ",(0,n.jsx)(i.code,{children:"SP_MTYPE_VIO"})," \u548c ",(0,n.jsx)(i.code,{children:"SP_MTYPE_DECODER"})]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"dst"}),"\uff1a \u76ee\u6807\u6a21\u5757\u7684\u5bf9\u8c61\u6307\u9488\uff08\u8c03\u7528\u5404\u6a21\u5757\u521d\u59cb\u5316\u63a5\u53e3\u5f97\u5230\uff09"]}),"\n",(0,n.jsxs)(i.li,{children:[(0,n.jsx)(i.code,{children:"dst_type"}),"\uff1a\u76ee\u6807\u6570\u636e\u6a21\u5757\u7c7b\u578b\uff0c\u652f\u6301 ",(0,n.jsx)(i.code,{children:"SP_MTYPE_ENCODER"})," \u548c ",(0,n.jsx)(i.code,{children:"SP_MTYPE_DISPLAY"})]}),"\n"]}),"\n",(0,n.jsx)(i.p,{children:(0,n.jsx)(i.strong,{children:"\u3010\u8fd4\u56de\u7c7b\u578b\u3011"})}),"\n",(0,n.jsx)(i.p,{children:"\u6210\u529f\u8fd4\u56de 0\uff0c\u5931\u8d25\u8fd4\u56de\u5176\u4ed6\u503c\u3002"})]})}function h(e={}){const{wrapper:i}={...(0,s.R)(),...e.components};return i?(0,n.jsx)(i,{...e,children:(0,n.jsx)(_,{...e})}):_(e)}},28453:(e,i,d)=>{d.d(i,{R:()=>t,x:()=>l});var n=d(96540);const s={},c=n.createContext(s);function t(e){const i=n.useContext(c);return n.useMemo((function(){return"function"==typeof e?e(i):{...i,...e}}),[i,e])}function l(e){let i;return i=e.disableParentContext?"function"==typeof e.components?e.components(s):e.components||s:t(e.components),n.createElement(c.Provider,{value:i},e.children)}}}]);