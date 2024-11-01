"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[1406],{23689:(e,n,d)=>{d.r(n),d.d(n,{assets:()=>o,contentTitle:()=>r,default:()=>x,frontMatter:()=>l,metadata:()=>t,toc:()=>s});var i=d(74848),c=d(28453);const l={sidebar_position:2},r="Encoder\u5bf9\u8c61",t={id:"Basic_Application/multi_media/multi_media_api/pydev_multimedia_api_x3/object_encoder",title:"Encoder\u5bf9\u8c61",description:"Encoder\u5bf9\u8c61\u5b9e\u73b0\u4e86\u5bf9\u89c6\u9891\u6570\u636e\u7684\u7f16\u7801\u538b\u7f29\u529f\u80fd\uff0c\u5305\u542b\u4e86encode\u3001encodefile\u3001getimg\u3001close\u7b49\u51e0\u79cd\u65b9\u6cd5\uff0c\u8be6\u7ec6\u8bf4\u660e\u5982\u4e0b\uff1a",source:"@site/docs/03_Basic_Application/04_multi_media/multi_media_api/pydev_multimedia_api_x3/object_encoder.md",sourceDirName:"03_Basic_Application/04_multi_media/multi_media_api/pydev_multimedia_api_x3",slug:"/Basic_Application/multi_media/multi_media_api/pydev_multimedia_api_x3/object_encoder",permalink:"/rdk_doc/en/Basic_Application/multi_media/multi_media_api/pydev_multimedia_api_x3/object_encoder",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/03_Basic_Application/04_multi_media/multi_media_api/pydev_multimedia_api_x3/object_encoder.md",tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"tutorialSidebar",previous:{title:"Camera\u5bf9\u8c61",permalink:"/rdk_doc/en/Basic_Application/multi_media/multi_media_api/pydev_multimedia_api_x3/object_camera"},next:{title:"Decoder\u5bf9\u8c61",permalink:"/rdk_doc/en/Basic_Application/multi_media/multi_media_api/pydev_multimedia_api_x3/object_decoder"}},o={},s=[{value:"encode",id:"encode",level:2},{value:"encode_file",id:"encode_file",level:2},{value:"get_img",id:"get_img",level:2},{value:"close",id:"close",level:2}];function h(e){const n={code:"code",h1:"h1",h2:"h2",p:"p",pre:"pre",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",...(0,c.R)(),...e.components};return(0,i.jsxs)(i.Fragment,{children:[(0,i.jsx)(n.h1,{id:"encoder\u5bf9\u8c61",children:"Encoder\u5bf9\u8c61"}),"\n",(0,i.jsxs)(n.p,{children:["Encoder\u5bf9\u8c61\u5b9e\u73b0\u4e86\u5bf9\u89c6\u9891\u6570\u636e\u7684\u7f16\u7801\u538b\u7f29\u529f\u80fd\uff0c\u5305\u542b\u4e86",(0,i.jsx)(n.code,{children:"encode"}),"\u3001",(0,i.jsx)(n.code,{children:"encode_file"}),"\u3001",(0,i.jsx)(n.code,{children:"get_img"}),"\u3001",(0,i.jsx)(n.code,{children:"close"}),"\u7b49\u51e0\u79cd\u65b9\u6cd5\uff0c\u8be6\u7ec6\u8bf4\u660e\u5982\u4e0b\uff1a"]}),"\n",(0,i.jsx)(n.h2,{id:"encode",children:"encode"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u914d\u7f6e\u5e76\u4f7f\u80fdencode\u7f16\u7801\u6a21\u5757"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u51fd\u6570\u58f0\u660e\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"Encoder.encode(video_chn, encode_type , width, height, bits)\n"})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u6570\u63cf\u8ff0\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u53c2\u6570\u540d\u79f0"}),(0,i.jsx)(n.th,{children:"\u63cf\u8ff0"}),(0,i.jsx)(n.th,{children:"\u53d6\u503c\u8303\u56f4"})]})}),(0,i.jsxs)(n.tbody,{children:[(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"video_chn"}),(0,i.jsx)(n.td,{children:"\u6307\u5b9a\u89c6\u9891\u7f16\u7801\u5668\u7684\u901a\u9053\u53f7"}),(0,i.jsx)(n.td,{children:"\u8303\u56f40~31"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"encode_type"}),(0,i.jsx)(n.td,{children:"\u89c6\u9891\u7f16\u7801\u7c7b\u578b"}),(0,i.jsxs)(n.td,{children:["\u8303\u56f41~3\uff0c\u5206\u522b\u5bf9\u5e94",(0,i.jsx)(n.code,{children:"H264"}),"\u3001",(0,i.jsx)(n.code,{children:"H265"}),"\u3001",(0,i.jsx)(n.code,{children:"MJPEG"})]})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"width"}),(0,i.jsx)(n.td,{children:"\u8f93\u5165\u7f16\u7801\u6a21\u5757\u7684\u56fe\u50cf\u5bbd\u5ea6"}),(0,i.jsx)(n.td,{children:"\u4e0d\u8d85\u8fc74096"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"height"}),(0,i.jsx)(n.td,{children:"\u8f93\u5165\u7f16\u7801\u6a21\u5757\u7684\u56fe\u50cf\u9ad8\u5ea6"}),(0,i.jsx)(n.td,{children:"\u4e0d\u8d85\u8fc74096"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"bits"}),(0,i.jsx)(n.td,{children:"\u7f16\u7801\u6a21\u5757\u7684\u6bd4\u7279\u7387"}),(0,i.jsx)(n.td,{children:"\u9ed8\u8ba48000kbps"})]})]})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u4f7f\u7528\u65b9\u6cd5\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"#create encode object\nencode = libsrcampy.Encoder()\n\n#enable encode channel 0, solution: 1080p, format: H264\nret = encode.encode(0, 1, 1920, 1080)\n"})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u8fd4\u56de\u503c\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u8fd4\u56de\u503c"}),(0,i.jsx)(n.th,{children:"\u5b9a\u4e49\u63cf\u8ff0"})]})}),(0,i.jsxs)(n.tbody,{children:[(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"0"}),(0,i.jsx)(n.td,{children:"\u6210\u529f"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"-1"}),(0,i.jsx)(n.td,{children:"\u5931\u8d25"})]})]})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u6ce8\u610f\u4e8b\u9879\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u8003\u4ee3\u7801\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)(n.h2,{id:"encode_file",children:"encode_file"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u5411\u4f7f\u80fd\u7684\u7f16\u7801\u901a\u9053\u8f93\u5165\u56fe\u50cf\u6587\u4ef6\uff0c\u6309\u9884\u5b9a\u683c\u5f0f\u8fdb\u884c\u7f16\u7801"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u51fd\u6570\u58f0\u660e\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"Encoder.encode_file(img)\n"})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u6570\u63cf\u8ff0\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u53c2\u6570\u540d\u79f0"}),(0,i.jsx)(n.th,{children:"\u63cf\u8ff0"}),(0,i.jsx)(n.th,{children:"\u53d6\u503c\u8303\u56f4"})]})}),(0,i.jsx)(n.tbody,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"img"}),(0,i.jsx)(n.td,{children:"\u9700\u8981\u7f16\u7801\u7684\u56fe\u50cf\u6570\u636e\uff0c\u9700\u8981\u4f7f\u7528NV12\u683c\u5f0f"}),(0,i.jsx)(n.td,{children:"\u65e0"})]})})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u4f7f\u7528\u65b9\u6cd5\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:'fin = open("output.img", "rb")\ninput_img = fin.read()\nfin.close()\n\n#input image data to encode\nret = encode.encode_file(input_img)\n'})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u8fd4\u56de\u503c\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u8fd4\u56de\u503c"}),(0,i.jsx)(n.th,{children:"\u5b9a\u4e49\u63cf\u8ff0"})]})}),(0,i.jsxs)(n.tbody,{children:[(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"0"}),(0,i.jsx)(n.td,{children:"\u6210\u529f"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"-1"}),(0,i.jsx)(n.td,{children:"\u5931\u8d25"})]})]})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u6ce8\u610f\u4e8b\u9879\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u8003\u4ee3\u7801\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)(n.h2,{id:"get_img",children:"get_img"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u83b7\u53d6\u7f16\u7801\u540e\u7684\u6570\u636e"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u51fd\u6570\u58f0\u660e\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"Encoder.get_img()\n"})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u4f7f\u7528\u65b9\u6cd5\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u6570\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u8fd4\u56de\u503c\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u8fd4\u56de\u503c"}),(0,i.jsx)(n.th,{children:"\u5b9a\u4e49\u63cf\u8ff0"})]})}),(0,i.jsxs)(n.tbody,{children:[(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"0"}),(0,i.jsx)(n.td,{children:"\u6210\u529f"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"-1"}),(0,i.jsx)(n.td,{children:"\u5931\u8d25"})]})]})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u6ce8\u610f\u4e8b\u9879\u3011"}),"\n",(0,i.jsxs)(n.p,{children:["\u8be5\u63a5\u53e3\u9700\u8981\u5728\u8c03\u7528",(0,i.jsx)(n.code,{children:"Encoder.encode()"}),"\u521b\u5efa\u7f16\u7801\u901a\u9053\u540e\u4f7f\u7528"]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u8003\u4ee3\u7801\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:'import sys, os, time\n\nimport numpy as np\nimport cv2\nfrom hobot_vio import libsrcampy\n\ndef test_encode():\n    #create encode object\n    enc = libsrcampy.Encoder()\n    ret = enc.encode(0, 1, 1920, 1080)\n    print("Encoder encode return:%d" % ret)\n\n    #save encoded data to file\n    fo = open("encode.h264", "wb+")\n    a = 0\n    fin = open("output.img", "rb")\n    input_img = fin.read()\n    fin.close()\n    while a < 100:\n        #send image data to encoder\n        ret = enc.encode_file(input_img)\n        print("Encoder encode_file return:%d" % ret)\n        #get encoded data\n        img = enc.get_img()\n        if img is not None:\n            fo.write(img)\n            print("encode write image success count: %d" % a)\n        else:\n            print("encode write image failed count: %d" % a)\n        a = a + 1\n\n    enc.close()\n    print("test_encode done!!!")\n\ntest_encode()\n'})}),"\n",(0,i.jsx)(n.h2,{id:"close",children:"close"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u529f\u80fd\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u5173\u95ed\u4f7f\u80fd\u7684\u7f16\u7801\u901a\u9053\u3002"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u51fd\u6570\u58f0\u660e\u3011"}),"\n",(0,i.jsx)(n.pre,{children:(0,i.jsx)(n.code,{className:"language-python",children:"Encoder.close()\n"})}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u6570\u63cf\u8ff0\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u4f7f\u7528\u65b9\u6cd5\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u8fd4\u56de\u503c\u3011"}),"\n",(0,i.jsxs)(n.table,{children:[(0,i.jsx)(n.thead,{children:(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.th,{children:"\u8fd4\u56de\u503c"}),(0,i.jsx)(n.th,{children:"\u5b9a\u4e49\u63cf\u8ff0"})]})}),(0,i.jsxs)(n.tbody,{children:[(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"0"}),(0,i.jsx)(n.td,{children:"\u6210\u529f"})]}),(0,i.jsxs)(n.tr,{children:[(0,i.jsx)(n.td,{children:"-1"}),(0,i.jsx)(n.td,{children:"\u5931\u8d25"})]})]})]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u6ce8\u610f\u4e8b\u9879\u3011"}),"\n",(0,i.jsxs)(n.p,{children:["\u8be5\u63a5\u53e3\u9700\u8981\u5728\u8c03\u7528",(0,i.jsx)(n.code,{children:"Encoder.encode()"}),"\u521b\u5efa\u7f16\u7801\u901a\u9053\u540e\u4f7f\u7528"]}),"\n",(0,i.jsx)("font",{color:"Blue",children:"\u3010\u53c2\u8003\u4ee3\u7801\u3011"}),"\n",(0,i.jsx)(n.p,{children:"\u65e0"})]})}function x(e={}){const{wrapper:n}={...(0,c.R)(),...e.components};return n?(0,i.jsx)(n,{...e,children:(0,i.jsx)(h,{...e})}):h(e)}},28453:(e,n,d)=>{d.d(n,{R:()=>r,x:()=>t});var i=d(96540);const c={},l=i.createContext(c);function r(e){const n=i.useContext(l);return i.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function t(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(c):e.components||c:r(e.components),i.createElement(l.Provider,{value:n},e.children)}}}]);