"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[9630],{1955:(e,t,n)=>{n.r(t),n.d(t,{assets:()=>d,contentTitle:()=>l,default:()=>p,frontMatter:()=>s,metadata:()=>c,toc:()=>u});var r=n(74848),o=n(28453),a=n(93859),i=n(19365);const s={sidebar_position:6},l="5.2.6 Model Inference",c={id:"Robot_development/quick_demo/ai_predict",title:"5.2.6 Model Inference",description:"Introduction",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/05_Robot_development/02_quick_demo/ai_predict.md",sourceDirName:"05_Robot_development/02_quick_demo",slug:"/Robot_development/quick_demo/ai_predict",permalink:"/rdk_doc/en/Robot_development/quick_demo/ai_predict",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/05_Robot_development/02_quick_demo/ai_predict.md",tags:[],version:"current",sidebarPosition:6,frontMatter:{sidebar_position:6},sidebar:"tutorialSidebar",previous:{title:"5.2.5 Communication",permalink:"/rdk_doc/en/Robot_development/quick_demo/demo_communication"},next:{title:"5.2.7 Tools",permalink:"/rdk_doc/en/Robot_development/quick_demo/demo_tool"}},d={},u=[{value:"Introduction",id:"introduction",level:2},{value:"Supported Platforms",id:"supported-platforms",level:2},{value:"Prerequisites",id:"prerequisites",level:2},{value:"RDK",id:"rdk",level:3},{value:"Usage",id:"usage",level:2},{value:"Analysis of Results",id:"analysis-of-results",level:2},{value:"Multi-Algorithm Inference",id:"multi-algorithm-inference",level:2}];function h(e){const t={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",h3:"h3",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",...(0,o.R)(),...e.components};return(0,r.jsxs)(r.Fragment,{children:[(0,r.jsx)(t.h1,{id:"526-model-inference",children:"5.2.6 Model Inference"}),"\n","\n",(0,r.jsx)(t.h2,{id:"introduction",children:"Introduction"}),"\n",(0,r.jsx)(t.p,{children:"This section introduces the usage of the model inference function. You can input a local image for inference, and get the rendered image saved locally."}),"\n",(0,r.jsx)(t.p,{children:"Finally, demonstrate the effects of simultaneous reasoning and fusion for the Human Body Detection, facial keypoint detection, Hand Keypoint Detection, and Gesture Recognition algorithms in the TROS application. The example uses MIPI/USB camera/local replay input and displays the inference rendering results through the WEB interface."}),"\n",(0,r.jsxs)(t.p,{children:["Code repository: ",(0,r.jsx)(t.a,{href:"https://github.com/D-Robotics/hobot_dnn",children:"https://github.com/D-Robotics/hobot_dnn"})]}),"\n",(0,r.jsx)(t.h2,{id:"supported-platforms",children:"Supported Platforms"}),"\n",(0,r.jsxs)(t.table,{children:[(0,r.jsx)(t.thead,{children:(0,r.jsxs)(t.tr,{children:[(0,r.jsx)(t.th,{children:"Platform"}),(0,r.jsx)(t.th,{children:"System"})]})}),(0,r.jsx)(t.tbody,{children:(0,r.jsxs)(t.tr,{children:[(0,r.jsx)(t.td,{children:"RDK X3, RDK X3 Module, RDK X5"}),(0,r.jsx)(t.td,{children:"Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)"})]})})]}),"\n",(0,r.jsx)(t.h2,{id:"prerequisites",children:"Prerequisites"}),"\n",(0,r.jsx)(t.h3,{id:"rdk",children:"RDK"}),"\n",(0,r.jsxs)(t.ol,{children:["\n",(0,r.jsxs)(t.li,{children:["\n",(0,r.jsx)(t.p,{children:"The RDK is already burned with the provided  Ubuntu 20.04/22.04 system image."}),"\n"]}),"\n",(0,r.jsxs)(t.li,{children:["\n",(0,r.jsx)(t.p,{children:"The TogetheROS.Bot has been successfully installed on the RDK."}),"\n"]}),"\n"]}),"\n",(0,r.jsx)(t.h2,{id:"usage",children:"Usage"}),"\n",(0,r.jsx)(t.p,{children:"Use the local JPEG image and model in the hobot_dnn configuration file (FCOS object detection model, supporting 80 types of object detection including human, animal, fruit, and transportation, etc.), perform inference through offline, and save the rendered image."}),"\n",(0,r.jsxs)(a.A,{groupId:"tros-distro",children:[(0,r.jsx)(i.A,{value:"foxy",label:"Foxy",children:(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-bash",children:"source /opt/tros/setup.bash\n"})})}),(0,r.jsx)(i.A,{value:"humble",label:"Humble",children:(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-bash",children:"source /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-shell",children:"# Copy the configuration file needed for the example to run from the installation path of tros.b. config contains the model used by the example and the local image used for feedback\ncp -r /opt/tros/${TROS_DISTRO}/lib/dnn_node_example/config/ .\n\n# Perform feedback prediction using the local jpg format image and save the rendered image\nros2 launch dnn_node_example dnn_node_example_feedback.launch.py dnn_example_config_file:=config/fcosworkconfig.json dnn_example_image:=config/target.jpg\n"})}),"\n",(0,r.jsx)(t.p,{children:"After a successful run, the rendered image will be automatically saved in the execution path, named as render_feedback_0_0.jpeg. Use Ctrl+C to exit the program."}),"\n",(0,r.jsx)(t.h2,{id:"analysis-of-results",children:"Analysis of Results"}),"\n",(0,r.jsx)(t.p,{children:"The terminal output during the execution of the command provides the following information:"}),"\n",(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-text",children:"[example-1] [INFO] [1679901151.612290039] [ImageUtils]: target size: 6\n[example-1] [INFO] [1679901151.612314489] [ImageUtils]: target type: couch, rois.size: 1\n[example-1] [INFO] [1679901151.612326734] [ImageUtils]: roi.type: couch, x_offset: 83 y_offset: 265 width: 357 height: 139\n[example-1] [INFO] [1679901151.612412454] [ImageUtils]: target type: potted plant, rois.size: 1\n[example-1] [INFO] [1679901151.612426522] [ImageUtils]: roi.type: potted plant, x_offset: 379 y_offset: 173 width: 131 height: 202\n[example-1] [INFO] [1679901151.612472961] [ImageUtils]: target type: book, rois.size: 1\n[example-1] [INFO] [1679901151.612497709] [ImageUtils]: roi.type: book, x_offset: 167 y_offset: 333 width: 67 height: 22\n[example-1] [INFO] [1679901151.612522859] [ImageUtils]: target type: vase, rois.size: 1\n[example-1] [INFO] [1679901151.612533487] [ImageUtils]: roi.type: vase, x_offset: 44 y_offset: 273 width: 26 height: 45\n[example-1] [INFO] [1679901151.612557172] [ImageUtils]: target type: couch, rois.size: 1\n[example-1] [INFO] [1679901151.612567740] [ImageUtils]: roi.type: couch, x_offset: 81 y_offset: 265 width: 221 height: 106\n[example-1] [INFO] [1679901151.612606444] [ImageUtils]: target type: potted plant, rois.size: 1\n[example-1] [INFO] [1679901151.612617518] [ImageUtils]: roi.type: potted plant, x_offset: 138 y_offset: 314 width: 45 height: 38\n[example-1] [WARN] [1679901151.612652352] [ImageUtils]: Draw result to file: render_feedback_0_0.jpeg\n"})}),"\n",(0,r.jsx)(t.p,{children:"The log output shows that the algorithm has inferred 6 targets based on the input image and provided the class (target type) and the coordinates of the detection boxes (x_offset and y_offset for the top left corner of the box, and width and height of the box). The rendered image file is saved as render_feedback_0_0.jpeg."}),"\n",(0,r.jsx)(t.p,{children:"The rendered image, render_feedback_0_0.jpeg, is shown below:"}),"\n",(0,r.jsx)(t.p,{children:(0,r.jsx)(t.img,{src:n(19889).A+"",width:"512",height:"512"})}),"\n",(0,r.jsx)(t.h2,{id:"multi-algorithm-inference",children:"Multi-Algorithm Inference"}),"\n",(0,r.jsx)(t.p,{children:"This section introduces the simultaneous inference of multiple algorithms, and the display of algorithm effects on the WEB\u7aef after fusing the inference results."}),"\n",(0,r.jsxs)(t.admonition,{type:"warning",children:[(0,r.jsxs)(t.p,{children:["Only ",(0,r.jsx)(t.code,{children:"TROS Humble 2.3.1"})," and later versions support this feature."]}),(0,r.jsxs)(t.p,{children:[(0,r.jsx)(t.code,{children:"TROS"})," version release records: ",(0,r.jsx)(t.a,{href:"/rdk_doc/en/Robot_development/quick_start/changelog",children:"Click to jump"}),", version check method: ",(0,r.jsx)(t.a,{href:"/rdk_doc/en/Robot_development/quick_start/install_tros",children:"Click to jump"}),"."]})]}),"\n",(0,r.jsx)(t.p,{children:(0,r.jsx)(t.strong,{children:"Publishing Images Using MIPI/USB Camera"})}),"\n",(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-bash",children:"# Configure the tros.b environment\nsource /opt/tros/humble/setup.bash\n\n# Configure MIPI camera\nexport CAM_TYPE=mipi\n# Configuration command for using a USB camera: export CAM_TYPE=usb\n\n# Launch the launch file\nros2 launch hand_gesture_detection hand_gesture_fusion.launch.py\n"})}),"\n",(0,r.jsx)(t.p,{children:(0,r.jsx)(t.strong,{children:"Using Local Image Replay"})}),"\n",(0,r.jsx)(t.pre,{children:(0,r.jsx)(t.code,{className:"language-bash",children:"# Configure the tros.b environment\nsource /opt/tros/humble/setup.bash\n# Copy the configuration files needed for the running example from the installation path of tros.b.\ncp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .\n\n# Configure local replay image\nexport CAM_TYPE=fb\n\n# Launch the launch file\nros2 launch hand_gesture_detection hand_gesture_fusion.launch.py publish_image_source:=config/person_face_hand.jpg publish_image_format:=jpg publish_output_image_w:=960 publish_output_image_h:=544 publish_fps:=30\n"})}),"\n",(0,r.jsxs)(t.p,{children:["Enter ",(0,r.jsx)(t.a,{href:"http://IP:8000",children:"http://IP:8000"})," in the browser on your PC to view the image and algorithm rendering effects (IP is the IP address of the RDK):"]}),"\n",(0,r.jsx)(t.p,{children:(0,r.jsx)(t.img,{src:n(48508).A+"",width:"1479",height:"1165"})})]})}function p(e={}){const{wrapper:t}={...(0,o.R)(),...e.components};return t?(0,r.jsx)(t,{...e,children:(0,r.jsx)(h,{...e})}):h(e)}},19365:(e,t,n)=>{n.d(t,{A:()=>i});n(96540);var r=n(34164);const o={tabItem:"tabItem_Ymn6"};var a=n(74848);function i(e){let{children:t,hidden:n,className:i}=e;return(0,a.jsx)("div",{role:"tabpanel",className:(0,r.A)(o.tabItem,i),hidden:n,children:t})}},93859:(e,t,n)=>{n.d(t,{A:()=>y});var r=n(96540),o=n(34164),a=n(86641),i=n(56347),s=n(205),l=n(38874),c=n(24035),d=n(82993);function u(e){return r.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,r.isValidElement)(e)&&function(e){const{props:t}=e;return!!t&&"object"==typeof t&&"value"in t}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function h(e){const{values:t,children:n}=e;return(0,r.useMemo)((()=>{const e=t??function(e){return u(e).map((e=>{let{props:{value:t,label:n,attributes:r,default:o}}=e;return{value:t,label:n,attributes:r,default:o}}))}(n);return function(e){const t=(0,c.X)(e,((e,t)=>e.value===t.value));if(t.length>0)throw new Error(`Docusaurus error: Duplicate values "${t.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[t,n])}function p(e){let{value:t,tabValues:n}=e;return n.some((e=>e.value===t))}function m(e){let{queryString:t=!1,groupId:n}=e;const o=(0,i.W6)(),a=function(e){let{queryString:t=!1,groupId:n}=e;if("string"==typeof t)return t;if(!1===t)return null;if(!0===t&&!n)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return n??null}({queryString:t,groupId:n});return[(0,l.aZ)(a),(0,r.useCallback)((e=>{if(!a)return;const t=new URLSearchParams(o.location.search);t.set(a,e),o.replace({...o.location,search:t.toString()})}),[a,o])]}function f(e){const{defaultValue:t,queryString:n=!1,groupId:o}=e,a=h(e),[i,l]=(0,r.useState)((()=>function(e){let{defaultValue:t,tabValues:n}=e;if(0===n.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(t){if(!p({value:t,tabValues:n}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${t}" but none of its children has the corresponding value. Available values are: ${n.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return t}const r=n.find((e=>e.default))??n[0];if(!r)throw new Error("Unexpected error: 0 tabValues");return r.value}({defaultValue:t,tabValues:a}))),[c,u]=m({queryString:n,groupId:o}),[f,g]=function(e){let{groupId:t}=e;const n=function(e){return e?`docusaurus.tab.${e}`:null}(t),[o,a]=(0,d.Dv)(n);return[o,(0,r.useCallback)((e=>{n&&a.set(e)}),[n,a])]}({groupId:o}),b=(()=>{const e=c??f;return p({value:e,tabValues:a})?e:null})();(0,s.A)((()=>{b&&l(b)}),[b]);return{selectedValue:i,selectValue:(0,r.useCallback)((e=>{if(!p({value:e,tabValues:a}))throw new Error(`Can't select invalid tab value=${e}`);l(e),u(e),g(e)}),[u,g,a]),tabValues:a}}var g=n(92303);const b={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var x=n(74848);function _(e){let{className:t,block:n,selectedValue:r,selectValue:i,tabValues:s}=e;const l=[],{blockElementScrollPositionUntilNextRender:c}=(0,a.a_)(),d=e=>{const t=e.currentTarget,n=l.indexOf(t),o=s[n].value;o!==r&&(c(t),i(o))},u=e=>{let t=null;switch(e.key){case"Enter":d(e);break;case"ArrowRight":{const n=l.indexOf(e.currentTarget)+1;t=l[n]??l[0];break}case"ArrowLeft":{const n=l.indexOf(e.currentTarget)-1;t=l[n]??l[l.length-1];break}}t?.focus()};return(0,x.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,o.A)("tabs",{"tabs--block":n},t),children:s.map((e=>{let{value:t,label:n,attributes:a}=e;return(0,x.jsx)("li",{role:"tab",tabIndex:r===t?0:-1,"aria-selected":r===t,ref:e=>l.push(e),onKeyDown:u,onClick:d,...a,className:(0,o.A)("tabs__item",b.tabItem,a?.className,{"tabs__item--active":r===t}),children:n??t},t)}))})}function j(e){let{lazy:t,children:n,selectedValue:o}=e;const a=(Array.isArray(n)?n:[n]).filter(Boolean);if(t){const e=a.find((e=>e.props.value===o));return e?(0,r.cloneElement)(e,{className:"margin-top--md"}):null}return(0,x.jsx)("div",{className:"margin-top--md",children:a.map(((e,t)=>(0,r.cloneElement)(e,{key:t,hidden:e.props.value!==o})))})}function v(e){const t=f(e);return(0,x.jsxs)("div",{className:(0,o.A)("tabs-container",b.tabList),children:[(0,x.jsx)(_,{...t,...e}),(0,x.jsx)(j,{...t,...e})]})}function y(e){const t=(0,g.A)();return(0,x.jsx)(v,{...e,children:u(e.children)},String(t))}},48508:(e,t,n)=>{n.d(t,{A:()=>r});const r=n.p+"assets/images/ai_predict_all_perc_render-23d2f963797c053990d37fb25a326358.jpg"},19889:(e,t,n)=>{n.d(t,{A:()=>r});const r=n.p+"assets/images/render1-e81c8fecfefab08621e46a49b23cdbae.jpg"},28453:(e,t,n)=>{n.d(t,{R:()=>i,x:()=>s});var r=n(96540);const o={},a=r.createContext(o);function i(e){const t=r.useContext(a);return r.useMemo((function(){return"function"==typeof e?e(t):{...t,...e}}),[t,e])}function s(e){let t;return t=e.disableParentContext?"function"==typeof e.components?e.components(o):e.components||o:i(e.components),r.createElement(a.Provider,{value:t},e.children)}}}]);