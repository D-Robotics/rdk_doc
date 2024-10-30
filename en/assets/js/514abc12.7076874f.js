"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[4140],{28547:(e,o,t)=>{t.r(o),t.d(o,{assets:()=>u,contentTitle:()=>l,default:()=>b,frontMatter:()=>s,metadata:()=>c,toc:()=>h});var n=t(74848),i=t(28453),a=t(93859),r=t(19365);const s={sidebar_position:12},l="Visual Inertial Odometry Algorithm",c={id:"Robot_development/boxs/function/hobot_vio",title:"Visual Inertial Odometry Algorithm",description:"Introduction",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/05_Robot_development/03_boxs/function/hobot_vio.md",sourceDirName:"05_Robot_development/03_boxs/function",slug:"/Robot_development/boxs/function/hobot_vio",permalink:"/rdk_doc/en/Robot_development/boxs/function/hobot_vio",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/05_Robot_development/03_boxs/function/hobot_vio.md",tags:[],version:"current",sidebarPosition:12,frontMatter:{sidebar_position:12},sidebar:"tutorialSidebar",previous:{title:"BEV\u611f\u77e5\u7b97\u6cd5",permalink:"/rdk_doc/en/Robot_development/boxs/function/hobot_bev"},next:{title:"CLIP",permalink:"/rdk_doc/en/Robot_development/boxs/function/hobot_clip"}},u={},h=[{value:"Introduction",id:"introduction",level:2},{value:"Supported Platforms",id:"supported-platforms",level:2},{value:"Prerequisites",id:"prerequisites",level:2},{value:"Usage",id:"usage",level:2},{value:"Result Analysis",id:"result-analysis",level:2}];function d(e){const o={a:"a",code:"code",h1:"h1",h2:"h2",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",table:"table",tbody:"tbody",td:"td",th:"th",thead:"thead",tr:"tr",...(0,i.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsx)(o.h1,{id:"visual-inertial-odometry-algorithm",children:"Visual Inertial Odometry Algorithm"}),"\n","\n",(0,n.jsx)(o.h2,{id:"introduction",children:"Introduction"}),"\n",(0,n.jsx)(o.p,{children:"Visual Inertial Odometry (VIO) is an algorithm that combines camera and Inertial Measurement Unit (IMU) data to achieve robot localization. VIO positioning algorithm has the advantages of low cost and wide applicability. It can effectively compensate for the failure scenarios such as obstruction and multi-path interference in satellite positioning in outdoor environments. Excellent and robust VIO algorithm is the key to achieve high-precision outdoor navigation positioning."}),"\n",(0,n.jsx)(o.p,{children:(0,n.jsx)(o.img,{src:t(18556).A+"",width:"2028",height:"1230"})}),"\n",(0,n.jsxs)(o.p,{children:["Code Repository:  (",(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/hobot_vio.git",children:"https://github.com/D-Robotics/hobot_vio.git"}),")"]}),"\n",(0,n.jsx)(o.h2,{id:"supported-platforms",children:"Supported Platforms"}),"\n",(0,n.jsxs)(o.table,{children:[(0,n.jsx)(o.thead,{children:(0,n.jsxs)(o.tr,{children:[(0,n.jsx)(o.th,{children:"Platform"}),(0,n.jsx)(o.th,{children:"System"}),(0,n.jsx)(o.th,{children:"Function"})]})}),(0,n.jsx)(o.tbody,{children:(0,n.jsxs)(o.tr,{children:[(0,n.jsx)(o.td,{children:"RDK X3, RDK X3 Module, RDK X5"}),(0,n.jsx)(o.td,{children:"Ubuntu 20.04 (Foxy), Ubuntu 22.04 (Humble)"}),(0,n.jsx)(o.td,{children:"Use realsense camera images and IMU data as algorithm inputs; Algorithm outputs robot motion trajectory that can be visualized in rviz2 on PC."})]})})]}),"\n",(0,n.jsx)(o.h2,{id:"prerequisites",children:"Prerequisites"}),"\n",(0,n.jsxs)(o.ol,{children:["\n",(0,n.jsxs)(o.li,{children:["\n",(0,n.jsx)(o.p,{children:"RDK has been flashed with the  Ubuntu 20.04/22.04 system image provided by D-Robotics."}),"\n"]}),"\n",(0,n.jsxs)(o.li,{children:["\n",(0,n.jsx)(o.p,{children:"TogetheROS.Bot and Realsense ROS2 Package have been successfully installed on RDK."}),"\n"]}),"\n",(0,n.jsxs)(o.li,{children:["\n",(0,n.jsx)(o.p,{children:"Realsense camera is connected to the USB 3.0 interface of RDK."}),"\n"]}),"\n",(0,n.jsxs)(o.li,{children:["\n",(0,n.jsx)(o.p,{children:"Confirm that the PC can access RDK through the network."}),"\n"]}),"\n"]}),"\n",(0,n.jsx)(o.h2,{id:"usage",children:"Usage"}),"\n",(0,n.jsx)(o.p,{children:"The algorithm subscribes to the images and IMU data from the realsense camera as inputs. After processing, it calculates the trajectory information of the camera and publishes the camera's motion trajectory using the ROS2 topic mechanism. The trajectory result can be viewed in the rviz2 software on the PC."}),"\n",(0,n.jsxs)(a.A,{groupId:"tros-distro",children:[(0,n.jsx)(r.A,{value:"foxy",label:"Foxy",children:(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{className:"language-bash",children:"# Configure the tros.b environment\nsource /opt/tros/setup.bash\n"})})}),(0,n.jsx)(r.A,{value:"humble",label:"Humble",children:(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{className:"language-bash",children:"# Configure the tros.b environment\nsource /opt/tros/humble/setup.bash\n"})})})]}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{className:"language-shell",children:"ros2 launch hobot_vio hobot_vio.launch.py \n"})}),"\n",(0,n.jsx)(o.h2,{id:"result-analysis",children:"Result Analysis"}),"\n",(0,n.jsx)(o.p,{children:"After starting the algorithm example on RDK, output the following information on the terminal. First, start the realsense node to publish images and IMU data. Then, the algorithm enters the initialization process, waiting for the user to move the camera to complete initialization. After initialization is completed, the algorithm starts outputting positioning coordinates:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{className:"language-text",children:"[INFO] [launch]: All log files can be found below /root/.ros/log/2023-07-07-19-48-31-464088-ubuntu-562910\n[INFO] [launch]: Default logging verbosity is set to INFO\n[INFO] [hobot_vio-1]: process started with pid [563077]\n[INFO] [ros2 launch realsense2_camera rs_launch.py  depth_module.profile:=640x480x30 enable_depth:=false enable_color:=false enable_gyro:=true enable_accel:=true enable_sync:=true gyro_fps:=200 accel_fps:=200 unite_imu_method:=2 enable_infra1:=true-2]: process started with pid [563081]\n[hobot_vio-1] T_CtoI:\n[hobot_vio-1]    0.999934   0.0103587   0.0049969   0.0270761\n[hobot_vio-1]  -0.0104067    0.999899  0.00967935 -0.00272628\n[hobot_vio-1] -0.00489613 -0.00973072    0.999941  -0.0518149\n[hobot_vio-1]           0           0           0           1\n[hobot_vio-1] system use_rtk_: 0\n[hobot_vio-1] [static initializer] not enough imu readings\n[hobot_vio-1] [static initializer] not enough imu readings\n[hobot_vio-1] [static initializer] IMU belows th 0.011508, 0.00274453 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.0105996, 0.00273085 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.00964632, 0.00280866 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.00892132, 0.00279346 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.00816016, 0.00281761 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.00776753, 0.00277049 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.00744219, 0.00274874 < 0.5, 0\n[hobot_vio-1] [static initializer] IMU belows th 0.420251, 0.36058 < 0.5, 0\n[hobot_vio-1] HorizonVIO Successfully initialized!\n[hobot_vio-1] [WARN] [1688730518.534178615] [horizon_vio_node]: Localization position[x, y, z]: [0.0225533, -0.0504654, 0.00943574]\n[hobot_vio-1] [WARN] [1688730518.534634139] [horizon_vio_node]: Image time 1688730518.314490318\n[hobot_vio-1] [WARN] [1688730518.621440869] [horizon_vio_node]: Localization position[x, y, z]: [0.0231779, -0.0533648, 0.00787081]\n[hobot_vio-1] [WARN] [1688730518.621558739] [horizon_vio_node]: Image time 1688730518.380982161\n[hobot_vio-1] [WARN] [1688730518.743525086] [horizon_vio_node]: Localization position[x, y, z]: [0.0290396, -0.0610474, 0.0106718]\n[hobot_vio-1] [WARN] [1688730518.743637249] [horizon_vio_node]: Image time 1688730518.447472572\n[hobot_vio-1] [WARN] [1688730518.866076119] [horizon_vio_node]: Localization position[x, y, z]: [0.0381324, -0.0737757, 0.0164843]\n[hobot_vio-1] [WARN] [1688730518.866186156] [horizon_vio_node]: Image time 1688730518.513962030\n[hobot_vio-1] SLAM feats: 0\n[hobot_vio-1] KF feats: 338\n[hobot_vio-1] 132.853 ms all consumed\n[hobot_vio-1] travel(m): 0.000\n[hobot_vio-1] [WARN] [1688730519.002002975] [horizon_vio_node]: Localization position[x, y, z]: [0.05018, -0.088422, 0.0240244]\n[hobot_vio-1] [WARN] [1688730519.002130095] [horizon_vio_node]: Image time 1688730518.580449104\n[hobot_vio-1] SLAM feats: 0\n[hobot_vio-1] KF feats: 31\n[hobot_vio-1] 142.996 ms all consumed\n[hobot_vio-1] travel(m): 0.014\n[hobot_vio-1] [WARN] [1688730519.146149433] [horizon_vio_node]: Localization position[x, y, z]: [0.0167176, -0.0189649, 0.0588413]\n[hobot_vio-1] [WARN] [1688730519.146279428] [horizon_vio_node]: Image time 1688730518.646935701\n[hobot_vio-1] SLAM feats: 0\n[hobot_vio-1] KF feats: 26\n[hobot_vio-1] 96.911 ms all consumed\n[hobot_vio-1] travel(m): 0.025\n[hobot_vio-1] [WARN] [1688730519.244168068] [horizon_vio_node]: Localization position[x, y, z]: [0.000805884, 0.0134815, 0.0730707]\n[hobot_vio-1] [WARN] [1688730519.244270439] [horizon_vio_node]: Image time 1688730518.713421583\n[hobot_vio-1] SLAM feats: 0\n[hobot_vio-1] KF feats: 23\n[hobot_vio-1] 52.470 ms all consumed\n[hobot_vio-1] travel(m): 0.034\n[hobot_vio-1] [WARN] [1688730519.297642444] [horizon_vio_node]: Localization position[x, y, z]: [0.00226324, 0.0120054, 0.0796328]\n[hobot_vio-1] [WARN] [1688730519.297738190] [horizon_vio_node]: Image time 1688730518.779906034\n[hobot_vio-1] SLAM feats: 0\n[hobot_vio-1] KF feats: 33\n[hobot_vio-1] 47.407 ms all consumed\n[hobot_vio-1] travel(m): 0.042\n"})})]})}function b(e={}){const{wrapper:o}={...(0,i.R)(),...e.components};return o?(0,n.jsx)(o,{...e,children:(0,n.jsx)(d,{...e})}):d(e)}},19365:(e,o,t)=>{t.d(o,{A:()=>r});t(96540);var n=t(34164);const i={tabItem:"tabItem_Ymn6"};var a=t(74848);function r(e){let{children:o,hidden:t,className:r}=e;return(0,a.jsx)("div",{role:"tabpanel",className:(0,n.A)(i.tabItem,r),hidden:t,children:o})}},93859:(e,o,t)=>{t.d(o,{A:()=>j});var n=t(96540),i=t(34164),a=t(86641),r=t(56347),s=t(205),l=t(38874),c=t(24035),u=t(82993);function h(e){return n.Children.toArray(e).filter((e=>"\n"!==e)).map((e=>{if(!e||(0,n.isValidElement)(e)&&function(e){const{props:o}=e;return!!o&&"object"==typeof o&&"value"in o}(e))return e;throw new Error(`Docusaurus error: Bad <Tabs> child <${"string"==typeof e.type?e.type:e.type.name}>: all children of the <Tabs> component should be <TabItem>, and every <TabItem> should have a unique "value" prop.`)}))?.filter(Boolean)??[]}function d(e){const{values:o,children:t}=e;return(0,n.useMemo)((()=>{const e=o??function(e){return h(e).map((e=>{let{props:{value:o,label:t,attributes:n,default:i}}=e;return{value:o,label:t,attributes:n,default:i}}))}(t);return function(e){const o=(0,c.X)(e,((e,o)=>e.value===o.value));if(o.length>0)throw new Error(`Docusaurus error: Duplicate values "${o.map((e=>e.value)).join(", ")}" found in <Tabs>. Every value needs to be unique.`)}(e),e}),[o,t])}function b(e){let{value:o,tabValues:t}=e;return t.some((e=>e.value===o))}function m(e){let{queryString:o=!1,groupId:t}=e;const i=(0,r.W6)(),a=function(e){let{queryString:o=!1,groupId:t}=e;if("string"==typeof o)return o;if(!1===o)return null;if(!0===o&&!t)throw new Error('Docusaurus error: The <Tabs> component groupId prop is required if queryString=true, because this value is used as the search param name. You can also provide an explicit value such as queryString="my-search-param".');return t??null}({queryString:o,groupId:t});return[(0,l.aZ)(a),(0,n.useCallback)((e=>{if(!a)return;const o=new URLSearchParams(i.location.search);o.set(a,e),i.replace({...i.location,search:o.toString()})}),[a,i])]}function v(e){const{defaultValue:o,queryString:t=!1,groupId:i}=e,a=d(e),[r,l]=(0,n.useState)((()=>function(e){let{defaultValue:o,tabValues:t}=e;if(0===t.length)throw new Error("Docusaurus error: the <Tabs> component requires at least one <TabItem> children component");if(o){if(!b({value:o,tabValues:t}))throw new Error(`Docusaurus error: The <Tabs> has a defaultValue "${o}" but none of its children has the corresponding value. Available values are: ${t.map((e=>e.value)).join(", ")}. If you intend to show no default tab, use defaultValue={null} instead.`);return o}const n=t.find((e=>e.default))??t[0];if(!n)throw new Error("Unexpected error: 0 tabValues");return n.value}({defaultValue:o,tabValues:a}))),[c,h]=m({queryString:t,groupId:i}),[v,p]=function(e){let{groupId:o}=e;const t=function(e){return e?`docusaurus.tab.${e}`:null}(o),[i,a]=(0,u.Dv)(t);return[i,(0,n.useCallback)((e=>{t&&a.set(e)}),[t,a])]}({groupId:i}),_=(()=>{const e=c??v;return b({value:e,tabValues:a})?e:null})();(0,s.A)((()=>{_&&l(_)}),[_]);return{selectedValue:r,selectValue:(0,n.useCallback)((e=>{if(!b({value:e,tabValues:a}))throw new Error(`Can't select invalid tab value=${e}`);l(e),h(e),p(e)}),[h,p,a]),tabValues:a}}var p=t(92303);const _={tabList:"tabList__CuJ",tabItem:"tabItem_LNqP"};var f=t(74848);function g(e){let{className:o,block:t,selectedValue:n,selectValue:r,tabValues:s}=e;const l=[],{blockElementScrollPositionUntilNextRender:c}=(0,a.a_)(),u=e=>{const o=e.currentTarget,t=l.indexOf(o),i=s[t].value;i!==n&&(c(o),r(i))},h=e=>{let o=null;switch(e.key){case"Enter":u(e);break;case"ArrowRight":{const t=l.indexOf(e.currentTarget)+1;o=l[t]??l[0];break}case"ArrowLeft":{const t=l.indexOf(e.currentTarget)-1;o=l[t]??l[l.length-1];break}}o?.focus()};return(0,f.jsx)("ul",{role:"tablist","aria-orientation":"horizontal",className:(0,i.A)("tabs",{"tabs--block":t},o),children:s.map((e=>{let{value:o,label:t,attributes:a}=e;return(0,f.jsx)("li",{role:"tab",tabIndex:n===o?0:-1,"aria-selected":n===o,ref:e=>l.push(e),onKeyDown:h,onClick:u,...a,className:(0,i.A)("tabs__item",_.tabItem,a?.className,{"tabs__item--active":n===o}),children:t??o},o)}))})}function x(e){let{lazy:o,children:t,selectedValue:i}=e;const a=(Array.isArray(t)?t:[t]).filter(Boolean);if(o){const e=a.find((e=>e.props.value===i));return e?(0,n.cloneElement)(e,{className:"margin-top--md"}):null}return(0,f.jsx)("div",{className:"margin-top--md",children:a.map(((e,o)=>(0,n.cloneElement)(e,{key:o,hidden:e.props.value!==i})))})}function y(e){const o=v(e);return(0,f.jsxs)("div",{className:(0,i.A)("tabs-container",_.tabList),children:[(0,f.jsx)(g,{...o,...e}),(0,f.jsx)(x,{...o,...e})]})}function j(e){const o=(0,p.A)();return(0,f.jsx)(y,{...e,children:h(e.children)},String(o))}},18556:(e,o,t)=>{t.d(o,{A:()=>n});const n=t.p+"assets/images/hobot_vio_rviz-bb435b4e9d1806133be8d5eacab7982f.jpeg"},28453:(e,o,t)=>{t.d(o,{R:()=>r,x:()=>s});var n=t(96540);const i={},a=n.createContext(i);function r(e){const o=n.useContext(a);return n.useMemo((function(){return"function"==typeof e?e(o):{...o,...e}}),[o,e])}function s(e){let o;return o=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:r(e.components),n.createElement(a.Provider,{value:o},e.children)}}}]);