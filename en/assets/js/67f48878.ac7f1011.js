"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[5685],{265:(e,o,t)=>{t.r(o),t.d(o,{assets:()=>l,contentTitle:()=>s,default:()=>h,frontMatter:()=>r,metadata:()=>d,toc:()=>c});var n=t(74848),i=t(28453);const r={sidebar_position:1},s="4.3.1 ModelZoo Overview",d={id:"Algorithm_Application/model_zoo/model_zoo_intro",title:"4.3.1 ModelZoo Overview",description:"Product Introduction",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/04_Algorithm_Application/03_model_zoo/model_zoo_intro.md",sourceDirName:"04_Algorithm_Application/03_model_zoo",slug:"/Algorithm_Application/model_zoo/model_zoo_intro",permalink:"/rdk_doc/en/Algorithm_Application/model_zoo/model_zoo_intro",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/04_Algorithm_Application/03_model_zoo/model_zoo_intro.md",tags:[],version:"current",sidebarPosition:1,frontMatter:{sidebar_position:1},sidebar:"tutorialSidebar",previous:{title:"4.2.9 Model Inference DEBUG Method",permalink:"/rdk_doc/en/Algorithm_Application/cdev_dnn_api/model_debug"},next:{title:"4.3.2 ModelZoo Quick Start",permalink:"/rdk_doc/en/Algorithm_Application/model_zoo/bpu_infer_lib_intro"}},l={},c=[{value:"Product Introduction",id:"product-introduction",level:2},{value:"Environment Preparation",id:"environment-preparation",level:2},{value:"Module Introduction",id:"module-introduction",level:2},{value:"Usage Guide",id:"usage-guide",level:2}];function a(e){const o={a:"a",admonition:"admonition",code:"code",h1:"h1",h2:"h2",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",...(0,i.R)(),...e.components};return(0,n.jsxs)(n.Fragment,{children:[(0,n.jsx)(o.h1,{id:"431-modelzoo-overview",children:"4.3.1 ModelZoo Overview"}),"\n",(0,n.jsx)(o.h2,{id:"product-introduction",children:"Product Introduction"}),"\n",(0,n.jsx)(o.p,{children:"This product serves as the model sample repository (Model Zoo) for the RDK series development boards, aiming to provide developers with a variety of model cases that can be directly deployed on the board."}),"\n",(0,n.jsx)(o.admonition,{title:"Tip",type:"tip",children:(0,n.jsxs)(o.p,{children:["The GitHub repository for the Model Zoo is available here: ",(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/rdk_model_zoo",children:"https://github.com/D-Robotics/rdk_model_zoo"})]})}),"\n",(0,n.jsx)(o.p,{children:"Through this repository, developers can access the following resources:"}),"\n",(0,n.jsxs)(o.ol,{children:["\n",(0,n.jsxs)(o.li,{children:[(0,n.jsx)(o.strong,{children:"Diverse Sweet Potato Heterogeneous Models"}),": The repository includes a variety of .bin models suitable for direct deployment on the board, applicable to multiple scenarios with broad versatility. These models span various fields, including but not limited to image classification, object detection, semantic segmentation, and natural language processing. Each model has been carefully selected and optimized for efficient performance."]}),"\n",(0,n.jsxs)(o.li,{children:[(0,n.jsx)(o.strong,{children:"Comprehensive Usage Guides"}),": Each model is accompanied by a Jupyter Notebook with a detailed model overview, usage instructions, example code, and annotations to help developers get started quickly. For certain models, we also provide performance evaluation reports and tuning recommendations to assist developers in customizing and optimizing the models according to specific needs."]}),"\n",(0,n.jsxs)(o.li,{children:[(0,n.jsx)(o.strong,{children:"Integrated Development Tools"}),": We provide developers with a Python interface, ",(0,n.jsx)(o.code,{children:"bpu_infer_lib"}),", for rapid model deployment on the RDK series development boards. By studying the Jupyter Notebooks included with the models, such as data preprocessing scripts and inference methods, developers can quickly master the use of this interface, significantly streamlining the model development and deployment process."]}),"\n"]}),"\n",(0,n.jsx)(o.h2,{id:"environment-preparation",children:"Environment Preparation"}),"\n",(0,n.jsxs)(o.p,{children:["Developers should first prepare an RDK development board corresponding to their branch and go to the D-Robotics official website to complete ",(0,n.jsx)(o.a,{href:"https://developer.d-robotics.cc/rdk_doc/en/install_os",children:"hardware preparation, driver installation, software download, and image burning"}),". For X3 and X5 images, please choose versions above 3.0.0."]}),"\n",(0,n.jsxs)(o.p,{children:["After completing the hardware connection and network configuration, use MobaXTerm to ",(0,n.jsx)(o.a,{href:"https://developer.d-robotics.cc/rdk_doc/Quick_start/remote_login",children:"remotely log in to the development board"}),". Connect the development board to the network ",(0,n.jsx)(o.a,{href:"https://developer.d-robotics.cc/rdk_doc/en/System_configuration/network_blueteeth",children:"here"}),"\u3002"]}),"\n",(0,n.jsx)(o.p,{children:"Use pip to install the corresponding Python libraries:"}),"\n",(0,n.jsxs)(o.ol,{children:["\n",(0,n.jsx)(o.li,{children:"bpu_infer_lib"}),"\n"]}),"\n",(0,n.jsx)(o.p,{children:"If using RDK X5:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"pip install bpu_infer_lib_x5 -i http://archive.d-robotics.cc/simple/ --trusted-host archive.d-robotics.cc\n"})}),"\n",(0,n.jsx)(o.p,{children:"If using RDK X3:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"pip install bpu_infer_lib_x3 -i http://archive.d-robotics.cc/simple/ --trusted-host archive.d-robotics.cc\n"})}),"\n",(0,n.jsxs)(o.ol,{start:"2",children:["\n",(0,n.jsx)(o.li,{children:"jupyterlab"}),"\n"]}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"pip install jupyterlab\n"})}),"\n",(0,n.jsx)(o.p,{children:"Then you can pull the Model Zoo repository with the following command:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"git clone https://github.com/D-Robotics/rdk_model_zoo\n"})}),"\n",(0,n.jsx)(o.p,{children:"Note: The branch cloned by git clone defaults to the RDK X5 branch. If the actual development board used is another product in the RDK series, please use the git checkout command to switch branches. Here is an example for switching to the RDK X3 branch:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"git checkout rdk_x3\n"})}),"\n",(0,n.jsx)(o.p,{children:"After cloning, use the following command to enter the Model Zoo directory:"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"cd rdk_model_zoo\n"})}),"\n",(0,n.jsx)(o.p,{children:"Then use the following command to enter Jupyter Lab (Note: The IP address is the actual IP used when logging in to the board):"}),"\n",(0,n.jsx)(o.pre,{children:(0,n.jsx)(o.code,{children:"jupyter lab --allow-root --ip 10.112.148.68\n"})}),"\n",(0,n.jsx)(o.p,{children:(0,n.jsx)(o.img,{src:t(64220).A+"",width:"1907",height:"574"})}),"\n",(0,n.jsx)(o.p,{children:"After using the command, the above log will appear. Hold Ctrl and click the link shown in the figure with the left mouse button to enter Jupyter Lab (as shown in the figure below). Double-click demos to select a model and experience RDK Model Zoo."}),"\n",(0,n.jsx)(o.p,{children:(0,n.jsx)(o.img,{src:t(90912).A+"",width:"1542",height:"1008"})}),"\n",(0,n.jsx)(o.h2,{id:"module-introduction",children:"Module Introduction"}),"\n",(0,n.jsx)(o.p,{children:"The RDK series Model Zoo is generally divided into the following modules (this part takes RDK X5 as an example, please switch to the corresponding branch according to the actual situation):"}),"\n",(0,n.jsxs)(o.ol,{children:["\n",(0,n.jsx)(o.li,{children:(0,n.jsx)(o.strong,{children:(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/llm",children:"Large Models"})})}),"\n",(0,n.jsx)(o.li,{children:(0,n.jsx)(o.strong,{children:(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/classification",children:"Image Classfication"})})}),"\n",(0,n.jsx)(o.li,{children:(0,n.jsx)(o.strong,{children:(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/detect",children:"Object Detection"})})}),"\n",(0,n.jsx)(o.li,{children:(0,n.jsx)(o.strong,{children:(0,n.jsx)(o.a,{href:"https://github.com/D-Robotics/rdk_model_zoo/tree/main/demos/Instance_Segmentation",children:"Instance Segmentation"})})}),"\n"]}),"\n",(0,n.jsx)(o.p,{children:"Developers can jump to the corresponding module to experience the deployment of models on RDK series development boards."}),"\n",(0,n.jsx)(o.h2,{id:"usage-guide",children:"Usage Guide"}),"\n",(0,n.jsx)(o.p,{children:"In Jupyter Lab, after selecting a model's notebook and entering, developers will arrive at an interface similar to the following:"}),"\n",(0,n.jsx)(o.p,{children:(0,n.jsx)(o.img,{src:t(72260).A+"",width:"1695",height:"635"})}),"\n",(0,n.jsx)(o.p,{children:"Here, taking the YOLO World model as an example, users only need to click the double triangle button in the above figure to run all cells. Drag the mouse to the bottom to see the result display:"}),"\n",(0,n.jsx)(o.p,{children:(0,n.jsx)(o.img,{src:t(67939).A+"",width:"1507",height:"967"})}),"\n",(0,n.jsx)(o.p,{children:"Developers can also choose to run cells one by one. In this case, just press Shift + Enter to complete the current cell and move to the next cell."})]})}function h(e={}){const{wrapper:o}={...(0,i.R)(),...e.components};return o?(0,n.jsx)(o,{...e,children:(0,n.jsx)(a,{...e})}):a(e)}},72260:(e,o,t)=>{t.d(o,{A:()=>n});const n=t.p+"assets/images/basic_usage-0bcf035695e2328d4a731196b44bed79.png"},67939:(e,o,t)=>{t.d(o,{A:()=>n});const n=t.p+"assets/images/basic_usage_res-f8e0e9872375980e3465f880521ea376.png"},90912:(e,o,t)=>{t.d(o,{A:()=>n});const n=t.p+"assets/images/into_jupyter-53d2ece1ed6a1d766b0387d831a42d35.png"},64220:(e,o,t)=>{t.d(o,{A:()=>n});const n=t.p+"assets/images/jupyter_start-04ab4b11767c9856d06b643b9e045a82.png"},28453:(e,o,t)=>{t.d(o,{R:()=>s,x:()=>d});var n=t(96540);const i={},r=n.createContext(i);function s(e){const o=n.useContext(r);return n.useMemo((function(){return"function"==typeof e?e(o):{...o,...e}}),[o,e])}function d(e){let o;return o=e.disableParentContext?"function"==typeof e.components?e.components(i):e.components||i:s(e.components),n.createElement(r.Provider,{value:o},e.children)}}}]);