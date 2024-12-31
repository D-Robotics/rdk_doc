"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[821],{31864:(e,n,r)=>{r.r(n),r.d(n,{assets:()=>c,contentTitle:()=>d,default:()=>p,frontMatter:()=>i,metadata:()=>s,toc:()=>l});var o=r(74848),t=r(28453);const i={sidebar_position:2},d="PWM Driver Debugging Guide",s={id:"Advanced_development/linux_development/driver_development_x5/driver_pwm",title:"PWM Driver Debugging Guide",description:"The X5 has two types of controllers: one is the standard PWM, with 4 groups, each having 2 output channels, for a total of 8 PWM outputs. The other is LPWM, with 2 groups, each having 4 PWM outputs, primarily used for supporting synchronized exposure of sensors.",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/07_Advanced_development/02_linux_development/driver_development_x5/driver_pwm.md",sourceDirName:"07_Advanced_development/02_linux_development/driver_development_x5",slug:"/Advanced_development/linux_development/driver_development_x5/driver_pwm",permalink:"/rdk_doc/en/Advanced_development/linux_development/driver_development_x5/driver_pwm",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/07_Advanced_development/02_linux_development/driver_development_x5/driver_pwm.md",tags:[],version:"current",sidebarPosition:2,frontMatter:{sidebar_position:2},sidebar:"tutorialSidebar",previous:{title:"Configure U-Boot and Kernel Option Parameters",permalink:"/rdk_doc/en/Advanced_development/linux_development/driver_development_x5/uboot_kernel_config"},next:{title:"UART Driver Debugging Guide",permalink:"/rdk_doc/en/Advanced_development/linux_development/driver_development_x5/driver_uart_dev"}},c={},l=[{value:"Driver Code",id:"driver-code",level:2},{value:"Code Path",id:"code-path",level:3},{value:"Kernel Configuration",id:"kernel-configuration",level:3},{value:"DTS Node Configuration",id:"dts-node-configuration",level:3},{value:"PWM and PWMCHIP Correspondence in DTS",id:"pwm-and-pwmchip-correspondence-in-dts",level:3},{value:"Testing",id:"testing",level:2}];function a(e){const n={admonition:"admonition",code:"code",h1:"h1",h2:"h2",h3:"h3",li:"li",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,t.R)(),...e.components};return(0,o.jsxs)(o.Fragment,{children:[(0,o.jsx)(n.h1,{id:"pwm-driver-debugging-guide",children:"PWM Driver Debugging Guide"}),"\n",(0,o.jsx)(n.p,{children:"The X5 has two types of controllers: one is the standard PWM, with 4 groups, each having 2 output channels, for a total of 8 PWM outputs. The other is LPWM, with 2 groups, each having 4 PWM outputs, primarily used for supporting synchronized exposure of sensors."}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsx)(n.li,{children:"The default supported frequency range for PWM is from 0.05Hz to 100MHz, and the duty cycle register (RATIO) has a precision of 16 bits. The period valid time ranges from 10ns to 21s, and the duty cycle valid time ranges from 10ns to 21s."}),"\n",(0,o.jsx)(n.li,{children:"The default supported frequency range for LPWM is from 1Hz to 500KHz, with no duty cycle register. There is only a high-level duration register (HIGH), where the HIGH register configuration unit is in microseconds (us), and the duty cycle valid time ranges from 1us to 4ms."}),"\n",(0,o.jsxs)(n.li,{children:["LPWM is designed for sensor synchronization and is not a general-purpose PWM. ",(0,o.jsx)(n.strong,{children:"For pure PWM functionality, it is recommended to use PWM."})]}),"\n"]}),"\n",(0,o.jsx)(n.h2,{id:"driver-code",children:"Driver Code"}),"\n",(0,o.jsx)(n.h3,{id:"code-path",children:"Code Path"}),"\n",(0,o.jsx)(n.p,{children:"The PWM driver code is located at:"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-c",children:"drivers/pwm/pwm-hobot.c\n"})}),"\n",(0,o.jsx)(n.p,{children:"Path of LPWM code"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-c",children:"kernel/drivers/media/platform/horizon/camsys/lpwm/\n"})}),"\n",(0,o.jsx)(n.h3,{id:"kernel-configuration",children:"Kernel Configuration"}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-bash",children:"/* arch/arm64/configs/hobot_x5_rdk_ubuntu_defconfig */\n...\nCONFIG_HOBOT_LPWM=m\n...\nCONFIG_PWM_DROBOT=y\n...\n"})}),"\n",(0,o.jsx)(n.h3,{id:"dts-node-configuration",children:"DTS Node Configuration"}),"\n",(0,o.jsxs)(n.p,{children:["The device tree definitions for X5 PWM and LPWM controllers are located in the ",(0,o.jsx)(n.code,{children:"x5.dtsi"})," file under the ",(0,o.jsx)(n.code,{children:"arch/arm64/boot/dts/hobot/"})," folder in the SDK package."]}),"\n",(0,o.jsx)(n.admonition,{type:"note",children:(0,o.jsxs)(n.p,{children:["The nodes in ",(0,o.jsx)(n.code,{children:"x5.dtsi"})," primarily declare SoC common features and are not related to a specific circuit board. In most cases, modifications are not required."]})}),"\n",(0,o.jsxs)(n.p,{children:["When you need to enable a specific PWM port output, you can modify the corresponding board-level file. Here, as an example, we enable ",(0,o.jsx)(n.code,{children:"lpwm1_0"}),", ",(0,o.jsx)(n.code,{children:"lpwm1_1"}),", ",(0,o.jsx)(n.code,{children:"pwm0_0"}),", ",(0,o.jsx)(n.code,{children:"pwm0_1"}),", ",(0,o.jsx)(n.code,{children:"pwm1_0"}),", ",(0,o.jsx)(n.code,{children:"pwm1_1"}),", ",(0,o.jsx)(n.code,{children:"pwm2_0"}),", ",(0,o.jsx)(n.code,{children:"pwm2_1"}),", ",(0,o.jsx)(n.code,{children:"pwm3_0"}),", and ",(0,o.jsx)(n.code,{children:"pwm3_1"})," in the ",(0,o.jsx)(n.code,{children:"x5-rdk-v1p0.dts"})," file."]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-c",children:'&lpwm1 {\n\tstatus = "okay";\n\tpinctrl-names = "default";\n\t/** for display backlight **/\n\tpinctrl-0 = <&pinctrl_lpwm1_0 &pinctrl_lpwm1_1>;\n};\n\n&pwm0 {\n\tstatus = "okay";\n\tpinctrl-names = "default";\n\tpinctrl-0 = <&pinctrl_pwm0_0 &pinctrl_pwm0_1>;\n};\n\n&pwm1 {\n\tstatus = "okay";\n\tpinctrl-names = "default";\n\tpinctrl-0 = <&pinctrl_pwm1_0 &pinctrl_pwm1_1>;\n};\n\n&pwm2 {\n\tstatus = "okay";\n\tpinctrl-names = "default";\n\tpinctrl-0 = <&pinctrl_pwm2_0 &pinctrl_pwm2_1>;\n};\n\n&pwm3 {\n\t/* LSIO_PWM_OUT6 and LSIO_PWM_OUT7 */\n\tstatus = "okay";\n\tpinctrl-names = "default";\n\tpinctrl-0 = <&pinctrl_pwm3_0 &pinctrl_pwm3_1>;\n};\n'})}),"\n",(0,o.jsx)(n.h3,{id:"pwm-and-pwmchip-correspondence-in-dts",children:"PWM and PWMCHIP Correspondence in DTS"}),"\n",(0,o.jsxs)(n.p,{children:["Although both PWM and LPWM belong to ",(0,o.jsx)(n.code,{children:"pwmchip"}),", the number of devices under PWM/LPWM is not consistent. Therefore, fixed numbering through aliases is not possible. When operating PWM on the board, you need to use the ",(0,o.jsx)(n.code,{children:"cat"})," command to view the ",(0,o.jsx)(n.code,{children:"device/uevent"})," under ",(0,o.jsx)(n.code,{children:"pwmchip"})," to check if the PWM address matches the target PWM address."]}),"\n",(0,o.jsxs)(n.p,{children:["For example, to check the ",(0,o.jsx)(n.code,{children:"pwmchip"})," uevent for ",(0,o.jsx)(n.code,{children:"pwm0"}),", use the following command on the board:"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"cat /sys/class/pwm/pwmchip0/device/uevent\nDRIVER=drobot-pwm\nOF_NAME=pwm\nOF_FULLNAME=/soc/a55_apb0/pwm@34140000\nOF_COMPATIBLE_0=d-robotics,pwm\nOF_COMPATIBLE_N=1\nMODALIAS=of:NpwmT(null)Cd-robotics,pwm\n"})}),"\n",(0,o.jsx)(n.h2,{id:"testing",children:"Testing"}),"\n",(0,o.jsx)(n.p,{children:"Users can refer to the following commands to test the PWM functionality and perform signal measurements to verify whether the PWM is working correctly. For specific hardware pins to be measured, users should refer to the documentation provided for the hardware they are using."}),"\n",(0,o.jsx)(n.p,{children:"The following commands are provided as an example to verify PWM0 channel 0."}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-shell",children:'cd /sys/class/pwm/pwmchip0/\necho 0 > export\ncd pwm0\n\n# Configure Period to 100us\necho 100000 > period\n\n# Configure Duty Cycle to 50% = 100us * 0.5 = 50us\necho 50000 > duty_cycle\n\n# Enable PWM Output\necho 1 > enable\n\n# The following is for reading register values\n\necho "Regs of PWM 3:"\necho "PWM_EN       `devmem 0x34170000 32`"\necho "PWM_INT_CTRL `devmem 0x34170004 32`"\necho "PWM0_CTRL    `devmem 0x34170008 32`"\necho "PWM0_CLK     `devmem 0x34170010 32`"\necho "PWM0_PERIOD  `devmem 0x34170020 32`"\necho "PWM0_STATUS  `devmem 0x34170028 32`"\necho "PWM1_CTRL    `devmem 0x34170030 32`"\necho "PWM1_CLK     `devmem 0x34170034 32`"\necho "PWM1_PERIOD  `devmem 0x34170040 32`"\necho "PWM1_STATUS  `devmem 0x34170048 32`"\n'})})]})}function p(e={}){const{wrapper:n}={...(0,t.R)(),...e.components};return n?(0,o.jsx)(n,{...e,children:(0,o.jsx)(a,{...e})}):a(e)}},28453:(e,n,r)=>{r.d(n,{R:()=>d,x:()=>s});var o=r(96540);const t={},i=o.createContext(t);function d(e){const n=o.useContext(i);return o.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function s(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(t):e.components||t:d(e.components),o.createElement(i.Provider,{value:n},e.children)}}}]);