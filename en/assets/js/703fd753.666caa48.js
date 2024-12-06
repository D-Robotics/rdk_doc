"use strict";(self.webpackChunkrdk_doc=self.webpackChunkrdk_doc||[]).push([[8038],{17206:(e,n,i)=>{i.r(n),i.d(n,{assets:()=>t,contentTitle:()=>s,default:()=>h,frontMatter:()=>a,metadata:()=>r,toc:()=>c});var o=i(74848),d=i(28453);const a={sidebar_position:3},s="3.2.3 RDK X5 Series Audio Guide",r={id:"Basic_Application/audio/audio_board_x5",title:"3.2.3 RDK X5 Series Audio Guide",description:"The RDK X5 integrates the ES8326 audio codec, and users can also connect audio boards to extend the audio capabilities to meet the needs of various voice scenarios. This section will provide a detailed guide on using the onboard audio codec and audio boards.",source:"@site/i18n/en/docusaurus-plugin-content-docs/current/03_Basic_Application/02_audio/audio_board_x5.md",sourceDirName:"03_Basic_Application/02_audio",slug:"/Basic_Application/audio/audio_board_x5",permalink:"/rdk_doc/en/Basic_Application/audio/audio_board_x5",draft:!1,unlisted:!1,editUrl:"https://github.com/D-Robotics/rdk_doc/blob/main/docs/03_Basic_Application/02_audio/audio_board_x5.md",tags:[],version:"current",sidebarPosition:3,frontMatter:{sidebar_position:3},sidebar:"tutorialSidebar",previous:{title:"3.2.2 Audio Adapter Board On RDK X3 MD",permalink:"/rdk_doc/en/Basic_Application/audio/audio_board_x3_md"},next:{title:"3.3.1 Pin Configuration and Definition",permalink:"/rdk_doc/en/Basic_Application/03_40pin_user_guide/40pin_define"}},t={},c=[{value:"Onboard ES8326",id:"onboard-es8326",level:2},{value:"Usage",id:"usage",level:3},{value:"Recording and Playback Test",id:"recording-and-playback-test",level:3},{value:"Audio Driver HAT REV2",id:"audio-driver-hat-rev2",level:2},{value:"Product Introduction",id:"product-introduction",level:3},{value:"Installation Method",id:"installation-method",level:3},{value:"Uninstallation Method",id:"uninstallation-method",level:3},{value:"Audio Nodes",id:"audio-nodes",level:3},{value:"Recording and Playback Test",id:"recording-and-playback-test-1",level:3}];function l(e){const n={a:"a",admonition:"admonition",br:"br",code:"code",h1:"h1",h2:"h2",h3:"h3",img:"img",li:"li",ol:"ol",p:"p",pre:"pre",strong:"strong",ul:"ul",...(0,d.R)(),...e.components},{Details:a}=n;return a||function(e,n){throw new Error("Expected "+(n?"component":"object")+" `"+e+"` to be defined: you likely forgot to import, pass, or provide it.")}("Details",!0),(0,o.jsxs)(o.Fragment,{children:[(0,o.jsx)(n.h1,{id:"323-rdk-x5-series-audio-guide",children:"3.2.3 RDK X5 Series Audio Guide"}),"\n",(0,o.jsx)(n.p,{children:"The RDK X5 integrates the ES8326 audio codec, and users can also connect audio boards to extend the audio capabilities to meet the needs of various voice scenarios. This section will provide a detailed guide on using the onboard audio codec and audio boards."}),"\n",(0,o.jsx)(n.admonition,{title:"Tip",type:"note",children:(0,o.jsxs)(n.p,{children:["If after installing the driver, you get a message that the Miniboot version is not the latest, please go to ",(0,o.jsx)(n.code,{children:"1 System Options"})," -> ",(0,o.jsx)(n.code,{children:"S7 Update Miniboot"})," to update Miniboot."]})}),"\n",(0,o.jsx)(n.h2,{id:"onboard-es8326",children:"Onboard ES8326"}),"\n",(0,o.jsx)(n.p,{children:"The onboard audio codec ES8326 provides basic audio functionality. You need to connect a 3.5mm audio interface to record and play sound."}),"\n",(0,o.jsx)(n.h3,{id:"usage",children:"Usage"}),"\n",(0,o.jsxs)(n.p,{children:["You can use user-space interfaces like ",(0,o.jsx)(n.code,{children:"amix"})," or ",(0,o.jsx)(n.code,{children:"tinyalsa"})," to control the audio. ",(0,o.jsx)(n.strong,{children:"Note!!!"})," When adjusting the gain or testing, it's best not to wear headphones during operation to avoid sudden noise or high volume that could damage your hearing. ",(0,o.jsx)(n.strong,{children:"The safest approach is"}),": execute the command first, observe the loudness or noise, and then adjust accordingly within the safe limits."]}),"\n",(0,o.jsxs)(a,{children:[(0,o.jsxs)("summary",{children:["Click here for usage tips on ",(0,o.jsx)(n.code,{children:"amixer"})]}),(0,o.jsx)(n.p,{children:"We have found that some users, when plugging in USB audio devices, may encounter issues where the commands below don't work as expected."}),(0,o.jsxs)(n.p,{children:["In embedded audio devices, you can generally see how many sound cards are available by running ",(0,o.jsx)(n.code,{children:"cat /proc/asound/cards"}),". After plugging in a USB device and rebooting, the UAC (USB Audio Class) device sound card registers first. Running the command mentioned above may show output similar to this:"]}),(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"  0 [RC08           ]: USB-Audio - ROCWARE RC08\n                      ROCWARE RC08 at usb-xhci-hcd.2.auto-1.2, high speed\n  1 [duplexaudio    ]: simple-card - duplex-audio\n                      duplex-audio\nAt this point, we notice that the onboard audio sound card's index has changed to 1. Since `amixer` or `tinymix` default to using the device with index 0 when no specific device or card index is specified, we can use the following commands to view the controls and properties of the onboard sound card:\n"})}),(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"  amixer -D 0 -c 1 controls\n"})}),(0,o.jsxs)(n.p,{children:["If you want to adjust the microphone gain using ",(0,o.jsx)(n.code,{children:"amixer"}),", you can use the following command:"]}),(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"  amixer -D 0 -c 1 sget 'ADC PGA Gain',0\n"})})]}),"\n",(0,o.jsx)(n.h3,{id:"recording-and-playback-test",children:"Recording and Playback Test"}),"\n",(0,o.jsx)(n.p,{children:"You can use the following commands to test if recording and playback work properly:"}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Recording Command"}),":"]}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"# arecord -Dhw:0,0 -c 2 -r 48000 -f S24_LE -t wav -d 10 /userdata/record1.wav\nRecording WAVE '/userdata/record1.wav' : Signed 24 bit Little Endian, Rate 48000 Hz, Stereo\n#\n"})}),"\n",(0,o.jsxs)(n.p,{children:["You should see normal recording logs on the screen. Wait for about 10 seconds (the ",(0,o.jsx)(n.code,{children:"-d 10"})," in the command means 10 seconds). After the recording is complete, you can play the recorded audio using the following command:"]}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Playback Command"}),":"]}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"# aplay -D hw:0,0 /userdata/record1.wav\nPlaying WAVE '/userdata/record1.wav' : Signed 24 bit Little Endian, Rate 48000 Hz, Stereo\n#\n"})}),"\n",(0,o.jsx)(n.p,{children:"If you find that the recorded sound is too quiet, you can check and adjust the microphone gain using the following commands."}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Query Command"}),":"]}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"# amixer sget 'ADC PGA Gain',0\nSimple mixer control 'ADC PGA Gain',0\n  Capabilities: volume volume-joined\n  Playback channels: Mono\n  Capture channels: Mono\n  Limits: 0 - 10\n  Mono: 0 [0%] [0.00dB]\n"})}),"\n",(0,o.jsx)(n.p,{children:'After executing the above command, you should see the return value "Mono: 0 [0%] [0.00dB]", which indicates that the current value is 0.'}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsxs)(n.li,{children:[(0,o.jsx)(n.strong,{children:"Adjust Gain Command"}),":"]}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"# amixer sset 'ADC PGA Gain',0 10\nSimple mixer control 'ADC PGA Gain',0\n  Capabilities: volume volume-joined\n  Playback channels: Mono\n  Capture channels: Mono\n  Limits: 0 - 10\n  Mono: 10 [100%] [30.00dB]\n"})}),"\n",(0,o.jsx)(n.p,{children:"The above command sets the ADC PGA Gain to its maximum, which is 10."}),"\n",(0,o.jsx)(n.h2,{id:"audio-driver-hat-rev2",children:"Audio Driver HAT REV2"}),"\n",(0,o.jsx)(n.admonition,{title:"Note",type:"note",children:(0,o.jsx)(n.p,{children:"After installing the audio daughter card driver, card 0 is the device registered by the daughter card, and the original onboard audio is now card 1."})}),"\n",(0,o.jsx)(n.h3,{id:"product-introduction",children:"Product Introduction"}),"\n",(0,o.jsx)(n.p,{children:"The Audio Driver HAT REV2 is an audio expansion board produced by Waveshare. It uses a dual Codec solution with ES7210 + ES8156, enabling features such as circular 4-microphone recording, dual-channel audio playback, and audio signal loopback. The appearance of the expansion board is shown in the following image:"}),"\n",(0,o.jsx)(n.p,{children:(0,o.jsx)(n.img,{alt:"image-audio-driver-hat",src:i(46564).A+"",width:"425",height:"371"})}),"\n",(0,o.jsxs)(n.p,{children:["For more detailed information about the audio expansion board, please refer to ",(0,o.jsx)(n.a,{href:"https://www.waveshare.net/shop/Audio-Driver-HAT.htm",children:"Audio Driver HAT"}),"."]}),"\n",(0,o.jsx)(n.h3,{id:"installation-method",children:"Installation Method"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:["Connect the expansion board to the 40-pin header of the RDK X5 as shown in the image below:",(0,o.jsx)(n.br,{}),"\n",(0,o.jsx)(n.img,{alt:"image-x5-audio-driver-hat-v2",src:i(61637).A+"",width:"706",height:"1064"}),(0,o.jsx)(n.br,{}),"\n","Make sure to switch all dip switches to ",(0,o.jsx)(n.strong,{children:"off"}),"."]}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:["Use ",(0,o.jsx)(n.code,{children:"raspi-config"})," to configure the audio board:",(0,o.jsx)(n.br,{}),"\n","Go to ",(0,o.jsx)(n.code,{children:"3 Interface Options"})," -> ",(0,o.jsx)(n.code,{children:"I5 Audio"}),(0,o.jsx)(n.br,{}),"\n","Select ",(0,o.jsx)(n.code,{children:"Audio Driver HAT V2"}),":",(0,o.jsx)(n.br,{}),"\n",(0,o.jsx)(n.img,{alt:"image-audio-driver-hat-config02",src:i(72607).A+"",width:"836",height:"412"})]}),"\n"]}),"\n",(0,o.jsxs)(n.li,{children:["\n",(0,o.jsxs)(n.p,{children:["Run the command ",(0,o.jsx)(n.code,{children:"sync && reboot"})," to reboot the development board. If the following device nodes appear under ",(0,o.jsx)(n.code,{children:"/dev/snd"}),", it indicates that the expansion board has been successfully installed."]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-shell",children:"root@ubuntu:/userdata# ls /dev/snd\nby-path  controlC0  controlC1  pcmC0D0c  pcmC0D1p  pcmC1D0c  pcmC1D0p  timer\n"})}),"\n",(0,o.jsxs)(n.p,{children:["Among them, ",(0,o.jsx)(n.code,{children:"pcmC0D0c"})," and ",(0,o.jsx)(n.code,{children:"pcmC0D1p"})," are the audio devices registered by the ES7210 + ES8156, while ",(0,o.jsx)(n.code,{children:"pcmC1D0c"})," and ",(0,o.jsx)(n.code,{children:"pcmC1D0p"})," are the audio devices registered by the onboard audio."]}),"\n"]}),"\n"]}),"\n",(0,o.jsx)(n.h3,{id:"uninstallation-method",children:"Uninstallation Method"}),"\n",(0,o.jsxs)(n.ol,{children:["\n",(0,o.jsxs)(n.li,{children:["Use ",(0,o.jsx)(n.code,{children:"raspi-config"})," to configure the audio board:",(0,o.jsx)(n.br,{}),"\n","Go to ",(0,o.jsx)(n.code,{children:"3 Interface Options"})," -> ",(0,o.jsx)(n.code,{children:"I5 Audio"}),(0,o.jsx)(n.br,{}),"\n","Select ",(0,o.jsx)(n.code,{children:"UNSET"})," to uninstall the audio driver and related configurations."]}),"\n"]}),"\n",(0,o.jsx)(n.h3,{id:"audio-nodes",children:"Audio Nodes"}),"\n",(0,o.jsxs)(n.p,{children:["The playback node for this audio board on the ",(0,o.jsx)(n.code,{children:"RDK X5"})," is ",(0,o.jsx)(n.code,{children:"pcmC1D1p"}),", and the recording node is ",(0,o.jsx)(n.code,{children:"pcmC1D0c"}),"."]}),"\n",(0,o.jsx)(n.h3,{id:"recording-and-playback-test-1",children:"Recording and Playback Test"}),"\n",(0,o.jsxs)(n.p,{children:["This test uses the ",(0,o.jsx)(n.code,{children:"tinyalsa"})," library tools: ",(0,o.jsx)(n.code,{children:"tinycap"})," for recording and ",(0,o.jsx)(n.code,{children:"tinyplay"})," for playback."]}),"\n",(0,o.jsxs)(n.p,{children:["Usage instructions for ",(0,o.jsx)(n.code,{children:"tinycap"}),":"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-shell",children:"tinycap\nUsage: tinycap {file.wav | --} [-D card] [-d device] [-c channels] [-r rate] [-b bits] [-p period_size] [-n n_periods] [-t time_in_seconds]\n\nUse -- for filename to send raw PCM to stdout\n"})}),"\n",(0,o.jsxs)(n.p,{children:[(0,o.jsx)(n.code,{children:"tinyplay"})," Usage Instructions:"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{className:"language-shell",children:"tinyplay\nusage: tinyplay file.wav [options]\noptions:\n-D | --card   <card number>    The device to receive the audio\n-d | --device <device number>  The card to receive the audio\n-p | --period-size <size>      The size of the PCM's period\n-n | --period-count <count>    The number of PCM periods\n-i | --file-type <file-type >  The type of file to read (raw or wav)\n-c | --channels <count>        The amount of channels per frame\n-r | --rate <rate>             The amount of frames per second\n-b | --bits <bit-count>        The number of bits in one sample\n-M | --mmap                    Use memory mapped IO to play audio\n"})}),"\n",(0,o.jsxs)(n.p,{children:["If you want to learn more about the ",(0,o.jsx)(n.code,{children:"tinyalsa"})," library, please refer to its ",(0,o.jsx)(n.a,{href:"https://github.com/tinyalsa/tinyalsa",children:"repository"}),"."]}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsx)(n.li,{children:"2-channel microphone recording:"}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"tinycap ./2chn_test.wav -D 0 -d 1 -c 2 -b 16 -r 48000 -p 512 -n 4 -t 5\n"})}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsx)(n.li,{children:"4-channel microphone recording:"}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"tinycap ./4chn_test.wav -D 0 -d 1 -c 4 -b 16 -r 48000 -p 512 -n 4 -t 5\n"})}),"\n",(0,o.jsxs)(n.ul,{children:["\n",(0,o.jsx)(n.li,{children:"Stereo audio playback (does not support direct playback of 4-channel recordings):"}),"\n"]}),"\n",(0,o.jsx)(n.pre,{children:(0,o.jsx)(n.code,{children:"tinyplay ./2chn_test.wav -D 0 -d 0\n"})})]})}function h(e={}){const{wrapper:n}={...(0,d.R)(),...e.components};return n?(0,o.jsx)(n,{...e,children:(0,o.jsx)(l,{...e})}):l(e)}},72607:(e,n,i)=>{i.d(n,{A:()=>o});const o=i.p+"assets/images/image-audio-driver-hat-config02-919e126bed44acac05b0dce660b9a2df.png"},46564:(e,n,i)=>{i.d(n,{A:()=>o});const o=i.p+"assets/images/image-audio-driver-hat-395081612ff49a92aa58af08066aaf05.jpg"},61637:(e,n,i)=>{i.d(n,{A:()=>o});const o=i.p+"assets/images/image-x5-audio-driver-hat-v2-750d4c4ccdea42fb5c2672a5b985d1c4.png"},28453:(e,n,i)=>{i.d(n,{R:()=>s,x:()=>r});var o=i(96540);const d={},a=o.createContext(d);function s(e){const n=o.useContext(a);return o.useMemo((function(){return"function"==typeof e?e(n):{...n,...e}}),[n,e])}function r(e){let n;return n=e.disableParentContext?"function"==typeof e.components?e.components(d):e.components||d:s(e.components),o.createElement(a.Provider,{value:n},e.children)}}}]);