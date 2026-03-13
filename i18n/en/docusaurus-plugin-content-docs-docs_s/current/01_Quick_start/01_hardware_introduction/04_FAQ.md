---

sidebar\_position: 4

---

# 1.1.4 FAQ

### Q1：What does the Pull up/down column in the RDK S100 Pinlist Excel file mean?​  

**A:** The Pull up/down column in the Pinlist Excel file describes the default pull-up or pull-down state of the interface signal at power-on.  

### Q2：Can the input voltage range of the RDK S100 power JACK exceed 20V or be lower than 12V?  

**A:** Exceeding 20V will damage the RDK S100 development board; being lower than 12V may cause startup abnormalities, such as power-on failure or failure to boot into the kernel.  

### Q3：Where can I find the description of the power supply capability of the RDK S100 external interface output power?  

**A:** It is described in the "Tip" or "Note" information within the introduction section of each interface.  

### Q4：Can the power supply capability of the RDK S100 external interface output power exceed the maximum supply current provided by Digua?​  

**A:** It must not exceed the maximum supply current provided by Digua, otherwise it may cause damage to the development board or startup abnormalities.  

### Q5：How are the MCU sub-board and CAM sub-board connected to the RDK S100 mainboard, and are there any reference guides?  

**A: The [1.1.1 RDK S100 Series](01_rdk_s100.md)** and **[1.1.3 MCU Interface Expansion Board](03_rdk_s100_mcu_port_expansion_board.md)** sections provide reference videos demonstrating the connection of the CAM sub-board and MCU sub-board to the RDK S100 mainboard.​  

### Q6：What do characters such as AO, AI, I, O, IO, NULL, / described in the RDK S100 Pinlist Excel file mean?  

**A:**   
- **AO :** Analog Output  

- **AI :** Analog Input  

- **I :** Digital Input  

- **O :** Digital Output  

- **IO :** Digital Input or Output  

- **NULL :** Null value  

- **/ :**  Indicates no pin or no state  

### Q7：What should I do if the 3.3V pin cannot drive my sensor?  

**A:** It is recommended to check whether the power supply provided by the RDK S100 meets the electrical characteristic requirements of the sensor itself (voltage/current/level amplitude matching, etc.), and also to check for any hardware connection errors.  

### Q8：Can multiple pins be connected in parallel to increase the output current?  

**A:** RThe maximum current that all pins of the RDK S100 external interface can output is the maximum current that the peripheral can obtain. It is not possible to provide greater current capability by connecting multiple pins in parallel.  

### Q9：Why is my USB device not starting properly?  

**A:** It is recommended to check whether the hardware connection is loose, whether the USB cable is too long, whether the power consumption required by the USB device can be met by the development board, whether the USB software configuration is correct, or try replacing the USB device to confirm the issue.  ​

### Q10：Are there any relevant safety usage recommendations for the RDK S100 kit?  

**A:**    

- **a.** Follow the connection guide videos for the CAM sub-board and MCU sub-board in the community documentation to securely attach the two sub-boards. Ensure the connection is accurate and secure before supplying power to the RDK S100 mainboard. If the sub-boards are not needed, you can directly supply 12-20V power to the RDK S100 mainboard and power it on.  

- **b.** When using the RDK S100 development kit, pay attention to whether there are conductors in the surrounding environment that could make direct contact with the kit's circuit. If so, move the development kit away from exposed conductors to avoid short circuits.  

