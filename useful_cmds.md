# Use this command to open a GDB session with the board
sudo openocd -f /usr/share/openocd/scripts/board/stm32l4discovery.cfg

# Use this to attach to the gdb-server opened by the previous command
gdb-multiarch -f ./build/TT.elf

# Command in makefile to flash the target... need to properly test
program: $(BUILD_DIR)/$(TARGET).elf
		openocd -f $(OPENOCD_BOARD_DIR)/stm32l4discovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash $(BUILD_DIR)/$(TARGET).elf" -c shutdown

# Setting up a MQTT broker/Server
https://obrienlabs.net/how-to-setup-your-own-mqtt-broker/
username: cache
password: Falcon!97
