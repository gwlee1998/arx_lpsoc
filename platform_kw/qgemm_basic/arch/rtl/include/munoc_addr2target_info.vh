always@(*)
begin
	target_node = 0;
	if (0)
		;
	else if ($unsigned(addr[31:30])==(32'h 0))
		target_node = `NODE_ID_I_SNIM_I_SYSTEM_DDR_NO_NAME;
	else if ($unsigned(addr[31:16])==(32'h c000))
		target_node = `NODE_ID_I_SNIM_I_VTA00_CONFIG;
	else if ($unsigned(addr[31:15])==(32'h 18002))
		target_node = `NODE_ID_I_SNIM_I_DCA_MATRIX_QGEMM_MQ2VTA;
	else if ($unsigned(addr[31:16])==(32'h e000))
		target_node = `NODE_ID_I_SNIM_I_SYSTEM_SRAM_NO_NAME;
	else if ($unsigned(addr[31:18])==(32'h 3840))
		target_node = `NODE_ID_I_SNIM_PLATFORM_CONTROLLER_NO_NAME;
	else if ($unsigned(addr[31:16])==(32'h e200))
		target_node = `NODE_ID_I_SNIM_COMMON_PERI_GROUP_NO_NAME;
	else if ($unsigned(addr[31:16])==(32'h e201))
		target_node = `NODE_ID_I_SNIM_EXTERNAL_PERI_GROUP_NO_NAME;
	else if ($unsigned(addr[31:12])==(32'h e2020))
		target_node = `NODE_ID_I_SNIM_I_DCA_MATRIX_QGEMM_CONTROL_MMIOX1_INTERFACE_MMIO;
	else if ($unsigned(addr[31:15])==(32'h 1e000))
		target_node = `NODE_ID_CORE_PERI_GROUP_NO_NAME;
	else
		target_node = `NODE_ID_DEFAULT_SLAVE;
end