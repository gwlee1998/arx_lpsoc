case(NETWORK_TYPE)
	`FORWARD_NETWORK:
		case(ROUTER_ID)
			`ROUTER_ID_I_SYSTEM_ROUTER:
				case(target_node)
					`NODE_ID_I_SNIM_I_SYSTEM_SRAM_NO_NAME: routing_info = 5'b 00001;
					`NODE_ID_I_SNIM_COMMON_PERI_GROUP_NO_NAME: routing_info = 5'b 00010;
					`NODE_ID_I_SNIM_EXTERNAL_PERI_GROUP_NO_NAME: routing_info = 5'b 00100;
					`NODE_ID_I_SNIM_PLATFORM_CONTROLLER_NO_NAME: routing_info = 5'b 01000;
					`NODE_ID_DEFAULT_SLAVE: routing_info = 5'b 10000;
				endcase
			`ROUTER_ID_I_USER_ROUTER:
				case(target_node)
					`NODE_ID_I_SNIM_I_USERIP1_SLAVE: routing_info = 7'b 0000001;
					`NODE_ID_I_SNIM_I_SYSTEM_DDR_NO_NAME: routing_info = 7'b 0000010;
					`NODE_ID_I_SNIM_I_SYSTEM_SRAM_NO_NAME: routing_info = 7'b 0000100;
					`NODE_ID_I_SNIM_COMMON_PERI_GROUP_NO_NAME: routing_info = 7'b 0000100;
					`NODE_ID_I_SNIM_EXTERNAL_PERI_GROUP_NO_NAME: routing_info = 7'b 0000100;
					`NODE_ID_I_SNIM_PLATFORM_CONTROLLER_NO_NAME: routing_info = 7'b 0000100;
					`NODE_ID_DEFAULT_SLAVE: routing_info = 7'b 0000100;
				endcase
		endcase
	`BACKWARD_NETWORK:
		case(ROUTER_ID)
			`ROUTER_ID_I_SYSTEM_ROUTER:
				case(target_node)
					`NODE_ID_I_MNIM_I_MAIN_CORE_INST: routing_info = 4'b 0001;
					`NODE_ID_I_MNIM_I_MAIN_CORE_DATA_C: routing_info = 4'b 0001;
					`NODE_ID_I_MNIM_I_MAIN_CORE_DATA_UC: routing_info = 4'b 0001;
					`NODE_ID_I_MNIM_PLATFORM_CONTROLLER_MASTER: routing_info = 4'b 0001;
				endcase
			`ROUTER_ID_I_USER_ROUTER:
				case(target_node)
					`NODE_ID_I_MNIM_I_MAIN_CORE_INST: routing_info = 4'b 0001;
					`NODE_ID_I_MNIM_I_MAIN_CORE_DATA_C: routing_info = 4'b 0010;
					`NODE_ID_I_MNIM_I_MAIN_CORE_DATA_UC: routing_info = 4'b 0100;
					`NODE_ID_I_MNIM_PLATFORM_CONTROLLER_MASTER: routing_info = 4'b 1000;
				endcase
		endcase
endcase