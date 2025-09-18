case(NETWORK_TYPE)
	`FORWARD_NETWORK:
		case(ROUTER_ID)
			`ROUTER_ID_I_SYSTEM_ROUTER:
				case(target_node)
					`NODE_ID_I_SNIM_I_USERIP1_SLAVE: routing_info = 6'b 000001;
					`NODE_ID_I_SNIM_I_SYSTEM_SRAM_NO_NAME: routing_info = 6'b 000010;
					`NODE_ID_I_SNIM_COMMON_PERI_GROUP_NO_NAME: routing_info = 6'b 000100;
					`NODE_ID_I_SNIM_EXTERNAL_PERI_GROUP_NO_NAME: routing_info = 6'b 001000;
					`NODE_ID_I_SNIM_PLATFORM_CONTROLLER_NO_NAME: routing_info = 6'b 010000;
					`NODE_ID_DEFAULT_SLAVE: routing_info = 6'b 100000;
				endcase
		endcase
	`BACKWARD_NETWORK:
		case(ROUTER_ID)
			`ROUTER_ID_I_SYSTEM_ROUTER:
				case(target_node)
					`NODE_ID_I_MNIM_I_MAIN_CORE_INST: routing_info = 3'b 001;
					`NODE_ID_I_MNIM_I_MAIN_CORE_DATA: routing_info = 3'b 010;
					`NODE_ID_I_MNIM_PLATFORM_CONTROLLER_MASTER: routing_info = 3'b 100;
				endcase
		endcase
endcase