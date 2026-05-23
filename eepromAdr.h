#ifndef __EEPROM_ADR_H__
#define __EEPROM_ADR_H__

#define DFU_NUMBER_LOGIC							0
#define EA_DP1_UPPER_ALARM_OFF_LIMIT        (64*2) 
#define EA_DP1_UPPER_ALARM_ON_LIMIT         (65*2) 
#define EA_DP1_LOWER_ALARM_OFF_LIMIT        (66*2) 
#define EA_DP1_LOWER_ALARM_ON_LIMIT         (67*2) 
#define EA_DP2_UPPER_ALARM_OFF_LIMIT       (68*2) 
#define EA_DP2_UPPER_ALARM_ON_LIMIT        (69*2) 
#define EA_DP2_LOWER_ALARM_OFF_LIMIT       (70*2) 
#define EA_DP2_LOWER_ALARM_ON_LIMIT        (71*2) 
#define EA_DP3_UPPER_ALARM_OFF_LIMIT      (72*2)
#define EA_DP3_UPPER_ALARM_ON_LIMIT       (73*2)
#define EA_DP3_LOWER_ALARM_OFF_LIMIT      (74*2)
#define EA_DP3_LOWER_ALARM_ON_LIMIT       (75*2)
#define EA_TEMP_UPPER_ALARM_OFF_LIMIT            (76*2) 
#define EA_TEMP_UPPER_ALARM_ON_LIMIT             (77*2) 
#define EA_TEMP_LOWER_ALARM_OFF_LIMIT            (78*2) 
#define EA_TEMP_LOWER_ALARM_ON_LIMIT             (79*2) 
#define EA_RH_UPPER_ALARM_OFF_LIMIT              (80*2) 
#define EA_RH_UPPER_ALARM_ON_LIMIT               (81*2) 
#define EA_RH_LOWER_ALARM_OFF_LIMIT              (82*2) 
#define EA_RH_LOWER_ALARM_ON_LIMIT               (83*2) 
#define EA_DISPLAY_INTERVAL                      (84*2) 
#define EA_LOGGING_INTERVAL                      (85*2)  
#define EA_DEVICE_ID                             (86*2) 
#define EA_BROADCAST_INTERVAL                    (87*2) 
#define EA_DP1_OUT_LOW_PRES                 (88*2) 
#define EA_DP1_OUT_HIGH_PRES                (89*2) 
#define EA_DP2_OUT_LOW_PRES                (90*2) 
#define EA_DP2_OUT_HIGH_PRES               (91*2) 
#define EA_TEMP_OUT_LOW_TEMP                     (92*2) 
#define EA_TEMP_OUT_HIGH_TEMP                    (93*2) 
#define EA_RH_OUT_LOW_RH                         (94*2) 
#define EA_RH_OUT_HIGH_RH                        (95*2) 
#define EA_DP1_OUT_LOW_COUNT                (96*2) 
#define EA_DP1_OUT_HIGH_COUNT               (97*2) 
#define EA_DP2_OUT_LOW_COUNT               (98*2) 
#define EA_DP2_OUT_HIGH_COUNT              (99*2) 
#define EA_TEMP_OUT_LOW_COUNT                    (100*2) 
#define EA_TEMP_OUT_HIGH_COUNT                   (101*2) 
#define EA_RH_OUT_LOW_COUNT                      (102*2) 
#define EA_RH_OUT_HIGH_COUNT                     (103*2) 
#define EA_DP1_UNIT                     (104*2)
#define EA_DP2_UNIT                    (105*2)
#define EA_DP3_UNIT                   (106*2)
#define EA_TEMPERATURE_UNIT                      (107*2)
#define EA_HUMIDITY_UNIT                         (108*2)
#define EA_SERIAL_BAUD_RATE_VAL                  (109*2)
#define EA_SERIAL_DATA_STOP_PARITY_VAL           (110*2)
#define EA_DISPLAY_MODE 				 		 (111*2)
#define EA_TEMP_ZERO_ADJ						 (112*2)
#define EA_RH_ZERO_ADJ							 (113*2)
#define EA_DP1_ZERO_ADJ				 	 (114*2)
#define EA_DP2_ZERO_ADJ					 (115*2)
#define EA_DP3_ZERO_ADJ					 (116*2)
#define EA_DP1_RANGE				         (117*2)		
#define EA_DP2_RANGE					     (118*2)	
#define EA_DP3_RANGE					     (119*2)	
#define EA_BUZZER_ON_TIME				         (120*2)		
#define EA_BUZZER_OFF_TIME					     (121*2)		
#define EA_DP1_OUTPUT_TYPE                  (122*2)
#define EA_DP1_PID_KP                  	 (123*2)
#define EA_DP1_PID_TI                  	 (124*2)
#define EA_DP1_PID_TD                  	 (125*2)
#define EA_DP1_PID_SET_VALUE                (126*2)
#define EA_DP2_OUTPUT_TYPE                 (127*2)
#define EA_DP2_PID_KP                  	 (128*2)
#define EA_DP2_PID_TI                  	 (129*2)
#define EA_DP2_PID_TD                  	 (130*2)
#define EA_DP2_PID_SET_VALUE               (131*2)
#define EA_TEMP_OUTPUT_TYPE                  	 (132*2)
#define EA_TEMP_PID_KP                  		 (133*2)
#define EA_TEMP_PID_TI                  		 (134*2)
#define EA_TEMP_PID_TD                  		 (135*2)
#define EA_TEMP_PID_SET_VALUE                    (136*2)
#define EA_RH_OUTPUT_TYPE                  		 (137*2)
#define EA_RH_PID_KP                  			 (138*2)
#define EA_RH_PID_TI                  			 (139*2)
#define EA_RH_PID_TD                  			 (140*2)
#define EA_RH_PID_SET_VALUE                  	 (141*2)
#define EA_TEMP_RH_SENS_TYPE                     (142*2)
#define EA_BUZZER_DISABLE_TIME                   (143*2)
#define EA_DP1_ALARM_SETUP                  (144*2)
#define EA_DP2_ALARM_SETUP                 (145*2)
#define EA_DP3_ALARM_SETUP                (146*2)
#define EA_TEMP_ALARM_SETUP                      (147*2)
#define EA_RH_ALARM_SETUP                        (148*2)
#define EA_DP1_OUT_POLARITY	             (149*2)
#define EA_DP2_OUT_POLARITY			     (150*2)
#define EA_TEMP_OUT_POLARITY				     (151*2)
#define EA_RH_OUT_POLARITY				         (152*2)
#define EA_TEMP_HIGH_PID_RH_CASCADE_FACTOR       (153*2)
#define EA_TEMP_LOW_PID_RH_CASCADE_FACTOR        (154*2)
#define EA_TEMP_HIGH_PID_CASCADE_RH_STATE	     (155*2)
#define EA_TEMP_LOW_PID_CASCADE_RH_STATE         (156*2)
#define EA_RH_HIGH_PID_TEMP_CASCADE_FACTOR       (157*2)
#define EA_RH_LOW_PID_TEMP_CASCADE_FACTOR        (158*2)
#define EA_RH_HIGH_PID_CASCADE_TEMP_STATE	     (159*2)
#define EA_RH_LOW_PID_CASCADE_TEMP_STATE         (160*2)
#define EA_DP1_PID_STARTUP_PERCENT               (161*2)
#define EA_DP2_PID_STARTUP_PERCENT               (162*2)
#define EA_TEMP_PID_STARTUP_PERCENT              (163*2)
#define EA_RH_PID_STARTUP_PERCENT                (164*2)
#define EA_TEMP_HIGH_PID_RH_ADJUST               (165*2)
#define EA_TEMP_LOW_PID_RH_ADJUST                (166*2)
#define EA_RH_HIGH_PID_TEMP_ADJUST               (167*2)
#define EA_RH_LOW_PID_TEMP_ADJUST                (168*2)
#define EA_DP1_AREA                         (169*2)
#define EA_DP2_AREA                        (170*2)
#define EA_OUTPUT1_MAPPING						 (171*2)
#define EA_OUTPUT2_MAPPING						 (172*2)
#define EA_OUTPUT3_MAPPING						 (173*2)
#define EA_OUTPUT4_MAPPING						 (174*2)
#define EA_DP1_ZERO_RNG				     (175*2)
#define EA_DP2_ZERO_RNG			    	 (176*2)
#define EA_DP3_ZERO_RNG			    	 (177*2)
#define EA_TEMP_RH_SCAN_TIME                     (178*2)
#define EA_TEMP_RH_READING_AVERAGE               (179*2)
#define EA_PRES_READING_AVERAGE                  (180*2)
#define EA_DP1_SENS_TYPE                    (181*2)
#define EA_DP2_SENS_TYPE                   (182*2)
#define EA_DP3_SENS_TYPE                  (183*2)
#define EA_DP1_SENS_MIN                     (184*2)
#define EA_DP1_SENS_MAX                     (185*2)
#define EA_DP2_SENS_MIN                    (186*2)
#define EA_DP2_SENS_MAX                    (187*2)
#define EA_DP3_SENS_MIN                   (188*2)
#define EA_DP3_SENS_MAX                   (189*2)
#define EA_RTC_SETUP							 (190*2)
#define EA_DP3_AREA                              (191*2)
#define EA_DP1_AREA_TYPE						 (192*2)
#define EA_DP2_AREA_TYPE						 (193*2)
#define EA_DP3_AREA_TYPE						 (194*2)
#define EA_DP1_AREA_LENGTH						 (195*2)
#define EA_DP2_AREA_LENGTH						 (196*2)
#define EA_DP3_AREA_LENGTH						 (197*2)
#define EA_DP1_AREA_WIDTH						 (198*2)
#define EA_DP2_AREA_WIDTH						 (199*2)
#define EA_DP3_AREA_WIDTH						 (200*2)
#define EA_DP1_AREA_RADIOUS						 (201*2)
#define EA_DP2_AREA_RADIOUS						 (202*2)
#define EA_DP3_AREA_RADIOUS						 (203*2)
#define EA_AHU_ID								 (204*2)
#define EA_AHU_CFM								 (205*2)
#define EA_AHU_AREA1							 (206*2)
#define EA_AHU_AREA2							 (207*2)
#define EA_AHU_AREA3							 (208*2)

#define EA_TEMP_FIRE_ALM_SET					 (209*2)
#define EA_TEMP_FIRE_TIME_SET					 (210*2)

#define EA_IP1_HIGH_NAME						 (211*2)
#define EA_IP1_LOW_NAME							 (216*2)
#define EA_IP2_HIGH_NAME						 (221*2)
#define EA_IP2_LOW_NAME							 (226*2)
#define EA_IP3_HIGH_NAME						 (231*2)
#define EA_IP3_LOW_NAME							 (236*2)
#define EA_IP4_HIGH_NAME						 (241*2)
#define EA_IP4_LOW_NAME							 (246*2)
#define EA_OP1_HIGH_NAME						 (251*2)
#define EA_OP1_LOW_NAME							 (256*2)
#define EA_OP2_HIGH_NAME						 (261*2)
#define EA_OP2_LOW_NAME							 (265*2)
#define EA_OP3_HIGH_NAME						 (271*2)
#define EA_OP3_LOW_NAME							 (276*2)
#define EA_OP4_HIGH_NAME						 (281*2)
#define EA_OP4_LOW_NAME							 (286*2)
#define EA_TEMP2_ZERO_ADJ						 (291*2)
#define EA_TEMP_DIFF_ALARM_LIMIT				 (292*2)
#define EA_RH2_ZERO_ADJ							 (293*2)
#define EA_TEMP_RH2_SENS_TYPE                    (294*2)

#define EA_DEFAULT_SAVE							 (300*2)
#define EA_CONFIG_SYSTEM         		         (301*2)
#define EA_CUSTOMER_ID                           (302*2)
#define EA_SERIAL_NO                             (303*2)

#endif






































