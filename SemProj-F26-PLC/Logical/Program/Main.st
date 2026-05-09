
PROGRAM _INIT
	(* Insert code here *)
	cart_velocity := 5000;
	pos_cart := - I_pos_cart;
	Q_vel_cart := 0;
	length := 0;
	SI_reset_last := SI_reset_btn;
	allow_Q_drive_enable := FALSE;
	Q_drive_enable  := FALSE;
	length_calibration_steps := 5;
	calibration_finished := FALSE;
	centering_in_progress := FALSE;
	Temp_checker := 0;
	stop_acc := 400;
	stopped := FALSE;
	balancing_active := FALSE;
	Swing_movement := position0;
	i := 1;
END_PROGRAM

PROGRAM _CYCLIC
	(* Insert code here *)
	
	// Handle EStop
	IF(SI_estop_A AND SI_estop_B) THEN
		Q_drive_enable := TRUE;
	ELSE 
		Q_drive_enable  := FALSE;
		Q_reset_LED := FALSE;
	END_IF
	
	// Handle Reset Btn
	IF(SI_reset_btn) THEN
		Q_drive_faultreset := NOT Q_drive_faultreset;
		IF(NOT SI_reset_last) THEN
			allow_Q_drive_enable := NOT allow_Q_drive_enable;
		END_IF
	ELSIF I_drive_no_fault THEN
		Q_reset_LED := TRUE;
	ELSE
		led_ton(IN:=NOT led_ton.Q, PT := T#200ms);
		IF(led_ton.Q) THEN
			Q_reset_LED := NOT Q_reset_LED;
			Q_reset_LED := NOT Q_reset_LED;
		END_IF
	END_IF
		
	
	
	
	
	pos_pendulum_absolute := I_pos_pendulum;
	WHILE(pos_pendulum_absolute > 4096 OR pos_pendulum_absolute < 0) DO
		IF(pos_pendulum_absolute < 0) THEN
			pos_pendulum_absolute := pos_pendulum_absolute + 4096;
		ELSIF(pos_pendulum_absolute > 4096) THEN
			pos_pendulum_absolute := pos_pendulum_absolute - 4096;
		END_IF
	END_WHILE
	pos_pendulum_deg := (pos_pendulum_absolute - 2048) / 4096.0 * 360.0;
	pos_pendulum_rad := (pos_pendulum_absolute - 2048) / 4096.0 * 2 * Pi;
	pos_cart_mm := pos_cart / 264;
	length_mm := length / 264;
	
	
	cart_velocity_calculated := (pos_cart_mm - cart_position_last) / 0.01;
	
	pend_vel_a := (pendulum_position_rad_last - pos_pendulum_rad) / 0.01;
	pend_vel_b := (pos_pendulum_rad - pendulum_position_rad_last) / 0.01;
	
	IF(pend_vel_a * pend_vel_a < pend_vel_b * pend_vel_b) THEN
		pendulum_velocity_rad := pend_vel_a;
	ELSE
		pendulum_velocity_rad := pend_vel_b;
	END_IF
	
	x[0] := - pos_cart_mm / 1000.0 + length_mm / 2 / 1000.0;
	x[1] := - cart_velocity_calculated / 1000.0;
	x[2] := pos_pendulum_rad;
	x[3] := pendulum_velocity_rad;
	
	
	
	
	IF(length_calibration_steps = 5) THEN
		reset_pendulum_position := TRUE;
		length_calibration_steps := 4;
		reset_cart_position := TRUE;
		reset_cart_position := FALSE;
	ELSIF(length_calibration_steps = 4) THEN
		reset_pendulum_position := FALSE;
		Q_vel_cart := -cart_velocity;
		Q_drive_faultreset := NOT Q_drive_faultreset;
		IF (NOT I_endstop_negative) THEN
			Q_vel_cart := 0;
			reset_cart_position := TRUE;
			length_calibration_steps := 3;
		END_IF
	ELSIF(length_calibration_steps = 3) THEN
		reset_cart_position := FALSE;
		Q_vel_cart := cart_velocity;
		Q_drive_faultreset := NOT Q_drive_faultreset;
		length_calibration_steps := 2;
	ELSIF(length_calibration_steps = 2) THEN
		Q_drive_faultreset := NOT Q_drive_faultreset;
		IF (NOT I_endstop_positive) THEN 
			Q_vel_cart := 0;
			length := MAX(length, I_pos_cart);
			length_calibration_steps := 1;
		END_IF
	ELSIF(length_calibration_steps = 1) THEN
		Q_vel_cart := -cart_velocity;
		Q_drive_faultreset := NOT Q_drive_faultreset;
		length_calibration_steps := 0;
		Swing_movement := position1;
	END_IF
	
	
	IF(swingup_complete) THEN
		Swing_movement := position10;
	END_IF
		
	CASE Swing_movement OF
		position0:
						
		position1:
			target_position :=  length / 2 + 20000;
			IF(ABS(pos_cart - target_position) < stop_acc) THEN
				Q_vel_cart := 0;
				Swing_movement := position2;
			ELSIF(pos_cart < target_position) THEN
				Q_vel_cart := REAL_TO_INT(cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			ELSIF(pos_cart > target_position) THEN
				Q_vel_cart := REAL_TO_INT(-cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			END_IF
			
		position2:
			target_position :=  length / 2 + 40000;
			IF(ABS(pos_cart - target_position) < stop_acc) THEN
				Q_vel_cart := 0;
				Swing_movement := position10;
			ELSIF(pos_cart < target_position) THEN
				Q_vel_cart := REAL_TO_INT(cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			ELSIF(pos_cart > target_position) THEN
				Q_vel_cart := REAL_TO_INT(-cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			END_IF

		position3:
			target_position :=  length / 2;
			IF(ABS(pos_cart - target_position) < stop_acc) THEN
				Q_vel_cart := 0;
				Swing_movement := position10;
			ELSIF(pos_cart < target_position) THEN
				Q_vel_cart := REAL_TO_INT(cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			ELSIF(pos_cart > target_position) THEN
				Q_vel_cart := REAL_TO_INT(-cart_velocity * 1.0);
				Q_drive_faultreset := NOT Q_drive_faultreset;
			END_IF			
			
		position10:
			swingup_complete := TRUE;

		END_CASE
	
	IF(swingup_complete AND NOT stopped) THEN
		u := 0;
		FOR i:= 0 TO 3 BY 1 DO
			u := u - K[i] * x[i];
		END_FOR
		
		IF(ABS(u) < 2) THEN
			balancing_active := TRUE;
		ELSIF(ABS(u) > 20) THEN
			balancing_active := FALSE;
		END_IF
		
		IF(ABS(u) < 20 AND balancing_active) THEN
			new_cart_velocity := REAL_TO_INT(- 10000 * u);
			IF(ABS(new_cart_velocity) < 10000) THEN
				Q_vel_cart := new_cart_velocity;
			ELSE
				IF(new_cart_velocity < 0) THEN
					Q_vel_cart := -10000;
				ELSE
					Q_vel_cart := 10000;
				END_IF
			END_IF
		ELSE
			Q_vel_cart := 0;
		END_IF
		
		
		IF(pos_cart_mm < 50 OR length_mm - pos_cart_mm < 50) THEN
			stopped := TRUE;
			Q_vel_cart := 0;
		END_IF
		IF(NOT I_endstop_positive OR NOT I_endstop_negative) THEN
			stopped := TRUE;
			Q_vel_cart := 0;
		END_IF
		
		
		
	END_IF
	
	
	
	
	
	
	
	
	
	SI_reset_last := SI_reset_btn;
	cart_position_last := pos_cart_mm;
	pendulum_position_rad_last := pos_pendulum_rad;
	
	
END_PROGRAM

PROGRAM _EXIT
	(* Insert code here *)
	 
END_PROGRAM