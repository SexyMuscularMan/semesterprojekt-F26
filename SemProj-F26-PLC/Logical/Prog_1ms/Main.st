
PROGRAM _INIT
	Q_vel_cart := 0;
	SystemState := Calibrating;
	CalibrationState := Start_Calibration;
	SwingupState := Start_Swingup;
	BalancingState := Cascade10ms;
	swingupOnReset := FALSE;
	smoothingConst := 0.99;
	iterationCounter := 0;
	i := 1;
END_PROGRAM

PROGRAM _CYCLIC
	
	IF(SI_estop_A AND SI_estop_B) THEN
		Q_drive_enable := TRUE;
	ELSE 
		Q_drive_enable  := FALSE;
		Q_reset_LED := FALSE;
	END_IF
	
	IF(SI_reset_btn) THEN
		Q_drive_faultreset := NOT Q_drive_faultreset;
	ELSIF I_drive_no_fault THEN
		Q_reset_LED := TRUE;
	ELSE
		led_ton(IN:=NOT led_ton.Q, PT := T#200ms);
		IF(led_ton.Q) THEN
			Q_reset_LED := NOT Q_reset_LED;
			Q_reset_LED := NOT Q_reset_LED;
		END_IF
	END_IF
	
	
	iterationCounter := iterationCounter + 1;
	lengthMm := length / 240;
	cartPos := I_pos_cart;
	cartPosMm := cartPos / 240;
	cartVelCalculated := (cartPos - cartPosLast) * 1000.0 / 240;
	pendulumPosAbsolute := I_pos_pendulum MOD 4096;
	IF(pendulumPosAbsolute < 0) THEN
		pendulumPosAbsolute := pendulumPosAbsolute + 4096;
	END_IF
	pendulumPosRad := (pendulumPosAbsolute - 2048) / 4096.0 * 2 * Pi;
	pendulumVel := (I_pos_pendulum - pendulumPosLast[9]) / 4096.0 * 1000.0 / 10;
	pendulumVelRad := pendulumVel * 2 * Pi;
	
	
	data[0] := (cartPosMm - lengthMm / 2) / 1000.0;
	data[1] := cartVelCalculated / 1000.0;
	data[2] := pendulumPosRad;
	data[3] := pendulumVelRad;
	
	dataSmoothed[0] := data[0];
	dataSmoothed[1] := smoothingConst * dataSmoothed[1] + (1 - smoothingConst) * data[1];
	dataSmoothed[2] := data[2];
	dataSmoothed[3] := smoothingConst * dataSmoothed[3] + (1 - smoothingConst) * data[3];
	
	FOR i := 0 TO 8 DO
		FOR j := 0 TO 3 DO
			dataBacklog[9 - i, j] := dataBacklog[8 - i, j];
		END_FOR
	END_FOR
	FOR j := 0 TO 3 DO
		dataBacklog[0, j] := data[j];
	END_FOR
	FOR i := 0 TO 3 DO
		data10ms[i] := 0;
		FOR j := 0 TO 9 DO
			data10ms[i] := data10ms[i] + dataBacklog[j, i];
		END_FOR
		data10ms[i] := data10ms[i] / 10;
	END_FOR
	
	FOR i := 0 TO 3 DO
		xGlobal[i] := data[i];
	END_FOR
	
	
	IF(SystemState <> Calibrating) THEN
		IF(data[3] > 20 OR cartPosMm < 100 OR lengthMm - cartPosMm < 100) THEN
			Q_vel_cart := 0;
			SystemState := Reset;
		ELSIF(NOT I_endstop_negative OR NOT I_endstop_positive) THEN
			Q_vel_cart := 0;
			SystemState := Stopped;
		END_IF
	END_IF
	
	
	CASE SystemState OF
		Calibrating: 
			IF(iterationCounter MOD 2 = 0) THEN
				CASE CalibrationState OF
					Start_Calibration: 
						reset_pendulum_position := TRUE;
						CalibrationState := Move_Left;
						
					Move_Left:
						reset_pendulum_position := FALSE;
						Q_vel_cart := -cartVelocity;
						Q_drive_faultreset := NOT Q_drive_faultreset;
						IF (NOT I_endstop_negative) THEN
							Q_vel_cart := 0;
							reset_cart_position := TRUE;
							CalibrationState := Reset_Position;
						END_IF
						
					Reset_Position:
						reset_cart_position := FALSE;
						Q_vel_cart := cartVelocity;
						Q_drive_faultreset := NOT Q_drive_faultreset;
						CalibrationState := Move_Right;
						
					Move_Right:
						Q_drive_faultreset := NOT Q_drive_faultreset;
						IF (NOT I_endstop_positive) THEN 
							Q_vel_cart := 0;
							length := ABS(cartPos);
							CalibrationState := Calibration_Center;
						END_IF
						
					Calibration_Center:
						Q_vel_cart := -cartVelocity;
						Q_drive_faultreset := NOT Q_drive_faultreset;
						IF(ABS(cartPos - length / 2) < stopAcc) THEN
							Q_vel_cart := 0;
							CalibrationState := Calibration_Complete;
						ELSE
							Q_vel_cart := REAL_TO_INT( - data[0] / ABS(data[0]) * cartVelocity);
						END_IF
												
					Calibration_Complete:
						SwingupState := Swingup_Center;
						SystemState := Swingup;
						
					ELSE
						calibrationStateFailure := TRUE;
					
				END_CASE
			END_IF
			
			
		Swingup:
			CASE SwingupState OF
				Swingup_Center:
					IF(ABS(cartPos - length / 2) < stopAcc) THEN
						Q_vel_cart := 0;
						SwingupState := Start_Swingup;
					ELSE
						Q_vel_cart := REAL_TO_INT( - data[0] / ABS(data[0]) * cartVelocity);
					END_IF
					
				Start_Swingup:
					Q_vel_cart := 5000;
					IF(ABS(data[0]) > 0.2) THEN
						SwingupState := Accelerate;
					END_IF
					
				Accelerate:
					IF(data[2] > Pi / 2 AND data[3] > 0 AND data[0] < 0.5) THEN
						Q_vel_cart := 5000;
					ELSIF(data[2] < - Pi / 2 AND data[3] < 0 AND data[0] > - 0.5) THEN
						Q_vel_cart := - 5000;
					ELSE
						Q_vel_cart := 0;
					END_IF
					
					IF(ABS(data[2]) < 0.5 AND ABS(data[3]) > 1) THEN
						Q_vel_cart := 0;
						SwingupState := Swingup_Complete;
						SystemState := Balancing;
					END_IF
					
				Swingup_Complete:
					SystemState := Balancing;
					
				ELSE
					swingupStateFailure := TRUE;
					
			END_CASE
			
			
		Balancing:
			u := 0;
			CASE BalancingState OF
				PolePlacement10ms: 
					FOR i:= 0 TO 1 DO
						u := u + pp10msK[i] * data10ms[i];
					END_FOR
					FOR i:= 2 TO 3 DO
						u := u - pp10msK[i] * data10ms[i];
					END_FOR
					newCartVelocity := REAL_TO_INT(- 10000 * u);
					
					IF(ABS(u) < 1 AND data[3] < 5 AND ABS(data[2]) < 1) THEN
						balancingActive := TRUE;
					ELSIF(ABS(u) >= 10 OR data[3] > 15) THEN
						balancingActive := FALSE;
					END_IF
					
				PolePlacement1ms:
					FOR i:= 0 TO 3 DO
						u := u - pp1msKTemp[i] * dataSmoothed[i];
					END_FOR
					newCartVelocity := REAL_TO_INT(1600 * u);
					
					IF(ABS(u) < 15 AND data[3] < 10 AND ABS(data[2]) < 1) THEN
						balancingActive := TRUE;
					ELSIF(ABS(u) >= 50 OR data[3] > 15) THEN
						balancingActive := FALSE;
					END_IF
					
				Cascade10ms:
					pidOutError := - data[0];		
					pidOutIntegration := pidOutIntegration + pidOutError * 0.001;
					pidOutDerivative := dataSmoothed[1];
					uOut := cpid10msK[0, 0] * pidOutError + cpid10msK[0, 1] * pidOutIntegration + cpid10msK[0, 2] * pidOutDerivative;
					
					IF(ABS(uOut) > 0.2) THEN
						uOut := 0.2 * uOut / ABS(uOut);
					END_IF
					
					pidInError := uOut - data[2];		
					pidInIntegration := pidInIntegration + pidInError * 0.001;
					pidInDerivative := dataSmoothed[3];
					uIn := cpid10msK[1, 0] * pidInError + cpid10msK[1, 1] * pidInIntegration + cpid10msK[1, 2] * pidInDerivative;
					
					IF(ABS(uIn) > 1.0) THEN
						uIn := 1.0 * uIn / ABS(uIn);
					END_IF
					u := uIn;
					newCartVelocity := REAL_TO_INT(- 2000 * u);
					
					IF(ABS(u) < 1 AND data[3] < 5 AND ABS(data[2]) < 1) THEN
						balancingActive := TRUE;
					ELSIF(ABS(u) >= 10 OR data[3] > 15) THEN
						balancingActive := FALSE;
					END_IF
					
				ELSE
					balancingStateFailure := TRUE;
					
			END_CASE
			
			IF(balancingActive) THEN
				IF(ABS(newCartVelocity) < 20000) THEN
					Q_vel_cart := newCartVelocity;
				ELSE
					Q_vel_cart := 20000 * newCartVelocity / ABS(newCartVelocity);
				END_IF
			ELSE
				Q_vel_cart := 0;
			END_IF
			
					
		Reset:
			IF(swingupOnReset) THEN
				SwingupState := Swingup_Center;
				SystemState := Swingup;
			END_IF
			
		Stopped:
			IF(SI_reset_btn) THEN
				CalibrationState := Start_Calibration;
				SystemState := Calibrating;
			END_IF
			
		ELSE
			systemStateFailure := TRUE;
		
	END_CASE
	
	
	cartPosLast := cartPos;
	FOR i := 0 TO 8 DO
		pendulumPosLast[9 - i] := pendulumPosLast[8 - i];
	END_FOR
	pendulumPosLast[0] := I_pos_pendulum;
		
END_PROGRAM

PROGRAM _EXIT
	(* Insert code here *)
	 
END_PROGRAM