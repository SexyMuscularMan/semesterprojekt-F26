
PROGRAM _INIT
	thetaSmoothingConst := 0.25;
	xSmoothingConst := 0.25;
	uSmoothingConst := 0.25;
	i := 0;
	
	(*FOR i := 0 TO 2 DO
		eOut[i] := 0;
		eIn[i] := 0;
		uOut[i] := 0;
		uIn[i] := 0;
	END_FOR*)
	
	
END_PROGRAM

PROGRAM _CYCLIC
	pos_cart := I_pos_cart;
	I_pos_pendulum_rad := ((I_pos_pendulum - 2048) / 4096.0) * 2 * Pi;
	pos_pendulum_absolute := I_pos_pendulum MOD 4096;
	IF(pos_pendulum_absolute < 0) THEN
		pos_pendulum_absolute := pos_pendulum_absolute + 4096;
	END_IF
	pos_pendulum_deg := (pos_pendulum_absolute - 2048) / 4096.0 * 360.0;
	pos_pendulum_rad := (pos_pendulum_absolute - 2048) / 4096.0 * 2 * Pi;
	pos_cart_mm := pos_cart / 240;
	length_mm := length / 240;
	cart_velocity_calculated := (pos_cart_mm - cart_position_last) / 0.01;
	pendulum_velocity_rad := (I_pos_pendulum_rad - pendulum_position_rad_last) / 0.01;
	
	xLocal[0] := - pos_cart_mm / 1000.0 + length_mm / 2000.0;
	xLocal[1] := - cart_velocity_calculated / 1000.0;
	xLocal[2] := pos_pendulum_rad;
	xLocal[3] := pendulum_velocity_rad;
	
	
	IF(NOT stopped) THEN
		eOut[2] := eOut[1];
		eOut[1] := eOut[0];
		eIn[2] := eIn[1];
		eIn[1] := eIn[0];
		uOut[2] := uOut[1];
		uOut[1] := uOut[0];
		uIn[2] := uIn[1];
		uIn[1] := uIn[0];
		
		(*
		eOut[0] := - x[0];
		uOut[0] := 1.924 * uOut[1] - 0.9235 * uOut[2] + 1.038 * eOut[0] - 2.073 * eOut[1] + 1.035 * eOut[2];
		
		IF(uOut[0] > 0.2) THEN
			uOut[0] := 0.2;
		END_IF
		IF(uOut[0] < - 0.2) THEN
			uOut[0] := - 0.2;
		END_IF
		
		
		
		theta_smoothed := thetaSmoothingConst * theta_smoothed + (1 - thetaSmoothingConst) * x[2];
		
		eIn[0] := uOut[0] - x[2];
		uIn[0] := 0.0515 * uIn[1] + 0.9485 * uIn[2] - 84.6 * eIn[0] + 150.9 * eIn[1] - 66.51 * eIn[2];*)
		
		eOut[0] :=  x[0];
		d1 := (eOut[0] - eOut[1]) / 0.01;
		uOut[0] := K_Out[0] * eOut[0] + K_Out[1] * (eOut[0] * 0.01) + K_Out[2] * d1;
		//uOut[0] := LIMIT(-0.2, uOut[0], 0.2);
		
		//eIn[0] := uOut[0] - x[2];
		eIn[0] := -x[2];
		d2 := (eIn[0] - eIn[1]) / 0.01;
		uIn[0] := K_In[0] * eIn[0] + K_In[1] * (eIn[0] * 0.01) + K_In[2] * d2;
		//uIn[0] := LIMIT(-1, uIn[0], 1);
		
		uLocal := 0;
		FOR i:= 0 TO 3 BY 1 DO
			uLocal := uLocal - K[i] * x[i];
		END_FOR
		
		
		IF(ABS(u) < 1) THEN
			balancing_active := TRUE;
		ELSIF(ABS(u) >= 10) THEN
			balancing_active := FALSE;
			(* FOR i := 0 TO 2 DO
				eOut[i] := 0;
				eIn[i] := 0;
				uOut[i] := 0;
				uIn[i] := 0;
			END_FOR *)
		END_IF
		
		(*IF(balancing_active) THEN
			new_cart_velocity := REAL_TO_INT( - 2000 * u - 2000 * (springforce * x[0]));
			IF(ABS(new_cart_velocity) < 20000) THEN
				Q_vel_cart := new_cart_velocity;
			ELSE
				IF(new_cart_velocity < 0) THEN
					Q_vel_cart := -20000;
				ELSE
					Q_vel_cart := 20000;
				END_IF
			END_IF
		ELSE
			Q_vel_cart := 0;
		END_IF*)
		
		
		(*IF(pos_cart_mm < 50 OR length_mm - pos_cart_mm < 50) THEN
			stopped := TRUE;
			Q_vel_cart := 0;
		END_IF
		IF(NOT I_endstop_positive OR NOT I_endstop_negative) THEN
			stopped := TRUE;
			Q_vel_cart := 0;
		END_IF*)
		
		
		
	END_IF
	
	FOR i := 0 TO 3 DO
		xGlobalDiff[i] := xGlobal[i] - xLocal[i];
		xGlobalAvgedDiff[i] := xGlobalAvged[i] - xLocal[i];
		IF(xGlobalDiff[i] < 0.01) THEN
			xGlobalDiff[i] := 0;
		END_IF
		IF(xGlobalAvgedDiff[i] < 0.01) THEN
			xGlobalAvgedDiff[i] := 0;
		END_IF
	END_FOR
	
	
	uLocalLast := uLocal;
	thetaLocalLast := xLocal[2];
	cart_position_last := pos_cart_mm;
	pendulum_position_rad_last := I_pos_pendulum_rad;
	
	
END_PROGRAM

PROGRAM _EXIT
	 
END_PROGRAM