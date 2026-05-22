
TYPE
	Swingup_State : 
		(
		Swingup_Center,
		Accelerate,
		Swingup_Complete,
		Start_Swingup
		);
	System_State : 
		(
		Stopped,
		Balancing,
		Reset,
		Swingup,
		Calibrating
		);
	Calibration_State : 
		(
		Calibration_Complete,
		Calibration_Center,
		Move_Right,
		Reset_Position,
		Move_Left,
		Start_Calibration
		);
	Balancing_State : 
		(
		Classic1ms,
		Classic10ms,
		Cascade1ms,
		Cascade10ms,
		PolePlacement1ms,
		PolePlacement10ms
		);
END_TYPE
