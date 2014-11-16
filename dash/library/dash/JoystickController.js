Dash.JoystickController = function(robot) {
	this.robot = robot;
	this.gamepad = null;
	this.useGamepad = false;
	this.enabled = false;
	this.fastMode = false;
	this.gamepad = new Gamepad();
};

Dash.JoystickController.prototype.init = function() {
	var self = this;

	this.gamepad.bind(Gamepad.Event.TICK, function(gamepads) {
		self.onTick(gamepads);
	});

	this.gamepad.bind(Gamepad.Event.BUTTON_DOWN, function(e) {
		self.onButtonDown(e);
	});

	this.gamepad.bind(Gamepad.Event.AXIS_CHANGED, function(e) {
		self.onAxisChanged(e);
	});

	$('#gamepad').html('Your browser supports gamepads, try connecting one');

	if (!this.gamepad.init()) {
		$('#gamepad').html('If you use latest Google Chrome or Firefox, you can use gamepads..');
	}
};

Dash.JoystickController.prototype.onButtonDown = function(e) {
	if (e.control == 'RIGHT_TOP_SHOULDER') {
		this.robot.kick();
	} else if (e.control == 'FACE_4') {
		this.fastMode = !this.fastMode;
	} else if (e.control == 'FACE_3') {
		dash.ui.toggleTargetSide();
		window.setTimeout(function() { dash.ui.toggleTargetSide(); }, 200);
		window.setTimeout(function() { dash.socket.send('<run-find-ball>'); }, 200);
	} else if (e.control == 'FACE_2') {
		dash.socket.send('<run-manual-control>');
	} else if (e.control == 'LEFT_TOP_SHOULDER') {
		this.robot.toggleDribbler();
	} else {
		console.log('unmapped button pressed: ' + e.control);
	}
};

Dash.JoystickController.prototype.onAxisChanged = function(e) {
	if (e.axis == 'LEFT_TRIGGER') {
		this.robot.setDribbler(e.value * 255);
	}
};

Dash.JoystickController.prototype.onTick = function(gamepads) {
	if (!this.enabled && gamepads[0].state.X === 1) {
		this.enabled = true;

		this.onSelfEnable();
	}

	if (!this.enabled) {
		return;
	}
	
	if (
		gamepads[0].state.RIGHT_STICK_X != 0
		|| gamepads[0].state.RIGHT_STICK_Y != 0
		|| gamepads[0].state.LEFT_STICK_X != 0
		|| gamepads[0].state.LEFT_STICK_Y != 0
	) {
		this.useGamepad = true;
	}

	if (!this.useGamepad) {
		return;
	}

	if (gamepads[0].state.DPAD_UP != 0) {
		this.robot.adjustDribbler(1, 0);
	} else if (gamepads[0].state.DPAD_DOWN != 0) {
		this.robot.adjustDribbler(-1, 0);
	}

	if (gamepads[0].state.DPAD_LEFT != 0) {
		this.robot.adjustDribbler(0, -1);
	} else if (gamepads[0].state.DPAD_RIGHT != 0) {
		this.robot.adjustDribbler(0, 1);
	}
	
	var speed = dash.config.joystick.speed,
		turnRate = dash.config.joystick.turnRate,
		currentTime = Dash.Util.getMicrotime();
		
	if (this.fastMode) {
		speed *= 2;
		turnRate *= 2;
	}

	this.robot.setTargetDir(
		gamepads[0].state.RIGHT_STICK_Y * -speed,
		gamepads[0].state.RIGHT_STICK_X * speed,
		gamepads[0].state.LEFT_STICK_X * turnRate
	);
};

Dash.JoystickController.prototype.onSelfEnable = function() {};