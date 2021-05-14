typedef struct Robot {
    float x;
    float y;
    float ori;

    int wait_mode;
    int power;
    float calibrate_deg;
    int bump_port;
} Robot_t;

void robot_init(Robot_t *r) {
    r->x = 0;
    r->y = 0;
    r->ori = 90;
    r->wait_mode = 1;
    r->power = 23;
    r->calibrate_deg = FLOOR_COFF;
    r->bump_port = port8;
}

void robot_set_wait_mode(Robot_t* r, int _wait_mode) {
    r->wait_mode = _wait_mode;
}

void robot_set_power(Robot_t *r, int power) {
    r->power = power;
}

void robot_move_degrees(Robot_t* r, float degree) {
    if (is_equal(degree, 0)) {
        return;
    }

	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);

    if (degree > 0) {
        moveMotorTarget(LeftMotor, degree, r->power);
        moveMotorTarget(RightMotor, degree, r->power);

        float distance = degree / 360 * WHEEL_CIRCUMFERENCE;

        r->x += distance * cos(deg_to_rad(r->ori));
        r->y += distance * sin(deg_to_rad(r->ori));
    } else {
        moveMotorTarget(LeftMotor, degree, -r->power);
        moveMotorTarget(RightMotor, degree, -r->power);

        float distance = -degree / 360 * WHEEL_CIRCUMFERENCE;

        r->x += distance * cos(deg_to_rad(180 + r->ori));
        r->y += distance * sin(deg_to_rad(180 + r->ori));
    }

    if (r->wait_mode) {
        waitUntilMotorStop(LeftMotor);
        waitUntilMotorStop(RightMotor);
    }
}

void robot_move_rotations(Robot_t* r, float rotations) {
    robot_move_degrees(r, rotations * 360);
}

void robot_move_inches(Robot_t* r, float inches) {
    float degs = 360 / WHEEL_CIRCUMFERENCE * inches;
    robot_move_degrees(r, degs);
}

//self explanitory
void robot_point_turn(Robot_t* r, float degree) {
   //degree = uniform_deg(degree);
	float rotation_degrees = 4 * degree * (1 + r->calibrate_deg) / WHEEL_RADIUS;

	screen_log_float(r->ori);
	screen_show();

	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);

	moveMotorTarget(LeftMotor, -rotation_degrees, r->power);
	moveMotorTarget(RightMotor, rotation_degrees, r->power);

    r->ori = uniform_deg(r->ori + degree);

    if (r->wait_mode) {
        waitUntilMotorStop(LeftMotor);
        waitUntilMotorStop(RightMotor);
    }
}

//point turn to an orientation relative to where the robot started
void robot_point_turn_to(Robot_t *r, float ori) {
    float diff_deg = uniform_deg(ori - r->ori);
    robot_point_turn(r, diff_deg);
}

//function returns the distance robot has traveled before touch sensor is triggered.
//distance is in rotations of the wheel
//max_distance is in rotations of the wheel
float robot_move_until_touch(Robot_t *r, int ifForward) {
    float max_distance = 10000000;

	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);

  	moveMotorTarget(LeftMotor, max_distance * ((ifForward < 0) ? -1 : 1), r->power);
  	moveMotorTarget(RightMotor, max_distance * ((ifForward < 0) ? -1 : 1), r->power);

	while (!getBumperValue(r->bump_port) && getMotorEncoder(leftMotor) <= max_distance) {
			sleep(100);
	}

	float distance_degrees = getMotorEncoder(LeftMotor);
    float distance = distance_degrees / 360 * WHEEL_CIRCUMFERENCE;

    r->x += distance * cos(deg_to_rad(r->ori + (ifForward ? 0 : 180)));
    r->y += distance * sin(deg_to_rad(r->ori + (ifForward ? 0 : 180)));

 	resetMotorEncoder(LeftMotor);
	resetMotorEncoder(RightMotor);

	return distance_degrees / 360;
}

void robot_move_to_location(Robot_t *r, float x, float y) {
    float dx = x - r->x;
    float dy = y - r->y;
    float deg;

    if (is_equal(dx, 0) && is_equal(dy, 0))
        return;

    if (is_equal(dx, 0)) {
        deg = (dy >= 0) ? 90 : -90;
    } else if (is_equal(dy, 0)) {
        deg = (dx >= 0) ? 0 : -180;
    } else {
        if (dx > 0 && dy > 0) {
            deg = rad_to_deg(atan(dy / dx));
        } else if (dx < 0 && dy > 0) {
            deg = 180 - rad_to_deg(atan(dy / -dx));
        } else if (dx < 0 && dy < 0) {
            deg = 180 + rad_to_deg(atan(dy / dx));
        } else {
            deg = -rad_to_deg(atan(-dy / dx));
        }
    }

    robot_point_turn_to(r, deg);

    float dist = sqrt(dx * dx + dy * dy);
    robot_move_inches(r, dist);
}