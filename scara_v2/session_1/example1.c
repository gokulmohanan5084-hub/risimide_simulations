#include "halo.h"
#include <math.h>

#define printf sim_printf
extern int sim_printf(const char* fmt, ...);

#define PI 3.141592653589793
#define RAD_TO_DEG(rad) ((rad) * 180.0 / PI)

#define L1 10.0
#define L2 10.0

/* Convert angle to PWM duty cycle */
unsigned int angle_to_duty_us(unsigned int angle)
{
	if (angle > 180) angle = 180;
	return 40000 + (angle * 40000) / 180;
}

/* PWM Initialization */
void initialize_pwm(void)
{
	WRITE_REGISTER(0x40004000, 800000); // PWM1 Period
	WRITE_REGISTER(0x40004040, 800000); // PWM2 Period
	WRITE_REGISTER(0x40004008, 0x01);// PWM1 Enable
	WRITE_REGISTER(0x40004048, 0x01);// PWM2 Enable
	//printf("<t:i> <m:fw> SCARA PWM Registers Initialized\n");
}

/* Inverse Kinematics */
void inverse_kinematics(float x, float y, float* t1, float* t2)
{
	float d = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);

	if (d > 1.0) d = 1.0;
	if (d < -1.0) d = -1.0;

	float theta2 = acos(d);
	float theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

	*t1 = RAD_TO_DEG(theta1);
	*t2 = RAD_TO_DEG(theta2);
}

/* Move end-effector */
void move_to(float x, float y)
{
	float t1, t2;
	inverse_kinematics(x, y, &t1, &t2);

	if (t1 < 0) { t1 = 0; }
	if (t1 > 180) t1 = 180;
	if (t2 < 0) { t2 = 0; }
	if (t2 > 180) t2 = 180;

	WRITE_REGISTER(0x40004004, angle_to_duty_us((unsigned int)t1));
	WRITE_REGISTER(0x40004044, angle_to_duty_us((unsigned int)t2));

	delay_us(200);
}

/* Draw Circle */
void draw_circle(float cx, float cy, float radius, int points)
{
	for (int i = 0; i < points; i++)
	{
		float angle = i * 2.0 * PI / points;
		float x = cx + radius * cos(angle);
		float y = cy + radius * sin(angle);
		move_to(x, y);
	}
}

void fw_main(void)
{
	initialize_pwm();

	//stepper
	// bit0=ENABLE, bit1=DIR(1=forward/end, 0=reverse/home), bit2=STEP
	// GPIO_IN bit3=HOME (0x08), bit4=END (0x10)
	WRITE_REGISTER(0x40000000, 0x07); // ENABLE/DIR/STEP (bits 0-2) as outputs; HOME/END (bits 3-4) as inputs
	unsigned int gpio_out = 0x3;      // ENABLE=1, DIR=1 (forward), STEP=0
	WRITE_REGISTER(0x40000008, gpio_out);
	unsigned int reg_value = 0;
	for (int i = 0; i < 1000; i++)
	{
		// Generate step pulse
		WRITE_REGISTER(0x40000008, gpio_out | 0x4); // STEP=1
		delay_us(20);
		WRITE_REGISTER(0x40000008, gpio_out);        // STEP=0
		delay_us(200);

		reg_value = READ_REGISTER(0x40000004);
		printf("register value:0x%x\n", reg_value);

		if (reg_value & 0x10)        // END reached: reverse direction
		{
			gpio_out &= ~0x2;        // clear DIR bit (DIR=0, move toward home)
			WRITE_REGISTER(0x40000008, gpio_out);
		}
		else if (reg_value & 0x08)   // HOME reached: forward direction
		{
			gpio_out |= 0x2;         // set DIR bit (DIR=1, move toward end)
			WRITE_REGISTER(0x40000008, gpio_out);
			break;
		}
	}
	printf("<t:i> <m:fw> Executing SCARA PWM Circle Trajectory Test\n");

	while (1)
	{
		/* Draw circle centered at (-7.5, 12.5) with radius 3.0 */
		draw_circle(-7.5, 12.5, 3.0, 24);
	}
}
