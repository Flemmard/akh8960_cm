#ifndef __MAX8903B_CHARGER_H__
#define __MAX8903B_CHARGER_H__

#define MAX8903B_CONNECTED_PS_AC	(1U << 1)
#define MAX8903B_CONNECTED_PS_USB	(1U << 2)
#define MAX8903B_CONNECTED_PS_DOCK	(1U << 3)
#define MAX8903B_DOCK_DRAW_MA		1400

enum max8903b_current {
	CHARGE_DISABLE,
	CURRENT_ZERO,
	CURRENT_100MA,
	CURRENT_500MA,
	CURRENT_750MA,
	CURRENT_900MA,
	CURRENT_1000MA,
	CURRENT_1400MA,
	CURRENT_1500MA,
	CURRENT_2000MA,
};


struct max8903b_platform_data {
	int DCM_in;
	int DCM_in_polarity;
	int IUSB_in;
	int IUSB_in_polarity;
	int USUS_in;
	int USUS_in_polarity;
	int CEN_N_in;
	int CEN_N_in_polarity;
	int DOK_N_out;
	int CHG_N_out;
	int FLT_N_out;
	int (*set_DC_CHG_Mode_current)(enum max8903b_current value);
	int  (*request_release_gpios)(int request);
	void (*suspend_gpio_config)(void);
};

void max8903b_set_charge_ma (unsigned ma);
void max8903b_disable_charge (void);

#endif /* !__MAX8903B_CHARGER_H__ */

