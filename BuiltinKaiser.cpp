#include "BuiltinKaiser.h"

// Kaiser filter coefficients for the kaiser filter in the decimate by 8 down converter
unsigned int ddc8_k5[] = {
		12, 13, 13, 11, 6, 262143, 262133, 262120, 262105, 262088,
		262071, 262053, 262038, 262027, 262021, 262022, 262032, 262052, 262083, 262125,
		33, 94, 161, 233, 303, 369, 425, 466, 486, 481,
		447, 382, 282, 150, 262132, 261943, 261735, 261517, 261298, 261092,
		260911, 260770, 260683, 260662, 260721, 260869, 261113, 261457, 261903, 302,
		936, 1649, 2426, 3248, 4094, 4941, 5764, 6540, 7244, 7854,
		8350, 8717, 8942, 9018
		};

// Filter coefficients for the kaiser filter in the decimate by 4 down converter
unsigned int ddc4_k5[] = {
		262131, 262120, 262110, 262104, 262105, 262117, 262143, 36, 82, 129,
		167, 185, 171, 119, 27, 262046, 261903, 261764, 261658, 261612,
		261650, 261786, 262019, 186, 537, 875, 1138, 1264, 1199, 913,
		408, 261863, 261067, 260273, 259613, 259227, 259246, 259771, 260857, 354,
		2483, 4967, 7622, 10231, 12565, 14411, 15594, 16002
		};

/////////////////////////////////////////////////////////////////////////////
BuiltinKaiser::BuiltinKaiser() {
	init("ddc8_5_0", ddc8_k5, sizeof(ddc8_k5) / sizeof(unsigned int));
	init("ddc4_4_0", ddc4_k5, sizeof(ddc4_k5) / sizeof(unsigned int));
}

