#include "BuiltinKaiser.h"

// Kaiser filter coefficients for the kaiser filter in the decimate by 8 down converter
unsigned int ddc8_k5[] = {
		120, 130, 130, 110, 6, 262143, 262133, 262120, 262105, 262088,
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
		2483, 4967, 7622, 10231, 12565, 14411, 15594, 16002, 15594, 14411,
		12565, 10231, 7622, 4967, 2483, 354, 260857, 259771, 259246, 259227,
		259613, 260273, 261067, 261863, 408, 913, 1199, 1264, 1138, 875,
		537, 186, 262019, 261786, 261650, 261612, 261658, 261764, 261903, 262046,
		27, 119, 171, 185, 167, 129, 82, 36, 262143, 262117,
		262105, 262104, 262110, 262120, 262131
		};

/////////////////////////////////////////////////////////////////////////////
BuiltinKaiser::BuiltinKaiser() {
	init("ddc8_5_0", ddc8_k5, sizeof(ddc8_k5) / sizeof(unsigned int));
	init("ddc4_5_0", ddc4_k5, sizeof(ddc4_k5) / sizeof(unsigned int));
}

