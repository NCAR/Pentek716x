#include "BuiltinGaussian.h"

// Gaussian filter coefficients for the decimate by 8 down converter
unsigned int ddc8_0_2[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 3, 43, 395, 2316,
		8730, 21144, 32904 };

unsigned int ddc8_0_3[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 3, 19, 92, 365, 1186, 3168, 6955,
		12543, 18584, 22620 };

unsigned int ddc8_0_5[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 1, 3, 12, 39, 118, 320, 776, 1682, 3266, 5676, 8833,
		12308, 15354, 17149 };
unsigned int ddc8_0_6[] = { 0, 0, 0, 0, 0, 0, 0, 1, 3,
8, 22, 55, 129, 281, 571, 1079, 1901, 3120, 4770, 6795, 9018,
11150, 12845, 13787 };

unsigned int ddc8_0_7[] = { 0, 0, 0, 0, 1, 3, 6, 14, 32,
		66, 132, 249, 450, 772, 1262, 1964, 2910, 4105, 5513, 7048, 8579,
		9941, 10968, 11521 };

unsigned int ddc8_0_8[] = { 0, 1, 2, 5, 10, 21, 39, 73, 130,
		223, 370, 592, 913, 1358, 1948, 2696, 3599, 4634, 5755, 6894, 7965,
		8876, 9540, 9891 };

unsigned int ddc8_1_0[] = { 4, 8, 15, 26, 45, 77, 126, 202, 314,
		476, 700, 1003, 1398, 1894, 2498, 3203, 3996, 4849, 5723, 6572, 7340,
		7975, 8428, 8664 };

// Gaussian filter coefficients for the decimate by 4 down converter
unsigned int ddc4_1_0[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
		3, 6, 14, 32, 66, 132, 249, 450, 772, 1262, 1964, 2910, 4105, 5513,
		7048, 8579, 9941, 10968, 11521};
        
// Gaussian filter coefficients for the decimate by 10 down converter
unsigned int ddc10_0_5[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
        0, 0, 0, 0, 0, 0, 1, 10, 70, 385, 1584, 4914, 11487, 20233, 26853
        };

// Gaussian filter coefficients for the decimate by 10 down converter -- flat passband
unsigned int ddc10_0_5_flat[] = {262125, 262080, 262044, 262042, 262095, 61, 198, 301, 299,
                                 139, 261975, 261610, 261349, 261374, 261793, 418, 1301, 1907,
                                 1828, 827, 261161, 259077, 257593, 257677, 260039, 2667, 9240,
                                 16338, 22348, 25793
};
/////////////////////////////////////////////////////////////////////////////
BuiltinGaussian::BuiltinGaussian() {
	init("ddc8_0_2", ddc8_0_2, sizeof(ddc8_0_2) / sizeof(unsigned int));
	init("ddc8_0_3", ddc8_0_3, sizeof(ddc8_0_3) / sizeof(unsigned int));
	init("ddc8_0_5", ddc8_0_5, sizeof(ddc8_0_5) / sizeof(unsigned int));
	init("ddc8_0_6", ddc8_0_6, sizeof(ddc8_0_6) / sizeof(unsigned int));
	init("ddc8_0_7", ddc8_0_7, sizeof(ddc8_0_7) / sizeof(unsigned int));
	init("ddc8_0_8", ddc8_0_8, sizeof(ddc8_0_8) / sizeof(unsigned int));
	init("ddc8_1_0", ddc8_1_0, sizeof(ddc8_1_0) / sizeof(unsigned int));

	init("ddc4_1_0", ddc4_1_0, sizeof(ddc4_1_0) / sizeof(unsigned int));
    
    init("ddc10_0_5_flat", ddc10_0_5_flat, sizeof(ddc10_0_5_flat)/ sizeof(unsigned int));
    init("ddc10_0_5", ddc10_0_5, sizeof(ddc10_0_5) / sizeof(unsigned int));
}

