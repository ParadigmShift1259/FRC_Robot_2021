// Game Auto Prelim test
// export format "t,v,a,X,Y,H"
double RightSideStraightBack[][6] = {
        {0,0,0,3.1441,0.6958,0},
        {0.0103,0.0116,1.125,3.1442,0.6958,0},
        {0.0206,0.0232,1.125,3.1445,0.6958,0.0001},
        {0.0301,0.0381,1.5692,3.1448,0.6958,0.0002},
        {0.0406,0.0531,1.4221,3.1454,0.6958,0.0003},
        {0.05,0.068,1.5912,3.146,0.6958,0.0005},
        {0.0602,0.0827,1.4445,3.1469,0.6958,0.0008},
        {0.07,0.0977,1.525,3.1478,0.6958,0.0011},
        {0.0803,0.1128,1.4664,3.149,0.6958,0.0014},
        {0.09,0.1278,1.5447,3.1502,0.6958,0.0018},
        {0.1001,0.1426,1.4707,3.1517,0.6958,0.0022},
        {0.11,0.1576,1.5149,3.1532,0.6958,0.0027},
        {0.12,0.1725,1.4943,3.1549,0.6958,0.0032},
        {0.1301,0.1876,1.4956,3.1568,0.6958,0.0038},
        {0.1401,0.2027,1.4965,3.1589,0.6958,0.0045},
        {0.1501,0.2177,1.5105,3.161,0.6958,0.0051},
        {0.1601,0.2327,1.4959,3.1634,0.6958,0.0059},
        {0.1702,0.2477,1.4966,3.1658,0.6958,0.0066},
        {0.1801,0.2627,1.5083,3.1685,0.6958,0.0075},
        {0.1901,0.2776,1.4963,3.1712,0.6958,0.0083},
        {0.2001,0.2927,1.4968,3.1742,0.6958,0.0092},
        {0.2101,0.3076,1.5068,3.1772,0.6958,0.0102},
        {0.2201,0.3226,1.4966,3.1805,0.6958,0.0112},
        {0.2301,0.3376,1.497,3.1838,0.6958,0.0122},
        {0.24,0.3526,1.5057,3.1873,0.6958,0.0133},
        {0.25,0.3676,1.4969,3.191,0.6958,0.0145},
        {0.2601,0.3826,1.4972,3.1949,0.6958,0.0156},
        {0.27,0.3976,1.5049,3.1988,0.6958,0.0169},
        {0.2801,0.4126,1.4971,3.203,0.6958,0.0181},
        {0.29,0.4276,1.5043,3.2072,0.6958,0.0195},
        {0.3,0.4425,1.497,3.2116,0.6958,0.0208},
        {0.31,0.4575,1.4973,3.2162,0.6958,0.0222},
        {0.32,0.4725,1.5038,3.2209,0.6958,0.0236},
        {0.33,0.4875,1.4972,3.2258,0.6958,0.0251},
        {0.3401,0.5026,1.4974,3.2309,0.6958,0.0267},
        {0.35,0.5176,1.5093,3.236,0.6958,0.0282},
        {0.3601,0.5325,1.4914,3.2414,0.6958,0.0298},
        {0.3701,0.5476,1.503,3.2468,0.6959,0.0315},
        {0.38,0.5626,1.5028,3.2524,0.6959,0.0332},
        {0.39,0.5775,1.4972,3.2582,0.6959,0.0349},
        {0.4001,0.5926,1.4974,3.2642,0.6959,0.0367},
        {0.41,0.6076,1.5075,3.2702,0.6959,0.0385},
        {0.42,0.6225,1.4923,3.2765,0.6959,0.0403},
        {0.4301,0.6376,1.5023,3.2828,0.6959,0.0422},
        {0.44,0.6526,1.5021,3.2894,0.6959,0.0441},
        {0.45,0.6675,1.5019,3.296,0.6959,0.046},
        {0.46,0.6825,1.4972,3.3028,0.6959,0.048},
        {0.47,0.6975,1.4973,3.3098,0.6959,0.0501},
        {0.4801,0.7126,1.5017,3.317,0.6959,0.0521},
        {0.49,0.7276,1.5016,3.3242,0.6959,0.0542},
        {0.5,0.7426,1.5014,3.3316,0.6959,0.0563},
        {0.51,0.7575,1.4972,3.3392,0.6959,0.0585},
        {0.52,0.7725,1.5013,3.3469,0.6959,0.0607},
        {0.53,0.7876,1.4973,3.3548,0.6959,0.0629},
        {0.54,0.8025,1.5051,3.3628,0.696,0.0652},
        {0.55,0.8175,1.4972,3.371,0.696,0.0675},
        {0.56,0.8325,1.4972,3.3794,0.696,0.0698},
        {0.57,0.8476,1.5047,3.3878,0.696,0.0722},
        {0.58,0.8625,1.4972,3.3965,0.696,0.0746},
        {0.59,0.8776,1.5008,3.4052,0.696,0.077},
        {0.6,0.8925,1.5007,3.4142,0.696,0.0794},
        {0.61,0.9075,1.5006,3.4232,0.696,0.0819},
        {0.62,0.9225,1.4972,3.4325,0.696,0.0844},
        {0.63,0.9376,1.5006,3.4418,0.6961,0.087},
        {0.64,0.9526,1.5005,3.4514,0.6961,0.0895},
        {0.65,0.9675,1.5038,3.461,0.6961,0.0921},
        {0.66,0.9825,1.4939,3.4709,0.6961,0.0948},
        {0.67,0.9976,1.5036,3.4808,0.6961,0.0974},
        {0.68,1.0125,1.5003,3.491,0.6961,0.1001},
        {0.69,1.0275,1.5002,3.5012,0.6962,0.1028},
        {0.7,1.0425,1.4971,3.5117,0.6962,0.1055},
        {0.71,1.0575,1.5032,3.5222,0.6962,0.1083},
        {0.72,1.0725,1.4971,3.533,0.6962,0.1111},
        {0.73,1.0875,1.5031,3.5438,0.6962,0.1139},
        {0.74,1.1025,1.4971,3.5549,0.6963,0.1167},
        {0.75,1.1176,1.5,3.566,0.6963,0.1196},
        {0.76,1.1325,1.5028,3.5774,0.6963,0.1224},
        {0.77,1.1475,1.497,3.5888,0.6963,0.1253},
        {0.78,1.1625,1.5027,3.6005,0.6964,0.1283},
        {0.79,1.1775,1.4998,3.6122,0.6964,0.1312},
        {0.8,1.1925,1.497,3.6242,0.6964,0.1342},
        {0.81,1.2075,1.5025,3.6362,0.6964,0.1372},
        {0.82,1.2225,1.4997,3.6485,0.6965,0.1402},
        {0.83,1.2375,1.4997,3.6608,0.6965,0.1432},
        {0.84,1.2525,1.4997,3.6734,0.6965,0.1463},
        {0.85,1.2675,1.4997,3.686,0.6966,0.1494},
        {0.86,1.2825,1.5022,3.6988,0.6966,0.1525},
        {0.87,1.2975,1.497,3.7118,0.6966,0.1556},
        {0.88,1.3125,1.5021,3.725,0.6967,0.1588},
        {0.89,1.3275,1.4995,3.7382,0.6967,0.1619},
        {0.9,1.3425,1.4995,3.7517,0.6968,0.1651},
        {0.91,1.3575,1.4995,3.7652,0.6968,0.1683},
        {0.92,1.3725,1.5019,3.779,0.6968,0.1716},
        {0.93,1.3875,1.4994,3.7928,0.6969,0.1748},
        {0.94,1.4025,1.4994,3.8068,0.6969,0.1781},
        {0.95,1.4175,1.4994,3.821,0.697,0.1814},
        {0.96,1.4325,1.5018,3.8353,0.697,0.1847},
        {0.97,1.4475,1.497,3.8498,0.6971,0.188},
        {0.98,1.4625,1.5017,3.8645,0.6971,0.1913},
        {0.99,1.4775,1.5016,3.8792,0.6972,0.1947},
        {1,1.4925,1.497,3.8942,0.6972,0.1981},
        {1.01,1.5075,1.5016,3.9092,0.6973,0.2015},
        {1.02,1.5225,1.5015,3.9244,0.6973,0.2049},
        {1.03,1.5375,1.497,3.9398,0.6974,0.2083},
        {1.04,1.5525,1.5015,3.9554,0.6974,0.2118},
        {1.05,1.5675,1.5014,3.971,0.6975,0.2153},
        {1.06,1.5825,1.497,3.9869,0.6975,0.2188},
        {1.07,1.5975,1.5014,4.0028,0.6976,0.2223},
        {1.08,1.6125,1.5014,4.0189,0.6977,0.2258},
        {1.09,1.6275,1.4971,4.0352,0.6977,0.2294},
        {1.1,1.6425,1.5013,4.0517,0.6978,0.2329},
        {1.11,1.6575,1.5013,4.0682,0.6979,0.2365},
        {1.12,1.6725,1.4992,4.0849,0.6979,0.2401},
        {1.13,1.6875,1.4992,4.1018,0.698,0.2437},
        {1.14,1.7025,1.5012,4.1188,0.6981,0.2474},
        {1.15,1.7175,1.4991,4.136,0.6982,0.251},
        {1.16,1.7325,1.5012,4.1533,0.6982,0.2547},
        {1.17,1.7475,1.4991,4.1708,0.6983,0.2584},
        {1.18,1.7625,1.4991,4.1885,0.6984,0.2621},
        {1.19,1.7775,1.5011,4.2062,0.6985,0.2659},
        {1.2,1.7925,1.4991,4.2242,0.6986,0.2696},
        {1.21,1.8075,1.5011,4.2422,0.6987,0.2734},
        {1.22,1.8225,1.4991,4.2605,0.6987,0.2772},
        {1.23,1.8375,1.5011,4.2788,0.6988,0.281},
        {1.24,1.8525,1.4991,4.2973,0.6989,0.2848},
        {1.25,1.8675,1.4991,4.316,0.699,0.2887},
        {1.26,1.8825,1.501,4.3349,0.6991,0.2925},
        {1.27,1.8975,1.501,4.3538,0.6992,0.2964},
        {1.28,1.9125,1.4991,4.3729,0.6993,0.3003},
        {1.29,1.9275,1.4991,4.3922,0.6994,0.3042},
        {1.3,1.9425,1.501,4.4116,0.6995,0.3082},
        {1.31,1.9575,1.4991,4.4312,0.6996,0.3122},
        {1.32,1.9725,1.501,4.4509,0.6997,0.3162},
        {1.33,1.9875,1.4991,4.4708,0.6998,0.3202},
        {1.34,2.0025,1.501,4.4908,0.7,0.3242},
        {1.35,2.0175,1.4991,4.511,0.7001,0.3283},
        {1.36,2.0325,1.5009,4.5313,0.7002,0.3323},
        {1.37,2.0475,1.4992,4.5518,0.7003,0.3364},
        {1.38,2.0625,1.4992,4.5725,0.7004,0.3405},
        {1.39,2.0775,1.5009,4.5932,0.7006,0.3447},
        {1.4,2.0925,1.5009,4.6141,0.7007,0.3489},
        {1.41,2.1075,1.4992,4.6352,0.7008,0.353},
        {1.42,2.1225,1.4992,4.6565,0.701,0.3573},
        {1.43,2.1375,1.5009,4.6778,0.7011,0.3615},
        {1.44,2.1525,1.4992,4.6994,0.7012,0.3658},
        {1.45,2.1675,1.5009,4.721,0.7014,0.3701},
        {1.46,2.1825,1.4992,4.7429,0.7015,0.3744},
        {1.47,2.1975,1.5009,4.7648,0.7017,0.3787},
        {1.48,2.2125,1.4993,4.787,0.7018,0.3831},
        {1.49,2.2275,1.5009,4.8092,0.7019,0.3875},
        {1.5,2.2425,1.4993,4.8316,0.7021,0.3919},
        {1.51,2.2575,1.4993,4.8542,0.7023,0.3963},
        {1.52,2.2725,1.5009,4.8769,0.7024,0.4008},
        {1.53,2.2875,1.4993,4.8998,0.7026,0.4053},
        {1.54,2.3025,1.501,4.9228,0.7027,0.4098},
        {1.55,2.3175,1.4994,4.946,0.7029,0.4144},
        {1.56,2.3325,1.4994,4.9693,0.7031,0.419},
        {1.57,2.3475,1.501,4.9928,0.7033,0.4236},
        {1.58,2.3625,1.4994,5.0164,0.7034,0.4283},
        {1.59,2.3775,1.4994,5.0402,0.7036,0.433},
        {1.6,2.3925,1.501,5.0641,0.7038,0.4377},
        {1.61,2.4075,1.4995,5.0882,0.704,0.4425},
        {1.62,2.4225,1.4995,5.1125,0.7042,0.4473},
        {1.63,2.4375,1.501,5.1368,0.7044,0.4521},
        {1.64,2.4525,1.4995,5.1613,0.7046,0.457},
        {1.65,2.4675,1.5011,5.186,0.7048,0.4619},
        {1.66,2.4825,1.4981,5.2108,0.705,0.4668},
        {1.67,2.4967,1.4167,5.2358,0.7052,0.4718},
        {1.68,2.5,0.332,5.2608,0.7054,0.4768},
        {1.69,2.5,0,5.2858,0.7056,0.4818},
        {1.7,2.5,0,5.3108,0.7058,0.4868},
        {1.71,2.5,0,5.3358,0.706,0.4918},
        {1.72,2.5,0,5.3608,0.7062,0.4969},
        {1.73,2.5,0,5.3858,0.7064,0.502},
        {1.74,2.5,0,5.4108,0.7067,0.507},
        {1.75,2.5,0,5.4358,0.7069,0.5121},
        {1.76,2.5,0,5.4608,0.7071,0.5172},
        {1.77,2.5,0,5.4858,0.7073,0.5224},
        {1.78,2.5,0,5.5108,0.7076,0.5275},
        {1.79,2.5,0,5.5358,0.7078,0.5326},
        {1.8,2.5,0,5.5608,0.708,0.5378},
        {1.81,2.5,0,5.5858,0.7083,0.543},
        {1.82,2.5,0,5.6108,0.7085,0.5482},
        {1.83,2.5,0,5.6358,0.7088,0.5534},
        {1.84,2.5,0,5.6608,0.709,0.5587},
        {1.85,2.5,0,5.6858,0.7092,0.564},
        {1.86,2.5,0,5.7108,0.7095,0.5693},
        {1.87,2.5,0,5.7358,0.7097,0.5746},
        {1.88,2.4986,-0.136,5.7608,0.71,0.5799},
        {1.89,2.4861,-1.2534,5.7856,0.7103,0.5853},
        {1.9,2.4711,-1.5002,5.8103,0.7105,0.5906},
        {1.91,2.4561,-1.4987,5.8349,0.7108,0.596},
        {1.92,2.4411,-1.5003,5.8593,0.711,0.6013},
        {1.93,2.4261,-1.5003,5.8836,0.7113,0.6066},
        {1.94,2.4111,-1.5003,5.9077,0.7115,0.612},
        {1.95,2.3961,-1.5003,5.9316,0.7118,0.6173},
        {1.96,2.3811,-1.4988,5.9554,0.7121,0.6226},
        {1.97,2.3661,-1.5004,5.9791,0.7123,0.6279},
        {1.98,2.3511,-1.5004,6.0026,0.7126,0.6332},
        {1.99,2.3361,-1.5004,6.026,0.7128,0.6385},
        {2,2.3211,-1.4988,6.0492,0.7131,0.6438},
        {2.01,2.3061,-1.5021,6.0722,0.7134,0.6491},
        {2.02,2.2911,-1.4972,6.0952,0.7136,0.6544},
        {2.03,2.2761,-1.5021,6.1179,0.7139,0.6596},
        {2.04,2.2611,-1.4989,6.1405,0.7141,0.6649},
        {2.05,2.2461,-1.5005,6.163,0.7144,0.6702},
        {2.06,2.2311,-1.5005,6.1853,0.7147,0.6755},
        {2.07,2.2161,-1.4989,6.2075,0.7149,0.6807},
        {2.08,2.2011,-1.5006,6.2295,0.7152,0.686},
        {2.09,2.1861,-1.5006,6.2513,0.7155,0.6913},
        {2.1,2.1711,-1.5006,6.273,0.7157,0.6965},
        {2.11,2.1561,-1.4989,6.2946,0.716,0.7018},
        {2.12,2.1411,-1.4989,6.316,0.7162,0.707},
        {2.13,2.1261,-1.5024,6.3372,0.7165,0.7123},
        {2.14,2.1111,-1.4989,6.3583,0.7168,0.7175},
        {2.15,2.0961,-1.4989,6.3793,0.717,0.7228},
        {2.16,2.0811,-1.5006,6.4002,0.7173,0.7281},
        {2.17,2.0661,-1.5007,6.4208,0.7176,0.7333},
        {2.18,2.0511,-1.5007,6.4413,0.7178,0.7385},
        {2.19,2.0361,-1.4989,6.4617,0.7181,0.7438},
        {2.2,2.0211,-1.5007,6.4819,0.7184,0.749},
        {2.21,2.0061,-1.4989,6.5019,0.7186,0.7542},
        {2.22,1.9911,-1.5007,6.5218,0.7189,0.7595},
        {2.23,1.9761,-1.5007,6.5416,0.7192,0.7647},
        {2.24,1.9611,-1.4989,6.5612,0.7194,0.7699},
        {2.25,1.9461,-1.5007,6.5807,0.7197,0.7751},
        {2.26,1.9311,-1.4988,6.6,0.7199,0.7804},
        {2.27,1.9161,-1.5007,6.6191,0.7202,0.7856},
        {2.28,1.9011,-1.5007,6.6381,0.7205,0.7908},
        {2.29,1.8861,-1.4988,6.657,0.7207,0.796},
        {2.3,1.8711,-1.5007,6.6757,0.721,0.8012},
        {2.31,1.8561,-1.5007,6.6943,0.7213,0.8064},
        {2.32,1.8411,-1.4988,6.7127,0.7215,0.8116},
        {2.33,1.8261,-1.4988,6.731,0.7218,0.8168},
        {2.34,1.8111,-1.5027,6.7491,0.722,0.822},
        {2.35,1.7961,-1.4988,6.767,0.7223,0.8272},
        {2.36,1.7811,-1.5008,6.7848,0.7226,0.8324},
        {2.37,1.7661,-1.4987,6.8025,0.7228,0.8375},
        {2.38,1.7511,-1.5007,6.82,0.7231,0.8427},
        {2.39,1.7361,-1.4987,6.8374,0.7233,0.8479},
        {2.4,1.7211,-1.5007,6.8546,0.7236,0.853},
        {2.41,1.7061,-1.5007,6.8716,0.7238,0.8582},
        {2.42,1.6911,-1.5007,6.8885,0.7241,0.8633},
        {2.43,1.6761,-1.4986,6.9053,0.7243,0.8685},
        {2.44,1.6611,-1.5007,6.9219,0.7246,0.8736},
        {2.45,1.6461,-1.4986,6.9384,0.7249,0.8787},
        {2.46,1.6311,-1.5007,6.9547,0.7251,0.8839},
        {2.47,1.6161,-1.5007,6.9708,0.7254,0.889},
        {2.48,1.6011,-1.5007,6.9868,0.7256,0.8941},
        {2.49,1.5861,-1.4985,7.0027,0.7259,0.8992},
        {2.5,1.5711,-1.5007,7.0184,0.7261,0.9042},
        {2.51,1.5561,-1.4984,7.034,0.7263,0.9093},
        {2.52,1.5411,-1.5029,7.0494,0.7266,0.9144},
        {2.53,1.5261,-1.4984,7.0646,0.7268,0.9194},
        {2.54,1.5111,-1.4983,7.0798,0.7271,0.9245},
        {2.55,1.4961,-1.503,7.0947,0.7273,0.9295},
        {2.56,1.4811,-1.4983,7.1095,0.7276,0.9345},
        {2.57,1.4661,-1.5006,7.1242,0.7278,0.9395},
        {2.58,1.4511,-1.4982,7.1387,0.728,0.9445},
        {2.59,1.4361,-1.503,7.153,0.7283,0.9495},
        {2.6,1.4211,-1.4981,7.1673,0.7285,0.9544},
        {2.61,1.4061,-1.5005,7.1813,0.7288,0.9594},
        {2.62,1.3911,-1.5005,7.1952,0.729,0.9643},
        {2.63,1.3761,-1.498,7.209,0.7292,0.9692},
        {2.64,1.3611,-1.503,7.2226,0.7295,0.9741},
        {2.65,1.3461,-1.498,7.236,0.7297,0.979},
        {2.66,1.3311,-1.5005,7.2494,0.7299,0.9839},
        {2.67,1.3161,-1.5004,7.2625,0.7301,0.9887},
        {2.68,1.3011,-1.4978,7.2755,0.7304,0.9935},
        {2.69,1.2861,-1.503,7.2884,0.7306,0.9983},
        {2.7,1.2711,-1.4977,7.3011,0.7308,1.0031},
        {2.71,1.2561,-1.5003,7.3137,0.731,1.0079},
        {2.72,1.2411,-1.5003,7.3261,0.7313,1.0126},
        {2.73,1.2261,-1.5003,7.3383,0.7315,1.0174},
        {2.74,1.2111,-1.5003,7.3504,0.7317,1.022},
        {2.75,1.1961,-1.5003,7.3624,0.7319,1.0267},
        {2.76,1.1811,-1.4974,7.3742,0.7321,1.0314},
        {2.77,1.1661,-1.5031,7.3859,0.7323,1.036},
        {2.78,1.1511,-1.5002,7.3974,0.7325,1.0406},
        {2.79,1.1361,-1.4972,7.4087,0.7327,1.0451},
        {2.8,1.1211,-1.5031,7.4199,0.7329,1.0497},
        {2.81,1.1061,-1.4971,7.431,0.7332,1.0542},
        {2.82,1.0911,-1.5,7.4419,0.7334,1.0587},
        {2.83,1.0761,-1.5031,7.4527,0.7336,1.0631},
        {2.84,1.0611,-1.4969,7.4633,0.7338,1.0675},
        {2.85,1.0461,-1.5031,7.4737,0.7339,1.0719},
        {2.86,1.0311,-1.4967,7.4841,0.7341,1.0763},
        {2.87,1.0161,-1.5031,7.4942,0.7343,1.0806},
        {2.88,1.0011,-1.4965,7.5042,0.7345,1.0849},
        {2.89,0.9861,-1.5031,7.5141,0.7347,1.0891},
        {2.9,0.9711,-1.4998,7.5238,0.7349,1.0933},
        {2.91,0.9561,-1.4962,7.5334,0.7351,1.0975},
        {2.92,0.9411,-1.5031,7.5428,0.7353,1.1016},
        {2.93,0.9261,-1.4996,7.552,0.7354,1.1057},
        {2.94,0.9111,-1.4995,7.5611,0.7356,1.1098},
        {2.95,0.8961,-1.5032,7.5701,0.7358,1.1138},
        {2.96,0.8811,-1.4958,7.5789,0.736,1.1177},
        {2.97,0.8661,-1.5031,7.5875,0.7361,1.1217},
        {2.98,0.8511,-1.4955,7.5961,0.7363,1.1255},
        {2.99,0.8361,-1.5031,7.6044,0.7365,1.1294},
        {3.0001,0.8211,-1.4992,7.6127,0.7366,1.1332},
        {3.01,0.8061,-1.5032,7.6207,0.7368,1.1369},
        {3.02,0.7911,-1.495,7.6286,0.7369,1.1406},
        {3.03,0.7761,-1.5032,7.6364,0.7371,1.1443},
        {3.04,0.7611,-1.499,7.644,0.7372,1.1478},
        {3.05,0.7461,-1.4989,7.6515,0.7374,1.1514},
        {3.06,0.7311,-1.5032,7.6587,0.7375,1.1549},
        {3.07,0.7161,-1.4988,7.6659,0.7377,1.1583},
        {3.08,0.7011,-1.4987,7.6729,0.7378,1.1617},
        {3.09,0.6861,-1.4986,7.6798,0.738,1.165},
        {3.1,0.6711,-1.5033,7.6865,0.7381,1.1683},
        {3.11,0.6561,-1.4985,7.693,0.7382,1.1715},
        {3.12,0.6411,-1.4983,7.6995,0.7384,1.1747},
        {3.13,0.6261,-1.5033,7.7057,0.7385,1.1778},
        {3.14,0.6111,-1.4982,7.7118,0.7386,1.1808},
        {3.15,0.5961,-1.498,7.7178,0.7388,1.1838},
        {3.16,0.5811,-1.5034,7.7236,0.7389,1.1868},
        {3.17,0.5661,-1.4979,7.7293,0.739,1.1896},
        {3.18,0.5511,-1.5035,7.7347,0.7391,1.1924},
        {3.1901,0.5361,-1.4917,7.7401,0.7392,1.1952},
        {3.2,0.5211,-1.5096,7.7453,0.7393,1.1978},
        {3.21,0.5061,-1.4975,7.7504,0.7394,1.2004},
        {3.22,0.4911,-1.4973,7.7553,0.7395,1.203},
        {3.2301,0.4761,-1.497,7.7601,0.7396,1.2054},
        {3.2401,0.461,-1.5036,7.7647,0.7397,1.2079},
        {3.25,0.4461,-1.5039,7.7691,0.7398,1.2102},
        {3.26,0.4311,-1.4967,7.7734,0.7399,1.2125},
        {3.2701,0.4161,-1.4963,7.7776,0.74,1.2147},
        {3.2801,0.401,-1.5039,7.7816,0.7401,1.2168},
        {3.29,0.3861,-1.5043,7.7855,0.7402,1.2188},
        {3.3,0.3711,-1.4961,7.7892,0.7403,1.2208},
        {3.3101,0.3561,-1.4955,7.7928,0.7403,1.2227},
        {3.32,0.3411,-1.5043,7.7962,0.7404,1.2246},
        {3.3301,0.326,-1.495,7.7994,0.7405,1.2263},
        {3.3401,0.311,-1.5045,7.8025,0.7405,1.228},
        {3.35,0.296,-1.5052,7.8055,0.7406,1.2296},
        {3.36,0.2811,-1.4944,7.8083,0.7407,1.2311},
        {3.3701,0.266,-1.4934,7.811,0.7407,1.2326},
        {3.3802,0.2509,-1.505,7.8135,0.7408,1.234},
        {3.3901,0.2359,-1.5059,7.8158,0.7408,1.2353},
        {3.4002,0.2209,-1.4924,7.8181,0.7409,1.2365},
        {3.4101,0.2059,-1.5064,7.8201,0.7409,1.2377},
        {3.4202,0.1909,-1.4908,7.822,0.741,1.2387},
        {3.4302,0.1758,-1.5069,7.8238,0.741,1.2397},
        {3.4401,0.1609,-1.5089,7.8254,0.741,1.2406},
        {3.4501,0.146,-1.489,7.8269,0.7411,1.2415},
        {3.4603,0.1308,-1.4852,7.8282,0.7411,1.2422},
        {3.4701,0.1158,-1.5373,7.8293,0.7411,1.2429},
        {3.4804,0.1008,-1.4547,7.8304,0.7411,1.2435},
        {3.4902,0.0857,-1.5472,7.8312,0.7412,1.244},
        {3.5002,0.0708,-1.4797,7.8319,0.7412,1.2444},
        {3.5108,0.0554,-1.4624,7.8325,0.7412,1.2448},
        {3.52,0.0405,-1.6036,7.8329,0.7412,1.245},
        {3.5319,0.0247,-1.3377,7.8332,0.7412,1.2452},
        {3.5424,0.0079,-1.591,7.8332,0.7412,1.2453}
    }
    ;