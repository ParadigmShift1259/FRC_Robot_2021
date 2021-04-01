#include <vector>
#include <frc/geometry/Translation2d.h>
#include "units/angle.h"
#include "units/length.h"
double BarrelRacing[][6] = {
        {0,0,0,1.9856,1.8201,-0.0005},
        {0.0209,0.0239,1.1428,1.9861,1.8201,-0.0042},
        {0.0403,0.0612,1.9245,1.9873,1.8201,-0.0132},
        {0.0602,0.1005,1.9751,1.9893,1.8201,-0.0282},
        {0.0802,0.1403,1.9941,1.9921,1.8201,-0.0494},
        {0.1002,0.1804,1.9981,1.9957,1.8201,-0.0766},
        {0.1202,0.2204,1.9996,2.0001,1.8201,-0.11},
        {0.14,0.2603,2.0115,2.0053,1.8201,-0.149},
        {0.1601,0.3001,1.9884,2.0113,1.8201,-0.1946},
        {0.1801,0.3402,2.0015,2.0181,1.8201,-0.2463},
        {0.2001,0.3802,2.0017,2.0257,1.8201,-0.3041},
        {0.22,0.4201,2.0019,2.0341,1.8201,-0.368},
        {0.24,0.4601,1.9957,2.0433,1.8202,-0.4385},
        {0.26,0.5,2.0033,2.0533,1.8202,-0.5151},
        {0.28,0.54,1.9978,2.0641,1.8202,-0.5983},
        {0.3,0.58,1.999,2.0757,1.8203,-0.6881},
        {0.3201,0.6201,1.9999,2.0881,1.8203,-0.7844},
        {0.3401,0.6601,2.0006,2.1013,1.8204,-0.8874},
        {0.3601,0.7001,2.0013,2.1153,1.8205,-0.9969},
        {0.38,0.7401,2.0018,2.1301,1.8206,-1.113},
        {0.4001,0.7801,1.9949,2.1457,1.8207,-1.2366},
        {0.42,0.8201,2.0035,2.1621,1.8208,-1.3668},
        {0.44,0.86,2.0004,2.1793,1.821,-1.504},
        {0.46,0.9,1.998,2.1973,1.8212,-1.6488},
        {0.48,0.94,1.9991,2.2161,1.8214,-1.801},
        {0.5,0.98,2.003,2.2357,1.8216,-1.9604},
        {0.52,1.02,1.998,2.2561,1.8219,-2.1277},
        {0.54,1.06,1.9991,2.2773,1.8222,-2.303},
        {0.56,1.1,2.0002,2.2993,1.8226,-2.4863},
        {0.58,1.1401,2.0011,2.3221,1.823,-2.6776},
        {0.6,1.1801,1.9996,2.3457,1.8234,-2.8773},
        {0.62,1.22,2.0006,2.3701,1.8239,-3.0856},
        {0.64,1.26,1.9994,2.3953,1.8245,-3.3027},
        {0.66,1.3,2.0006,2.4213,1.8251,-3.5288},
        {0.68,1.34,1.9996,2.448,1.8258,-3.7642},
        {0.7,1.38,1.9989,2.4756,1.8266,-4.0096},
        {0.72,1.42,2.0001,2.5041,1.8274,-4.2648},
        {0.74,1.46,2.0013,2.5332,1.8283,-4.5299},
        {0.76,1.5,1.999,2.5632,1.8293,-4.8057},
        {0.78,1.54,2.0004,2.594,1.8305,-5.0924},
        {0.8,1.58,1.9985,2.6256,1.8317,-5.3909},
        {0.82,1.62,2.0015,2.6579,1.833,-5.7006},
        {0.84,1.66,1.9999,2.6911,1.8345,-6.0226},
        {0.86,1.7,1.9986,2.7251,1.8361,-6.3577},
        {0.88,1.74,2.0016,2.7598,1.8379,-6.7055},
        {0.9,1.78,1.9992,2.7954,1.8398,-7.0674},
        {0.92,1.82,1.9997,2.8317,1.8419,-7.4439},
        {0.94,1.86,2.0003,2.8688,1.8442,-7.8354},
        {0.96,1.9,1.9998,2.9067,1.8467,-8.2429},
        {0.98,1.94,1.9996,2.9455,1.8494,-8.6673},
        {1,1.98,1.9996,2.985,1.8524,-9.1095},
        {1.02,2.02,2.0007,3.0252,1.8556,-9.5701},
        {1.04,2.06,2,3.0663,1.8591,-10.0505},
        {1.06,2.1,1.9996,3.1081,1.863,-10.552},
        {1.08,2.14,2.0004,3.1507,1.8672,-11.0756},
        {1.1,2.18,1.9989,3.1941,1.8717,-11.6236},
        {1.12,2.22,2.0011,3.2381,1.8767,-12.1965},
        {1.14,2.26,1.9994,3.283,1.8821,-12.7972},
        {1.16,2.3,1.9998,3.3286,1.8881,-13.4274},
        {1.18,2.34,1.9998,3.375,1.8946,-14.0897},
        {1.2,2.3588,0.9436,3.4216,1.9017,-14.7796},
        {1.22,2.3218,-1.8506,3.4675,1.9091,-15.4832},
        {1.24,2.2818,-1.9998,3.5124,1.9171,-16.2},
        {1.26,2.2418,-1.9999,3.5565,1.9254,-16.9303},
        {1.28,2.2018,-2.0001,3.5996,1.9342,-17.6748},
        {1.3,2.1618,-1.9996,3.6418,1.9435,-18.4343},
        {1.32,2.1218,-2.0003,3.6831,1.9533,-19.2089},
        {1.34,2.0818,-1.9992,3.7235,1.9637,-19.9999},
        {1.36,2.0418,-2.0009,3.7628,1.9745,-20.8064},
        {1.38,2.0018,-1.9991,3.8012,1.9859,-21.6303},
        {1.4,1.9618,-2,3.8386,1.9979,-22.4712},
        {1.42,1.9218,-2.0001,3.8749,2.0104,-23.3289},
        {1.44,1.8818,-1.9995,3.9102,2.0235,-24.2041},
        {1.46,1.8418,-2.0007,3.9444,2.0372,-25.0952},
        {1.48,1.8018,-1.9988,3.9775,2.0515,-26.0032},
        {1.5,1.7619,-1.9971,4.0094,2.0664,-26.9258},
        {1.52,1.725,-1.8432,4.0403,2.0818,-27.8635},
        {1.54,1.6958,-1.4633,4.0702,2.0978,-28.8195},
        {1.56,1.6751,-1.0351,4.0992,2.1146,-29.7961},
        {1.58,1.664,-0.5539,4.1275,2.1321,-30.7986},
        {1.6,1.6641,0.0068,4.1552,2.1504,-31.8312},
        {1.62,1.678,0.6954,4.1826,2.1698,-32.9009},
        {1.64,1.709,1.5485,4.21,2.1903,-34.0172},
        {1.66,1.7488,1.9884,4.2373,2.2121,-35.1837},
        {1.68,1.7888,2.0002,4.2647,2.2352,-36.3973},
        {1.7,1.8288,1.9998,4.2921,2.2594,-37.654},
        {1.72,1.8471,0.9152,4.3193,2.2843,-38.9324},
        {1.74,1.8099,-1.8611,4.3457,2.3091,-40.1883},
        {1.76,1.7699,-1.9997,4.3713,2.3336,-41.4136},
        {1.78,1.7299,-1.9999,4.3963,2.3575,-42.6031},
        {1.8,1.6899,-1.9997,4.4208,2.3808,-43.7532},
        {1.82,1.6499,-2.0004,4.4449,2.4033,-44.8601},
        {1.84,1.6099,-1.9994,4.4687,2.425,-45.9228},
        {1.86,1.5699,-2.0004,4.4922,2.4459,-46.9389},
        {1.88,1.5336,-1.8157,4.515,2.4664,-44.9556},
        {1.9,1.5034,-1.5073,4.5366,2.4872,-42.7687},
        {1.92,1.4752,-1.4094,4.5571,2.5085,-40.5732},
        {1.94,1.4491,-1.3084,4.5764,2.5301,-38.3683},
        {1.96,1.4249,-1.2068,4.5945,2.5521,-36.1529},
        {1.98,1.4028,-1.1044,4.6114,2.5745,-33.9261},
        {2,1.3828,-1.002,4.6272,2.5972,-31.6888},
        {2.02,1.3648,-0.8985,4.6418,2.6202,-29.4392},
        {2.04,1.3489,-0.795,4.6553,2.6436,-27.1773},
        {2.06,1.3351,-0.6915,4.6675,2.6674,-24.9039},
        {2.08,1.3233,-0.5875,4.6786,2.6914,-22.6173},
        {2.1,1.3137,-0.484,4.6884,2.7158,-20.3183},
        {2.12,1.306,-0.3808,4.6971,2.7404,-18.0069},
        {2.14,1.3005,-0.2782,4.7045,2.7653,-15.6832},
        {2.16,1.297,-0.1764,4.7108,2.7905,-13.3472},
        {2.18,1.2954,-0.0758,4.7158,2.8159,-10.9997},
        {2.2,1.2959,0.0235,4.7196,2.8416,-8.6399},
        {2.22,1.2983,0.121,4.7222,2.8674,-6.2697},
        {2.24,1.3027,0.2165,4.7235,2.8934,-3.889},
        {2.26,1.3089,0.3095,4.7236,2.9196,-1.4969},
        {2.28,1.3169,0.4,4.7225,2.9459,0.9047},
        {2.3,1.3266,0.4873,4.7202,2.9723,3.3158},
        {2.32,1.338,0.571,4.7166,2.9988,5.7374},
        {2.34,1.351,0.6512,4.7118,3.0254,8.1684},
        {2.36,1.3656,0.7273,4.7059,3.0521,10.608},
        {2.38,1.3816,0.7986,4.6986,3.0787,13.0571},
        {2.4,1.3989,0.8654,4.6902,3.1054,15.5147},
        {2.42,1.4174,0.9268,4.6806,3.1321,17.9819},
        {2.44,1.4371,0.9832,4.6698,3.1587,20.4576},
        {2.46,1.4577,1.0335,4.6578,3.1853,22.9428},
        {2.48,1.4793,1.0785,4.6446,3.2118,25.4356},
        {2.5,1.5016,1.1169,4.6302,3.2381,27.9369},
        {2.52,1.5246,1.1492,4.6147,3.2644,30.4468},
        {2.54,1.5481,1.1753,4.598,3.2904,32.9643},
        {2.56,1.572,1.1948,4.5802,3.3163,35.4894},
        {2.58,1.5962,1.2082,4.5612,3.342,38.0212},
        {2.6,1.6205,1.2147,4.5411,3.3674,40.5605},
        {2.62,1.6448,1.2156,4.5199,3.3926,43.1056},
        {2.64,1.669,1.2098,4.4976,3.4174,45.6573},
        {2.66,1.6786,0.482,4.4744,3.4417,48.1367},
        {2.68,1.6423,-1.8175,4.4511,3.4648,49.9238},
        {2.7,1.6047,-1.8781,4.4275,3.4866,51.7221},
        {2.72,1.5694,-1.7649,4.4038,3.5072,53.5315},
        {2.74,1.5364,-1.6507,4.3799,3.5265,55.3539},
        {2.76,1.5057,-1.5366,4.3558,3.5446,57.1908},
        {2.78,1.4772,-1.4235,4.3316,3.5614,59.0415},
        {2.8,1.451,-1.3089,4.3071,3.5771,60.9095},
        {2.82,1.4271,-1.1959,4.2825,3.5915,62.7937},
        {2.84,1.4055,-1.0823,4.2577,3.6048,64.696},
        {2.86,1.3861,-0.9691,4.2328,3.6168,66.6181},
        {2.88,1.369,-0.8569,4.2076,3.6277,68.56},
        {2.9,1.3541,-0.7448,4.1823,3.6373,70.5234},
        {2.92,1.3414,-0.6338,4.1568,3.6457,72.5091},
        {2.94,1.3309,-0.5238,4.1312,3.6529,74.5181},
        {2.96,1.3226,-0.4153,4.1055,3.6589,76.5511},
        {2.98,1.3164,-0.3086,4.0796,3.6637,78.6083},
        {3,1.3124,-0.2038,4.0536,3.6672,80.692},
        {3.02,1.3103,-0.1019,4.0275,3.6696,82.8016},
        {3.04,1.3103,-0.0031,4.0013,3.6707,84.9396},
        {3.06,1.3121,0.0918,3.975,3.6706,87.1051},
        {3.08,1.3157,0.182,3.9488,3.6692,89.2998},
        {3.1,1.3211,0.2667,3.9225,3.6667,91.5255},
        {3.12,1.328,0.3448,3.8962,3.6629,93.783},
        {3.14,1.3363,0.4152,3.8699,3.6579,96.0731},
        {3.16,1.3458,0.4764,3.8437,3.6517,98.3969},
        {3.18,1.3563,0.5267,3.8177,3.6443,100.7558},
        {3.2,1.3676,0.5644,3.7917,3.6357,103.1518},
        {3.22,1.3794,0.5876,3.7659,3.6259,105.5856},
        {3.24,1.3912,0.5936,3.7403,3.6149,108.059},
        {3.26,1.4028,0.58,3.715,3.6028,110.5736},
        {3.28,1.4137,0.5441,3.69,3.5896,113.1295},
        {3.3,1.4234,0.4825,3.6654,3.5753,115.7284},
        {3.32,1.4312,0.3925,3.6413,3.56,118.3704},
        {3.34,1.4366,0.2709,3.6176,3.5436,121.057},
        {3.36,1.4389,0.1154,3.5946,3.5263,123.7866},
        {3.38,1.4374,-0.0761,3.5723,3.5082,126.5601},
        {3.4,1.4313,-0.3039,3.5509,3.4892,129.3749},
        {3.42,1.42,-0.5665,3.5303,3.4696,132.2301},
        {3.44,1.4083,-0.5868,3.5107,3.4494,134.4098},
        {3.46,1.4362,1.3965,3.4911,3.4284,135.5096},
        {3.48,1.4762,1.9993,3.4712,3.4066,136.6724},
        {3.5,1.5162,2.0002,3.4511,3.3839,137.9019},
        {3.52,1.5562,2.0008,3.4308,3.3603,139.2029},
        {3.54,1.5962,1.9991,3.4104,3.3358,140.5838},
        {3.56,1.6362,2.0002,3.39,3.3102,142.0502},
        {3.58,1.6762,1.9993,3.3697,3.2835,143.6115},
        {3.6,1.6757,-0.0258,3.35,3.2564,145.2349},
        {3.62,1.6357,-1.9994,3.3315,3.2294,146.8874},
        {3.64,1.5957,-1.9999,3.3142,3.2026,148.5691},
        {3.66,1.5557,-1.9996,3.2981,3.176,150.2808},
        {3.68,1.5157,-2.0003,3.2832,3.1496,152.0217},
        {3.7,1.4757,-1.9997,3.2697,3.1233,153.7926},
        {3.72,1.4357,-2.0003,3.2575,3.0974,155.5918},
        {3.74,1.3957,-1.9996,3.2466,3.0717,157.4192},
        {3.76,1.3557,-1.9982,3.2371,3.0463,159.2728},
        {3.78,1.3174,-1.9164,3.229,3.0212,161.151},
        {3.8,1.2833,-1.7047,3.2223,2.9964,163.0573},
        {3.82,1.2537,-1.4817,3.2169,2.9719,164.9946},
        {3.84,1.2285,-1.2577,3.213,2.9477,166.9649},
        {3.86,1.2079,-1.0311,3.2104,2.9237,168.9718},
        {3.88,1.1919,-0.8017,3.2092,2.8999,171.0181},
        {3.9,1.1805,-0.5678,3.2094,2.8763,173.1059},
        {3.92,1.174,-0.3273,3.211,2.8528,175.2378},
        {3.94,1.1724,-0.0778,3.214,2.8296,177.4158},
        {3.96,1.1761,0.1847,3.2184,2.8065,179.6436},
        {3.98,1.1854,0.4657,3.2242,2.7835,-178.0741},
        {4,1.2009,0.774,3.2314,2.7606,-175.7316},
        {4.02,1.2234,1.1232,3.2401,2.7377,-173.3215},
        {4.04,1.2541,1.5355,3.2502,2.7148,-170.8323},
        {4.06,1.2926,1.9267,3.2618,2.6917,-168.252},
        {4.08,1.3326,1.9997,3.2748,2.6684,-165.5787},
        {4.1,1.3726,1.9999,3.2891,2.645,-162.8132},
        {4.12,1.4126,2.0004,3.3048,2.6215,-159.9556},
        {4.14,1.4526,1.9996,3.3216,2.5978,-156.9993},
        {4.16,1.4926,2,3.3394,2.5738,-153.9387},
        {4.18,1.4944,0.0892,3.3576,2.5501,-150.8432},
        {4.2,1.4544,-1.9989,3.3755,2.5272,-147.7901},
        {4.22,1.4144,-1.9999,3.3929,2.5049,-144.7718},
        {4.24,1.3744,-2.0002,3.4095,2.483,-141.7816},
        {4.26,1.3344,-2.0002,3.4253,2.4615,-138.8131},
        {4.28,1.2944,-1.9997,3.44,2.4401,-135.8596},
        {4.3,1.2544,-2.0003,3.4534,2.4189,-132.9203},
        {4.32,1.2335,-1.0454,3.4658,2.3976,-130.8971},
        {4.34,1.2693,1.7914,3.4787,2.3757,-129.2317},
        {4.36,1.3093,1.9998,3.4922,2.3533,-127.4957},
        {4.38,1.3493,2.0002,3.5064,2.3304,-125.6885},
        {4.4,1.3893,2.0001,3.5213,2.3069,-123.8082},
        {4.42,1.4293,1.9998,3.5371,2.2831,-121.8533},
        {4.44,1.4693,2.0003,3.5538,2.2589,-119.8238},
        {4.46,1.5093,1.9996,3.5716,2.2345,-117.7181},
        {4.48,1.5493,2.0003,3.5904,2.2099,-115.5377},
        {4.5,1.5787,1.4668,3.6104,2.1855,-113.2991},
        {4.52,1.548,-1.5321,3.6307,2.1621,-111.09},
        {4.54,1.5083,-1.9848,3.6513,2.1401,-108.9293},
        {4.56,1.4735,-1.7425,3.6722,2.1193,-106.8145},
        {4.58,1.4463,-1.3597,3.6935,2.0998,-104.7407},
        {4.6,1.4259,-1.0182,3.7153,2.0813,-102.7014},
        {4.62,1.4117,-0.7129,3.7375,2.0639,-100.6949},
        {4.64,1.403,-0.4327,3.7603,2.0475,-98.7162},
        {4.66,1.3996,-0.1722,3.7836,2.0321,-96.763},
        {4.68,1.401,0.074,3.8076,2.0177,-94.8343},
        {4.7,1.4072,0.3095,3.8323,2.0041,-92.9262},
        {4.72,1.418,0.5375,3.8577,1.9916,-91.0385},
        {4.74,1.4332,0.7598,3.8839,1.9799,-89.1681},
        {4.76,1.4528,0.9791,3.9109,1.969,-87.3141},
        {4.78,1.4767,1.1964,3.9387,1.9591,-85.4748},
        {4.8,1.5049,1.4128,3.9674,1.95,-83.6479},
        {4.82,1.5375,1.6301,3.997,1.9418,-81.8316},
        {4.84,1.5745,1.8482,4.0276,1.9344,-80.0251},
        {4.86,1.6143,1.9881,4.0592,1.9278,-78.2277},
        {4.88,1.6543,1.9999,4.0918,1.9221,-76.4417},
        {4.9,1.6943,2,4.1253,1.9173,-74.6697},
        {4.92,1.7343,1.9998,4.1598,1.9133,-72.9133},
        {4.94,1.7743,2.0004,4.1951,1.9101,-71.1749},
        {4.96,1.8143,2.0003,4.2313,1.9078,-69.4553},
        {4.98,1.8543,1.9994,4.2684,1.9064,-67.7538},
        {5,1.8943,2.0005,4.3063,1.9057,-66.0728},
        {5.02,1.9343,1.9999,4.345,1.9059,-64.4115},
        {5.04,1.9743,2.0004,4.3844,1.9068,-62.7707},
        {5.06,2.0143,1.9994,4.4247,1.9085,-61.1487},
        {5.08,2.0543,2.0005,4.4657,1.911,-59.5473},
        {5.1,2.0943,1.9997,4.5075,1.9142,-57.9647},
        {5.12,2.1343,2.0005,4.55,1.9181,-56.4017},
        {5.14,2.1743,1.9997,4.5932,1.9226,-54.8569},
        {5.16,2.2143,2,4.6372,1.9279,-53.33},
        {5.18,2.2543,2.0003,4.6819,1.9339,-51.8212},
        {5.2,2.2943,1.9995,4.7273,1.9405,-50.3288},
        {5.22,2.3342,1.9968,4.7734,1.9476,-51.8432},
        {5.24,2.3742,1.9984,4.8203,1.955,-54.361},
        {5.26,2.4142,2,4.868,1.9626,-57.0656},
        {5.28,2.4542,2.0002,4.9164,1.9706,-59.984},
        {5.3,2.46,0.2891,4.9649,1.9788,-63.1072},
        {5.32,2.4202,-1.9885,5.0126,1.9871,-66.4032},
        {5.34,2.3802,-1.9999,5.0595,1.9955,-69.8963},
        {5.36,2.3402,-2,5.1055,2.0039,-73.616},
        {5.38,2.3002,-2,5.1507,2.0125,-77.5968},
        {5.4,2.2602,-1.9996,5.1951,2.0211,-81.881},
        {5.42,2.2202,-2,5.2386,2.0298,-86.5107},
        {5.44,2.1802,-1.9994,5.2814,2.0385,-91.5322},
        {5.46,2.1402,-1.9999,5.3233,2.0472,-96.9734},
        {5.48,2.1002,-1.9994,5.3644,2.0557,-102.832},
        {5.5,2.0602,-1.9992,5.4048,2.0637,-109.0362},
        {5.52,2.0202,-1.9993,5.4446,2.0711,-115.4195},
        {5.54,1.9803,-1.9992,5.4836,2.0775,-121.744},
        {5.56,1.9507,-1.4773,5.5223,2.0829,-127.8035},
        {5.58,1.9782,1.3747,5.5616,2.0875,-133.5942},
        {5.6,2.0182,2.0004,5.6018,2.0912,-139.0419},
        {5.62,2.0582,2.0006,5.6428,2.0943,-144.1043},
        {5.64,2.0982,2,5.6847,2.0968,-148.7917},
        {5.66,2.1382,2.0003,5.7274,2.0989,-153.1334},
        {5.68,2.1782,2.0001,5.7709,2.1007,-157.1693},
        {5.7,2.1935,0.7617,5.8148,2.1022,-160.8979},
        {5.72,2.1553,-1.9097,5.8579,2.1034,-164.2797},
        {5.74,2.1153,-1.9998,5.9002,2.1044,-167.3683},
        {5.76,2.0753,-1.9999,5.9417,2.1053,-170.2061},
        {5.78,2.0353,-2.0003,5.9824,2.1061,-172.8262},
        {5.8,1.9953,-2.0002,6.0223,2.1068,-175.257},
        {5.82,1.9553,-2.0001,6.0614,2.1074,-177.5213},
        {5.84,1.921,-1.7161,6.0998,2.1076,179.0866},
        {5.86,1.9006,-1.0205,6.1378,2.1069,175.3271},
        {5.88,1.8808,-0.9888,6.1754,2.1053,171.5464},
        {5.9,1.8614,-0.9708,6.2125,2.1028,167.7391},
        {5.92,1.8422,-0.9556,6.2492,2.0993,163.9071},
        {5.94,1.8234,-0.9424,6.2854,2.0948,160.052},
        {5.96,1.8048,-0.9307,6.3211,2.0895,156.1704},
        {5.98,1.7864,-0.9216,6.3563,2.0833,152.2622},
        {6,1.7681,-0.9151,6.3909,2.0761,148.3293},
        {6.02,1.7499,-0.9102,6.425,2.0681,144.368},
        {6.04,1.7317,-0.9087,6.4584,2.0592,140.382},
        {6.06,1.7135,-0.9091,6.4913,2.0494,136.3694},
        {6.08,1.6953,-0.9119,6.5235,2.0387,132.3285},
        {6.1,1.6769,-0.9179,6.555,2.0273,128.2628},
        {6.12,1.6584,-0.9258,6.5858,2.015,124.1706},
        {6.14,1.6397,-0.9362,6.6158,2.0019,120.0518},
        {6.16,1.6207,-0.9492,6.6451,1.988,115.9082},
        {6.18,1.6014,-0.9638,6.6736,1.9734,111.7381},
        {6.2,1.5818,-0.9806,6.7013,1.958,107.5432},
        {6.22,1.5618,-0.9991,6.728,1.9419,103.3253},
        {6.24,1.5414,-1.0182,6.7539,1.9252,99.0826},
        {6.26,1.5207,-1.0386,6.7788,1.9077,94.8169},
        {6.28,1.4995,-1.0591,6.8028,1.8897,90.5299},
        {6.3,1.4779,-1.079,6.8257,1.871,86.2218},
        {6.32,1.4559,-1.0981,6.8476,1.8518,81.8941},
        {6.34,1.4336,-1.1152,6.8684,1.8321,77.547},
        {6.36,1.411,-1.1299,6.8881,1.8118,73.1822},
        {6.38,1.3882,-1.1409,6.9066,1.7912,68.7997},
        {6.4,1.3653,-1.148,6.9239,1.7701,64.403},
        {6.42,1.3423,-1.1493,6.94,1.7486,59.9904},
        {6.44,1.3194,-1.145,6.9549,1.7268,55.5636},
        {6.46,1.2967,-1.1338,6.9685,1.7047,51.1244},
        {6.48,1.2744,-1.1147,6.9808,1.6824,46.6729},
        {6.5,1.2527,-1.0875,6.9917,1.6599,42.2107},
        {6.52,1.2316,-1.0515,7.0013,1.6372,37.7397},
        {6.54,1.2115,-1.0057,7.0095,1.6144,33.2581},
        {6.56,1.1925,-0.951,7.0163,1.5915,28.7693},
        {6.58,1.1748,-0.8861,7.0217,1.5687,24.2735},
        {6.6,1.1585,-0.8119,7.0256,1.5458,19.7742},
        {6.62,1.144,-0.7281,7.028,1.5231,15.2731},
        {6.64,1.1313,-0.6353,7.0289,1.5005,10.772},
        {6.66,1.1206,-0.5342,7.0283,1.4781,6.2762},
        {6.68,1.1243,0.185,7.0262,1.4557,3.4471},
        {6.7,1.1623,1.9017,7.0231,1.4326,1.2579},
        {6.72,1.2023,1.9997,7.0188,1.409,-0.9558},
        {6.74,1.2423,2.0001,7.0133,1.3847,-3.1921},
        {6.76,1.2823,2,7.0067,1.36,-5.4509},
        {6.78,1.3223,2.0001,6.999,1.3347,-7.7323},
        {6.8,1.3623,1.9999,6.9901,1.3089,-10.0372},
        {6.82,1.4023,1.9999,6.9801,1.2827,-12.3665},
        {6.84,1.4423,2,6.9689,1.2561,-14.7212},
        {6.86,1.4823,1.9999,6.9566,1.2292,-17.1032},
        {6.88,1.5223,1.9998,6.943,1.2019,-19.5143},
        {6.9,1.5623,2.0003,6.9283,1.1744,-21.9554},
        {6.92,1.6023,1.9997,6.9124,1.1466,-24.4305},
        {6.94,1.6423,2.0003,6.8952,1.1185,-26.9403},
        {6.96,1.6823,1.9996,6.8768,1.0904,-29.4895},
        {6.98,1.7164,1.7065,6.8572,1.0622,-32.0717},
        {7,1.7259,0.4724,6.8367,1.0344,-34.653},
        {7.02,1.7281,0.1131,6.8154,1.0072,-37.2276},
        {7.04,1.7254,-0.1366,6.7933,0.9807,-39.7929},
        {7.06,1.7179,-0.3755,6.7706,0.955,-42.345},
        {7.08,1.7059,-0.5993,6.7472,0.9301,-44.8802},
        {7.1,1.6898,-0.8037,6.7234,0.9061,-47.3984},
        {7.12,1.6701,-0.987,6.6992,0.8831,-49.8951},
        {7.14,1.6472,-1.1457,6.6745,0.8612,-52.3701},
        {7.16,1.6216,-1.2799,6.6497,0.8405,-54.8216},
        {7.18,1.5938,-1.3884,6.6245,0.8208,-57.2487},
        {7.2,1.5644,-1.4716,6.5993,0.8024,-59.6513},
        {7.22,1.5338,-1.5309,6.5739,0.7852,-62.0276},
        {7.24,1.5025,-1.5658,6.5484,0.7692,-64.3786},
        {7.26,1.4709,-1.579,6.5229,0.7545,-66.7041},
        {7.28,1.4395,-1.5713,6.4975,0.7411,-69.0053},
        {7.3,1.4086,-1.5447,6.472,0.729,-71.2829},
        {7.32,1.3785,-1.5011,6.4467,0.7181,-73.537},
        {7.34,1.3497,-1.4416,6.4214,0.7086,-75.7686},
        {7.36,1.3224,-1.3676,6.3963,0.7004,-77.9794},
        {7.38,1.291,-1.5679,6.3714,0.6935,-80.1612},
        {7.4,1.2511,-1.9942,6.347,0.6881,-82.295},
        {7.42,1.2111,-1.9999,6.3231,0.6839,-84.378},
        {7.44,1.1711,-2.0003,6.2999,0.6811,-86.4056},
        {7.46,1.1311,-1.9993,6.2773,0.6795,-88.3758},
        {7.48,1.1048,-1.3136,6.2552,0.6792,-90.4229},
        {7.5,1.1239,0.9535,6.2328,0.6804,-92.537},
        {7.52,1.1496,1.2853,6.21,0.6832,-94.6637},
        {7.54,1.1787,1.4533,6.1868,0.6874,-96.8039},
        {7.56,1.2108,1.6074,6.1633,0.6932,-98.9585},
        {7.58,1.2457,1.7448,6.1394,0.7004,-101.1302},
        {7.6,1.283,1.8638,6.1152,0.709,-103.3217},
        {7.62,1.3222,1.96,6.0908,0.7191,-105.5348},
        {7.64,1.3622,1.9988,6.0661,0.7306,-107.7704},
        {7.66,1.4022,2.0004,6.0412,0.7434,-110.0276},
        {7.68,1.4422,1.9996,6.0161,0.7577,-112.3091},
        {7.7,1.4822,2.0002,5.9909,0.7733,-114.6149},
        {7.72,1.5222,2.0002,5.9656,0.7902,-116.9468},
        {7.74,1.5622,1.9993,5.9403,0.8085,-119.3093},
        {7.76,1.6022,2.0003,5.9149,0.8281,-121.7024},
        {7.78,1.638,1.7907,5.8897,0.8489,-124.1252},
        {7.8,1.6673,1.4637,5.8646,0.8709,-126.5723},
        {7.82,1.6917,1.2197,5.8399,0.894,-129.0392},
        {7.84,1.7104,0.9387,5.8155,0.9181,-131.525},
        {7.86,1.7179,0.3717,5.7918,0.9429,-134.0171},
        {7.88,1.6851,-1.6364,5.7693,0.968,-136.4633},
        {7.9,1.6451,-1.9998,5.748,0.993,-138.8591},
        {7.92,1.6051,-2.0002,5.7278,1.0181,-141.2081},
        {7.94,1.5651,-1.9997,5.7089,1.043,-143.5157},
        {7.96,1.5251,-2.0005,5.6911,1.0677,-145.7837},
        {7.98,1.4851,-2,5.6744,1.0923,-148.0166},
        {8,1.4451,-1.9997,5.6588,1.1167,-150.218},
        {8.02,1.4051,-2.0005,5.6443,1.1407,-152.3888},
        {8.04,1.3651,-1.9996,5.631,1.1646,-154.5335},
        {8.06,1.3251,-2.0002,5.6187,1.188,-156.653},
        {8.08,1.2851,-2.0002,5.6075,1.2112,-158.7491},
        {8.1,1.2451,-1.9997,5.5974,1.2339,-160.8245},
        {8.12,1.2051,-2.0004,5.5884,1.2563,-162.8792},
        {8.14,1.1651,-1.9996,5.5805,1.2782,-164.9159},
        {8.16,1.1251,-1.9999,5.5736,1.2996,-166.9346},
        {8.18,1.0851,-2,5.5679,1.3206,-168.9353},
        {8.2,1.0451,-2.0003,5.5633,1.341,-170.9171},
        {8.22,1.0051,-1.9997,5.5598,1.3607,-172.88},
        {8.24,0.9651,-2.0001,5.5574,1.3799,-174.8213},
        {8.26,0.9251,-1.9999,5.5561,1.3984,-176.7383},
        {8.28,0.8851,-2,5.556,1.4161,-178.6265},
        {8.3,0.8707,-0.7198,5.5578,1.4333,-179.7903},
        {8.32,0.9092,1.9219,5.5626,1.4509,179.321},
        {8.34,0.9492,2.0001,5.5696,1.4685,178.5137},
        {8.36,0.9892,1.9997,5.5782,1.4863,177.7644},
        {8.38,1.0292,1.9995,5.5882,1.5043,177.0573},
        {8.4,1.0692,2.0007,5.5993,1.5226,176.383},
        {8.42,1.1092,1.9987,5.6115,1.5411,175.7325},
        {8.44,1.1492,2.0006,5.6246,1.56,175.1019},
        {8.46,1.1892,1.9997,5.6386,1.5792,174.4864},
        {8.48,1.2292,2.0002,5.6535,1.5988,173.8833},
        {8.5,1.2692,1.9996,5.6691,1.6188,173.2899},
        {8.52,1.3092,2.0005,5.6856,1.6391,172.7048},
        {8.54,1.3492,1.9993,5.7028,1.6599,172.1257},
        {8.56,1.3892,2,5.7208,1.681,171.5516},
        {8.58,1.4292,2.0003,5.7395,1.7026,170.9817},
        {8.6,1.4692,1.9995,5.759,1.7247,170.4145},
        {8.62,1.5092,2.0002,5.7791,1.7471,169.8496},
        {8.64,1.5491,1.9998,5.8,1.77,169.2861},
        {8.66,1.5892,1.9993,5.8216,1.7934,168.7231},
        {8.68,1.6292,2.0011,5.8438,1.8172,168.1609},
        {8.7,1.6692,1.9988,5.8668,1.8414,167.5979},
        {8.72,1.7092,2.0004,5.8904,1.8661,167.0344},
        {8.74,1.7492,2.0004,5.9147,1.8913,166.47},
        {8.76,1.7891,1.9995,5.9397,1.9169,165.9037},
        {8.78,1.8292,1.9994,5.9653,1.943,165.3352},
        {8.8,1.8692,2.0009,5.9916,1.9696,164.7648},
        {8.82,1.9091,1.9999,6.0186,1.9966,164.1916},
        {8.84,1.9492,1.9989,6.0462,2.0241,163.6148},
        {8.86,1.9892,2.0011,6.0745,2.0521,163.0352},
        {8.88,2.0291,1.9993,6.1034,2.0806,162.4514},
        {8.9,2.0692,1.9999,6.133,2.1096,161.8635},
        {8.92,2.1091,2.0005,6.1631,2.139,161.2715},
        {8.94,2.1491,1.9996,6.194,2.1689,160.6744},
        {8.96,2.1891,2.0002,6.2255,2.1993,160.0723},
        {8.98,2.2291,1.9993,6.2576,2.2303,159.4642},
        {9,2.2691,2.0007,6.2903,2.2617,158.8505},
        {9.02,2.3091,1.9991,6.3237,2.2936,158.23},
        {9.04,2.3491,2.0005,6.3577,2.326,157.603},
        {9.06,2.3891,1.9998,6.3924,2.359,156.9687},
        {9.08,2.4291,1.9998,6.4276,2.3924,156.3265},
        {9.1,2.4691,1.9999,6.4635,2.4264,155.6761},
        {9.12,2.5091,2.0001,6.4999,2.4609,155.0169},
        {9.14,2.5491,2.0003,6.5369,2.4959,154.3485},
        {9.16,2.5891,1.9992,6.5746,2.5314,153.6696},
        {9.18,2.6291,1.9997,6.6129,2.5675,152.9796},
        {9.2,2.6629,1.6913,6.6516,2.604,152.2799},
        {9.22,2.6377,-1.2636,6.69,2.6402,151.5848},
        {9.24,2.5977,-2,6.7278,2.6759,150.8976},
        {9.26,2.5577,-2.0003,6.765,2.711,150.2177},
        {9.28,2.5177,-1.9999,6.8016,2.7456,149.5443},
        {9.3,2.4777,-2.0001,6.8376,2.7796,148.8768},
        {9.32,2.4377,-1.9996,6.8729,2.8132,148.2144},
        {9.34,2.3977,-2.0011,6.9077,2.8462,147.5575},
        {9.36,2.3577,-1.9992,6.9418,2.8788,146.9043},
        {9.38,2.3177,-2,6.9753,2.9108,146.2548},
        {9.4,2.2777,-2.0009,7.0082,2.9423,145.609},
        {9.42,2.2377,-1.9996,7.0404,2.9734,144.9654},
        {9.44,2.1977,-1.9998,7.072,3.0039,144.3237},
        {9.46,2.1577,-2.0007,7.103,3.034,143.6839},
        {9.48,2.1177,-1.9995,7.1333,3.0636,143.0445},
        {9.5,2.0777,-1.9999,7.1629,3.0927,142.4051},
        {9.52,2.0377,-2.001,7.1919,3.1214,141.7657},
        {9.54,1.9977,-1.9993,7.2202,3.1496,141.1244},
        {9.56,1.9577,-2,7.2478,3.1774,140.4809},
        {9.58,1.9176,-2,7.2747,3.2047,139.8341},
        {9.6,1.8776,-2.0003,7.301,3.2316,139.1832},
        {9.62,1.8377,-2.0008,7.3265,3.258,138.5273},
        {9.64,1.7977,-1.9988,7.3513,3.284,137.8635},
        {9.66,1.7576,-2.0007,7.3753,3.3097,137.1914},
        {9.68,1.7176,-2.0003,7.3986,3.3349,136.5088},
        {9.7,1.6777,-2,7.4212,3.3597,135.8128},
        {9.72,1.6376,-1.9993,7.4429,3.3843,135.0998},
        {9.74,1.5977,-2.0004,7.4641,3.4081,133.797},
        {9.76,1.5577,-1.9991,7.4855,3.4308,132.3802},
        {9.78,1.5177,-2.0005,7.507,3.4522,130.9423},
        {9.8,1.4777,-1.9993,7.5286,3.4724,129.4815},
        {9.82,1.4377,-2.0004,7.5503,3.4912,127.9996},
        {9.84,1.3977,-1.9994,7.5722,3.5087,126.4957},
        {9.86,1.3577,-2.0004,7.594,3.5248,124.9724},
        {9.88,1.3177,-1.9997,7.616,3.5394,123.4306},
        {9.9,1.2787,-1.9471,7.6379,3.5526,121.8713},
        {9.92,1.2435,-1.7601,7.6598,3.5643,120.2934},
        {9.94,1.2125,-1.5507,7.6818,3.5745,118.6954},
        {9.96,1.1857,-1.3403,7.7038,3.5833,117.0744},
        {9.98,1.1631,-1.1297,7.7259,3.5906,115.4297},
        {10,1.1448,-0.9184,7.748,3.5964,113.7612},
        {10.02,1.1306,-0.7056,7.7702,3.6007,112.0672},
        {10.04,1.1208,-0.4921,7.7925,3.6035,110.3477},
        {10.06,1.1152,-0.2776,7.8148,3.6047,108.6035},
        {10.08,1.114,-0.062,7.837,3.6044,106.8356},
        {10.1,1.1171,0.1547,7.8593,3.6025,105.043},
        {10.12,1.1246,0.373,7.8815,3.5991,103.2285},
        {10.14,1.1364,0.5923,7.9037,3.5942,101.3893},
        {10.16,1.1527,0.8146,7.9258,3.5878,99.5272},
        {10.18,1.1735,1.0391,7.9479,3.5799,97.6405},
        {10.2,1.1988,1.2672,7.97,3.5705,95.7256},
        {10.22,1.2289,1.5016,7.992,3.5597,93.7808},
        {10.24,1.2637,1.7431,8.0141,3.5473,91.8017},
        {10.26,1.3028,1.9541,8.0362,3.5335,89.7838},
        {10.28,1.3428,2.0003,8.0583,3.5182,87.729},
        {10.3,1.3828,2.0002,8.0803,3.5016,85.6382},
        {10.32,1.4228,1.9998,8.1024,3.4836,83.5103},
        {10.34,1.4628,1.9999,8.1243,3.4642,81.3446},
        {10.36,1.5028,2,8.1462,3.4436,79.1394},
        {10.38,1.5428,2.0002,8.1681,3.4219,76.8918},
        {10.4,1.5828,1.9996,8.1899,3.3989,74.5968},
        {10.42,1.6082,1.2678,8.2114,3.375,72.271},
        {10.44,1.5744,-1.6882,8.2321,3.3512,69.9935},
        {10.46,1.5344,-2.0001,8.2517,3.3277,67.7671},
        {10.48,1.4944,-2.0003,8.2706,3.3045,65.5865},
        {10.5,1.4544,-1.9996,8.2886,3.2816,63.4446},
        {10.52,1.4144,-2.0003,8.3059,3.2593,61.3378},
        {10.54,1.3744,-2.0002,8.3226,3.2374,59.261},
        {10.56,1.3344,-1.9999,8.3386,3.2161,57.2089},
        {10.58,1.2944,-1.9997,8.3541,3.1953,55.1761},
        {10.6,1.2544,-2.0004,8.3691,3.1752,53.1591},
        {10.62,1.2144,-2.0003,8.3836,3.1558,51.1527},
        {10.64,1.1744,-1.9995,8.3977,3.137,49.1498},
        {10.66,1.1344,-2.0004,8.4115,3.119,47.1461},
        {10.68,1.1089,-1.2752,8.4245,3.101,44.796},
        {10.7,1.1051,-0.1899,8.4361,3.0822,42.3845},
        {10.72,1.1051,0.0013,8.4464,3.0626,39.9384},
        {10.74,1.1089,0.1888,8.4553,3.0423,37.4596},
        {10.76,1.1163,0.373,8.4629,3.0213,34.9471},
        {10.78,1.1274,0.553,8.469,2.9997,32.4027},
        {10.8,1.1419,0.7271,8.4738,2.9773,29.8247},
        {10.82,1.1598,0.8945,8.4772,2.9544,27.213},
        {10.84,1.1809,1.053,8.4793,2.9308,24.5658},
        {10.86,1.2049,1.2008,8.4799,2.9068,21.8804},
        {10.88,1.2316,1.3356,8.4793,2.8821,19.154},
        {10.9,1.2607,1.4543,8.4773,2.857,16.3821},
        {10.92,1.2918,1.554,8.474,2.8314,13.5611},
        {10.94,1.3244,1.6299,8.4694,2.8053,10.6864},
        {10.96,1.3579,1.6773,8.4635,2.7788,7.7535},
        {10.98,1.3917,1.6897,8.4563,2.7519,4.7578},
        {11,1.4249,1.6599,8.4478,2.7247,1.6956},
        {11.02,1.4565,1.5781,8.4381,2.6973,-1.4384},
        {11.04,1.4852,1.4354,8.4271,2.6697,-4.6471},
        {11.06,1.5096,1.2195,8.4149,2.6421,-7.9331},
        {11.08,1.5167,0.3577,8.4017,2.6147,-11.2728},
        {11.1,1.4804,-1.8195,8.3879,2.5886,-14.5833},
        {11.12,1.4404,-1.9996,8.3736,2.5635,-17.8721},
        {11.14,1.4004,-2.0004,8.359,2.5397,-21.1499},
        {11.16,1.3604,-1.9998,8.344,2.5169,-24.4341},
        {11.18,1.3204,-2.0001,8.3288,2.4954,-27.741},
        {11.2,1.2804,-2.0003,8.3133,2.475,-31.0916},
        {11.22,1.2404,-2,8.2976,2.4558,-34.5142},
        {11.24,1.2004,-2.0001,8.2816,2.4379,-38.045},
        {11.26,1.1603,-2,8.2654,2.4213,-41.7368},
        {11.28,1.1277,-1.633,8.2487,2.406,-44.9055},
        {11.3,1.1536,1.296,8.2314,2.3909,-47.1759},
        {11.32,1.1936,2.0004,8.2132,2.3754,-49.5783},
        {11.34,1.2336,1.9998,8.1941,2.3598,-52.122},
        {11.36,1.2736,1.9999,8.174,2.3441,-54.8148},
        {11.38,1.3136,2.0002,8.1531,2.3283,-57.6646},
        {11.4,1.3536,1.9997,8.1311,2.3125,-60.6822},
        {11.42,1.3936,2.0003,8.108,2.2968,-63.8752},
        {11.44,1.4336,1.9995,8.0838,2.2814,-67.2558},
        {11.46,1.4736,2.0004,8.0585,2.2663,-70.8303},
        {11.48,1.5061,1.6269,8.0322,2.2517,-74.5897},
        {11.5,1.4845,-1.0841,8.0057,2.2383,-78.3979},
        {11.52,1.4564,-1.4044,7.9792,2.2261,-82.2312},
        {11.54,1.4337,-1.1361,7.9527,2.2152,-86.0922},
        {11.56,1.4163,-0.8667,7.9261,2.2054,-89.9875},
        {11.58,1.4044,-0.5945,7.8994,2.1968,-93.9184},
        {11.6,1.3981,-0.3156,7.8725,2.1894,-97.8916},
        {11.62,1.3976,-0.0265,7.8452,2.183,-101.9124},
        {11.64,1.4032,0.2779,7.8177,2.1778,-105.9832},
        {11.66,1.4152,0.6036,7.7897,2.1736,-110.1122},
        {11.68,1.4344,0.959,7.7612,2.1705,-114.3085},
        {11.7,1.4615,1.3561,7.732,2.1685,-118.5813},
        {11.72,1.4973,1.7901,7.7021,2.1676,-122.9413},
        {11.74,1.5372,1.9952,7.6713,2.1677,-127.3857},
        {11.76,1.5772,1.9998,7.6398,2.1688,-131.9028},
        {11.78,1.5919,0.7329,7.608,2.171,-136.4066},
        {11.8,1.5535,-1.9166,7.5771,2.1739,-140.7415},
        {11.82,1.5135,-2.0002,7.547,2.1774,-144.9008},
        {11.84,1.4735,-1.9999,7.5178,2.1814,-148.8872},
        {11.86,1.4335,-1.9997,7.4895,2.1858,-152.7046},
        {11.88,1.3935,-2.0005,7.462,2.1903,-156.3544},
        {11.9,1.3535,-1.9996,7.4354,2.1951,-159.8445},
        {11.92,1.3135,-2.0003,7.4095,2.1999,-163.1775},
        {11.94,1.2735,-2.0002,7.3845,2.2048,-166.3587},
        {11.96,1.2335,-1.9998,7.3603,2.2096,-169.3947},
        {11.98,1.1935,-2.0001,7.337,2.2144,-172.2895},
        {12,1.1535,-1.9999,7.3144,2.219,-175.0483},
        {12.02,1.1302,-1.1665,7.2922,2.2234,-174.0943},
        {12.04,1.1651,1.7422,7.2691,2.2267,-171.338},
        {12.06,1.205,2,7.2451,2.2292,-168.769},
        {12.08,1.245,1.9992,7.2203,2.231,-166.3382},
        {12.1,1.285,2.0007,7.1947,2.2324,-164.0177},
        {12.12,1.325,1.999,7.1682,2.2332,-161.7812},
        {12.14,1.365,1.9999,7.1409,2.2337,-159.6148},
        {12.16,1.405,2.0007,7.1128,2.2338,-157.5078},
        {12.18,1.445,1.9994,7.0839,2.2336,-155.448},
        {12.2,1.485,1.9997,7.0542,2.2331,-153.4285},
        {12.22,1.525,2,7.0237,2.2324,-151.444},
        {12.24,1.565,1.9999,6.9924,2.2313,-149.4892},
        {12.26,1.605,2.0002,6.9603,2.2301,-147.5607},
        {12.28,1.645,1.9992,6.9274,2.2286,-145.6532},
        {12.3,1.685,2.0007,6.8938,2.2269,-143.7667},
        {12.32,1.725,2.0002,6.8594,2.225,-141.8977},
        {12.34,1.765,1.9995,6.8241,2.223,-140.0427},
        {12.36,1.805,1.9995,6.7881,2.2207,-138.2},
        {12.38,1.845,2.0004,6.7513,2.2182,-136.3695},
        {12.4,1.885,2.0002,6.7137,2.2156,-134.5495},
        {12.42,1.925,2,6.6753,2.2129,-132.7382},
        {12.44,1.965,1.9996,6.6361,2.2099,-130.934},
        {12.46,2.005,1.9992,6.5961,2.2068,-129.135},
        {12.48,2.045,2.0017,6.5553,2.2036,-127.3447},
        {12.5,2.085,1.9992,6.5138,2.2002,-125.558},
        {12.52,2.125,1.9996,6.4714,2.1967,-123.7747},
        {12.54,2.165,2,6.4283,2.1931,-121.995},
        {12.56,2.205,2.0003,6.3843,2.1893,-120.2187},
        {12.58,2.245,1.9997,6.3396,2.1854,-118.4442},
        {12.6,2.285,2,6.294,2.1814,-116.6715},
        {12.62,2.325,2.0002,6.2477,2.1772,-114.9005},
        {12.64,2.365,2.0005,6.2006,2.173,-113.1312},
        {12.66,2.405,1.9997,6.1528,2.1687,-111.362},
        {12.68,2.445,1.999,6.104,2.1642,-109.591},
        {12.7,2.485,2.0011,6.0546,2.1596,-107.8217},
        {12.72,2.525,1.9994,6.0043,2.155,-106.0507},
        {12.74,2.565,1.9995,5.9532,2.1502,-104.278},
        {12.76,2.605,2.0007,5.9013,2.1454,-102.5052},
        {12.78,2.645,1.9999,5.8486,2.1405,-100.7307},
        {12.8,2.6847,1.9844,5.7952,2.1355,-98.9545},
        {12.82,2.7,0.7665,5.7414,2.1305,-97.1922},
        {12.84,2.7,0,5.6876,2.1255,-95.4527},
        {12.86,2.7,0,5.6339,2.1204,-93.7342},
        {12.88,2.7,0,5.5801,2.1154,-92.0367},
        {12.9,2.7,0,5.5264,2.1104,-90.3602},
        {12.92,2.7,0,5.4726,2.1055,-88.7012},
        {12.94,2.7,0,5.4188,2.1005,-87.0597},
        {12.96,2.7,0,5.365,2.0956,-85.434},
        {12.98,2.7,0,5.3113,2.0906,-83.8275},
        {13,2.7,0,5.2575,2.0857,-82.2332},
        {13.02,2.7,0,5.2037,2.0809,-80.6565},
        {13.04,2.7,0,5.1499,2.076,-79.092},
        {13.06,2.7,0,5.0961,2.0712,-77.5432},
        {13.08,2.7,0,5.0423,2.0665,-76.0067},
        {13.1,2.7,0,4.9885,2.0617,-74.4825},
        {13.12,2.7,0,4.9347,2.057,-72.9705},
        {13.14,2.7,0,4.881,2.0523,-71.4708},
        {13.16,2.7,0,4.8272,2.0477,-69.9815},
        {13.18,2.7,0,4.7733,2.0431,-68.5028},
        {13.2,2.7,0,4.7196,2.0386,-67.0363},
        {13.22,2.7,0,4.6657,2.0341,-65.5785},
        {13.24,2.7,0,4.6119,2.0296,-64.1295},
        {13.26,2.7,0,4.5581,2.0252,-62.691},
        {13.28,2.7,0,4.5043,2.0209,-61.263},
        {13.3,2.7,0,4.4505,2.0166,-59.842},
        {13.32,2.7,0,4.3966,2.0123,-58.4298},
        {13.34,2.7,0,4.3427,2.0081,-57.0245},
        {13.36,2.7,0,4.2889,2.0039,-55.628},
        {13.38,2.7,0,4.2351,1.9998,-54.2403},
        {13.4,2.7,0,4.1813,1.9958,-52.8595},
        {13.42,2.7,0,4.1274,1.9918,-51.4858},
        {13.44,2.7,0,4.0735,1.9879,-50.1173},
        {13.46,2.7,0,4.0197,1.984,-48.7575},
        {13.48,2.7,0,3.9658,1.9802,-47.403},
        {13.5,2.7,0,3.9119,1.9764,-46.0555},
        {13.52,2.7,0,3.8581,1.9727,-44.715},
        {13.54,2.6992,-0.0394,3.8042,1.9691,-43.3798},
        {13.56,2.6721,-1.3577,3.7509,1.9655,-42.0638},
        {13.58,2.6321,-1.9996,3.6983,1.9621,-40.7723},
        {13.6,2.5921,-1.9999,3.6466,1.9588,-39.5053},
        {13.62,2.552,-2.0002,3.5956,1.9556,-38.2628},
        {13.64,2.512,-2.0005,3.5455,1.9525,-37.0448},
        {13.66,2.472,-1.9994,3.4961,1.9495,-35.8495},
        {13.68,2.432,-2.0011,3.4476,1.9467,-34.6788},
        {13.7,2.3921,-1.9999,3.3999,1.9439,-33.5308},
        {13.72,2.3521,-1.9985,3.3529,1.9412,-32.4038},
        {13.74,2.312,-2.0019,3.3067,1.9386,-31.3013},
        {13.76,2.2721,-1.999,3.2613,1.9362,-30.2198},
        {13.78,2.2321,-2.0008,3.2168,1.9338,-29.161},
        {13.8,2.1921,-1.9994,3.173,1.9315,-28.1233},
        {13.82,2.1521,-1.9995,3.13,1.9293,-27.1065},
        {13.84,2.112,-1.9997,3.0878,1.9271,-26.1108},
        {13.86,2.072,-2.0017,3.0464,1.9251,-25.1378},
        {13.88,2.032,-1.9983,3.0058,1.9232,-24.184},
        {13.9,1.992,-2.0022,2.966,1.9213,-23.253},
        {13.92,1.9521,-1.9986,2.927,1.9195,-22.3413},
        {13.94,1.9121,-2.0007,2.8889,1.9178,-21.4505},
        {13.96,1.8721,-1.9988,2.8514,1.9161,-20.579},
        {13.98,1.832,-2.001,2.8148,1.9146,-19.7285},
        {14,1.792,-1.9991,2.779,1.9131,-18.8973},
        {14.02,1.752,-2.0014,2.744,1.9117,-18.087},
        {14.04,1.7121,-1.9994,2.7098,1.9103,-17.296},
        {14.06,1.6721,-1.9995,2.6764,1.909,-16.5243},
        {14.08,1.632,-1.9996,2.6437,1.9077,-15.7718},
        {14.1,1.592,-2.0021,2.6119,1.9066,-15.0403},
        {14.12,1.552,-1.9975,2.5809,1.9054,-14.3263},
        {14.14,1.512,-2.0025,2.5507,1.9044,-13.6333},
        {14.16,1.472,-1.9976,2.5212,1.9034,-12.9578},
        {14.18,1.432,-2.0029,2.4926,1.9024,-12.3033},
        {14.2,1.392,-1.9977,2.4648,1.9015,-11.6663},
        {14.22,1.352,-2.0005,2.4378,1.9006,-11.0485},
        {14.24,1.312,-2.0006,2.4115,1.8998,-10.45},
        {14.26,1.272,-2.0008,2.3861,1.8991,-9.8708},
        {14.28,1.232,-1.9978,2.3615,1.8983,-9.309},
        {14.3,1.192,-2.001,2.3376,1.8977,-8.7665},
        {14.32,1.152,-2.0012,2.3146,1.897,-8.2433},
        {14.34,1.112,-1.9979,2.2924,1.8964,-7.7375},
        {14.36,1.072,-2.0014,2.2709,1.8958,-7.251},
        {14.3801,1.032,-1.9979,2.2503,1.8953,-6.782},
        {14.4001,0.9919,-2.0017,2.2304,1.8948,-6.3323},
        {14.42,0.9519,-2.002,2.2114,1.8943,-5.9018},
        {14.4401,0.912,-1.998,2.1932,1.8939,-5.4888},
        {14.46,0.872,-2.0024,2.1758,1.8935,-5.095},
        {14.48,0.8321,-1.998,2.1592,1.8931,-4.7188},
        {14.5001,0.792,-1.9978,2.1433,1.8927,-4.36},
        {14.52,0.752,-2.0028,2.1283,1.8924,-4.0205},
        {14.5401,0.712,-1.9978,2.114,1.8921,-3.6985},
        {14.56,0.672,-2.0034,2.1006,1.8918,-3.3958},
        {14.58,0.632,-1.9978,2.088,1.8915,-3.1105},
        {14.6001,0.5919,-1.9975,2.0761,1.8913,-2.8428},
        {14.6201,0.5519,-2.0043,2.0651,1.8911,-2.5943},
        {14.6401,0.5119,-1.9975,2.0548,1.8909,-2.3633},
        {14.66,0.4719,-2.0055,2.0454,1.8907,-2.1515},
        {14.6802,0.4319,-1.9884,2.0367,1.8905,-1.9555},
        {14.7,0.3919,-2.0161,2.0289,1.8904,-1.7805},
        {14.7201,0.352,-1.9867,2.0219,1.8903,-1.6213},
        {14.74,0.3119,-2.0079,2.0156,1.8901,-1.4813},
        {14.7601,0.272,-1.9963,2.0102,1.89,-1.3588},
        {14.7802,0.2318,-1.9943,2.0055,1.89,-1.2538},
        {14.8001,0.1918,-2.0131,2.0017,1.8899,-1.168},
        {14.82,0.152,-1.9954,1.9987,1.8898,-1.0998},
        {14.8402,0.1118,-1.9896,1.9964,1.8898,-1.049},
        {14.8611,0.0708,-1.9659,1.995,1.8898,-1.0158},
        {14.886,0.0249,-1.8371,1.9943,1.8898,-1.0018}
    };