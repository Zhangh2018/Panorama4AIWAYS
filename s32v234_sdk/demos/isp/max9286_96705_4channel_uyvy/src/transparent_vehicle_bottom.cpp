#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/time.h>
#include "transparent_vehicle_bottom.h"
///#include "ImageStitching.h"
#include "math.h"
#include "data_type.h"


#define START_STEERING_WHEEL_ANGLE	(-529)
#define STEERING_WHEEL_ANGLE_NUM	(1062)

static const Float64_t turning_angle_array[1062] = {
	-34.77253732, -34.69505346, -34.61765368, -34.54033764, -34.46310501, -34.38595548, -34.30888870, -34.23190436, -34.15500214, -34.07818172,
	-34.00144276, -33.92478496, -33.84820800, -33.77171154, -33.69529528, -33.61895891, -33.54270210, -33.46652454, -33.39042592, -33.31440592,
	-33.23846424, -33.16260056, -33.08681457, -33.01110597, -32.93547444, -32.85991969, -32.78444139, -32.70903926, -32.63371298, -32.55846226,
	-32.48328678, -32.40818625, -32.33316037, -32.25820884, -32.18333136, -32.10852764, -32.03379737, -31.95914026, -31.88455602, -31.81004435,
	-31.73560496, -31.66123756, -31.58694186, -31.51271756, -31.43856438, -31.36448203, -31.29047022, -31.21652866, -31.14265708, -31.06885518,
	-30.99512269, -30.92145931, -30.84786476, -30.77433878, -30.70088107, -30.62749135, -30.55416935, -30.48091479, -30.40772740, -30.33460689,
	-30.26155299, -30.18856544, -30.11564395, -30.04278825, -29.96999807, -29.89727314, -29.82461320, -29.75201796, -29.67948718, -29.60702057,
	-29.53461787, -29.46227881, -29.39000314, -29.31779059, -29.24564089, -29.17355378, -29.10152901, -29.02956630, -28.95766541, -28.88582607,
	-28.81404803, -28.74233103, -28.67067481, -28.59907911, -28.52754369, -28.45606829, -28.38465266, -28.31329654, -28.24199968, -28.17076184,
	-28.09958276, -28.02846220, -27.95739990, -27.88639563, -27.81544913, -27.74456016, -27.67372847, -27.60295382, -27.53223597, -27.46157467,
	-27.39096969, -27.32042078, -27.24992771, -27.17949023, -27.10910811, -27.03878110, -26.96850898, -26.89829150, -26.82812844, -26.75801955,
	-26.68796461, -26.61796338, -26.54801562, -26.47812112, -26.40827963, -26.33849093, -26.26875479, -26.19907098, -26.12943927, -26.05985945,
	-25.99033127, -25.92085452, -25.85142897, -25.78205440, -25.71273058, -25.64345730, -25.57423433, -25.50506146, -25.43593845, -25.36686510,
	-25.29784119, -25.22886649, -25.15994079, -25.09106388, -25.02223554, -24.95345555, -24.88472370, -24.81603978, -24.74740357, -24.67881487,
	-24.61027346, -24.54177914, -24.47333168, -24.40493089, -24.33657656, -24.26826847, -24.20000643, -24.13179022, -24.06361964, -23.99549449,
	-23.92741456, -23.85937965, -23.79138956, -23.72344408, -23.65554301, -23.58768616, -23.51987332, -23.45210430, -23.38437889, -23.31669691,
	-23.24905814, -23.18146240, -23.11390949, -23.04639921, -22.97893138, -22.91150579, -22.84412225, -22.77678058, -22.70948058, -22.64222206,
	-22.57500483, -22.50782870, -22.44069348, -22.37359898, -22.30654502, -22.23953140, -22.17255795, -22.10562448, -22.03873079, -21.97187672,
	-21.90506206, -21.83828665, -21.77155030, -21.70485282, -21.63819403, -21.57157376, -21.50499183, -21.43844805, -21.37194225, -21.30547424,
	-21.23904386, -21.17265093, -21.10629526, -21.03997668, -20.97369502, -20.90745011, -20.84124177, -20.77506982, -20.70893410, -20.64283444,
	-20.57677065, -20.51074258, -20.44475004, -20.37879288, -20.31287092, -20.24698400, -20.18113194, -20.11531458, -20.04953176, -19.98378330,
	-19.91806905, -19.85238884, -19.78674249, -19.72112986, -19.65555078, -19.59000508, -19.52449260, -19.45901318, -19.39356667, -19.32815289,
	-19.26277170, -19.19742293, -19.13210642, -19.06682202, -19.00156957, -18.93634891, -18.87115989, -18.80600234, -18.74087613, -18.67578108,
	-18.61071705, -18.54568388, -18.48068142, -18.41570952, -18.35076803, -18.28585679, -18.22097565, -18.15612447, -18.09130309, -18.02651137,
	-17.96174915, -17.89701630, -17.83231265, -17.76763806, -17.70299240, -17.63837550, -17.57378723, -17.50922744, -17.44469599, -17.38019272,
	-17.31571751, -17.25127020, -17.18685066, -17.12245873, -17.05809429, -16.99375718, -16.92944727, -16.86516442, -16.80090849, -16.73667934,
	-16.67247683, -16.60830082, -16.54415118, -16.48002776, -16.41593044, -16.35185908, -16.28781353, -16.22379367, -16.15979936, -16.09583046,
	-16.03188685, -15.96796839, -15.90407494, -15.84020638, -15.77636257, -15.71254338, -15.64874867, -15.58497833, -15.52123222, -15.45751020,
	-15.39381216, -15.33013796, -15.26648747, -15.20286056, -15.13925712, -15.07567700, -15.01212009, -14.94858626, -14.88507538, -14.82158733,
	-14.75812198, -14.69467920, -14.63125889, -14.56786090, -14.50448512, -14.44113142, -14.37779969, -14.31448980, -14.25120163, -14.18793506,
	-14.12468996, -14.06146622, -13.99826372, -13.93508234, -13.87192196, -13.80878245, -13.74566371, -13.68256562, -13.61948805, -13.55643089,
	-13.49339403, -13.43037734, -13.36738071, -13.30440403, -13.24144719, -13.17851005, -13.11559252, -13.05269448, -12.98981581, -12.92695640,
	-12.86411614, -12.80129492, -12.73849262, -12.67570913, -12.61294434, -12.55019814, -12.48747042, -12.42476107, -12.36206998, -12.29939704,
	-12.23674213, -12.17410516, -12.11148601, -12.04888457, -11.98630074, -11.92373440, -11.86118546, -11.79865381, -11.73613933, -11.67364192,
	-11.61116148, -11.54869790, -11.48625107, -11.42382090, -11.36140727, -11.29901008, -11.23662923, -11.17426462, -11.11191613, -11.04958367,
	-10.98726714, -10.92496643, -10.86268144, -10.80041208, -10.73815822, -10.67591979, -10.61369667, -10.55148877, -10.48929598, -10.42711820,
	-10.36495534, -10.30280730, -10.24067398, -10.17855527, -10.11645108, -10.05436131, -9.992285871, -9.930224654, -9.868177565, -9.806144508,
	-9.744125385, -9.682120101, -9.620128558, -9.558150661, -9.496186313, -9.434235421, -9.372297887, -9.310373617, -9.248462517, -9.186564492,
	-9.124679448, -9.062807291, -9.000947927, -8.939101262, -8.877267204, -8.815445659, -8.753636535, -8.691839739, -8.630055179, -8.568282763,
	-8.506522399, -8.444773996, -8.383037462, -8.321312705, -8.259599637, -8.197898164, -8.136208198, -8.074529648, -8.012862424, -7.951206435,
	-7.889561594, -7.827927809, -7.766304993, -7.704693056, -7.643091909, -7.581501464, -7.519921632, -7.458352326, -7.396793458, -7.335244939,
	-7.273706682, -7.212178600, -7.150660606, -7.089152613, -7.027654533, -6.966166281, -6.904687770, -6.843218913, -6.781759626, -6.720309820,
	-6.658869412, -6.597438316, -6.536016445, -6.474603716, -6.413200043, -6.351805340, -6.290419524, -6.229042510, -6.167674213, -6.106314550,
	-6.044963436, -5.983620787, -5.922286519, -5.860960550, -5.799642794, -5.738333170, -5.677031594, -5.615737983, -5.554452254, -5.493174324,
	-5.431904111, -5.370641531, -5.309386504, -5.248138946, -5.186898776, -5.125665911, -5.064440270, -5.003221770, -4.942010331, -4.880805871,
	-4.819608308, -4.758417561, -4.697233549, -4.636056192, -4.574885407, -4.513721115, -4.452563234, -4.391411684, -4.330266384, -4.269127254,
	-4.207994214, -4.146867184, -4.085746082, -4.024630830, -3.963521347, -3.902417554, -3.841319370, -3.780226716, -3.719139513, -3.658057680,
	-3.596981138, -3.535909808, -3.474843611, -3.413782468, -3.352726298, -3.291675024, -3.230628565, -3.169586844, -3.108549780, -3.047517296,
	-2.986489312, -2.925465750, -2.864446532, -2.803431577, -2.742420808, -2.681414146, -2.620411514, -2.559412831, -2.498418020, -2.437427003,
	-2.376439701, -2.315456035, -2.254475929, -2.193499302, -2.132526078, -2.071556178, -2.010589523, -1.949626036, -1.888665639, -1.827708253,
	-1.766753801, -1.705802204, -1.644853384, -1.583907264, -1.522963764, -1.462022809, -1.401084318, -1.340148215, -1.279214421, -1.218282858,
	-1.157353449, -1.096426115, -1.035500778, -0.974577361, -0.913655785, -0.852735972, -0.791817845, -0.730901325, -0.669986333, -0.609072793,
	-0.548160626, -0.487249754, -0.426340098, -0.365431580, -0.304524123, -0.243617648, -0.182712076, -0.121807330, -0.060903331, 0,
	0.060902740, 0.121804969, 0.182706764, 0.243608205, 0.304509370, 0.365410337, 0.426311186, 0.487211994, 0.548112842, 0.609013807,
	0.669914970, 0.730816407, 0.791718200, 0.852620427, 0.913523166, 0.974426498, 1.035330502, 1.096235257, 1.157140843, 1.218047338,
	1.278954824, 1.339863379, 1.400773084, 1.461684018, 1.522596261, 1.583509893, 1.644424995, 1.705341647, 1.766259929, 1.827179921,
	1.888101705, 1.949025360, 2.009950968, 2.070878609, 2.131808365, 2.192740316, 2.253674543, 2.314611129, 2.375550153, 2.436491699,
	2.497435847, 2.558382679, 2.619332278, 2.680284724, 2.741240101, 2.802198491, 2.863159975, 2.924124637, 2.985092559, 3.046063824,
	3.107038515, 3.168016715, 3.228998507, 3.289983975, 3.350973202, 3.411966272, 3.472963268, 3.533964274, 3.594969376, 3.655978656,
	3.716992199, 3.778010091, 3.839032415, 3.900059257, 3.961090701, 4.022126834, 4.083167739, 4.144213504, 4.205264213, 4.266319953,
	4.327380810, 4.388446869, 4.449518218, 4.510594944, 4.571677132, 4.632764871, 4.693858246, 4.754957346, 4.816062259, 4.877173072,
	4.938289872, 4.999412749, 5.060541791, 5.121677085, 5.182818722, 5.243966790, 5.305121378, 5.366282576, 5.427450473, 5.488625160,
	5.549806725, 5.610995260, 5.672190855, 5.733393601, 5.794603589, 5.855820909, 5.917045654, 5.978277915, 6.039517784, 6.100765352,
	6.162020713, 6.223283959, 6.284555182, 6.345834476, 6.407121934, 6.468417650, 6.529721717, 6.591034229, 6.652355282, 6.713684968,
	6.775023384, 6.836370623, 6.897726782, 6.959091957, 7.020466242, 7.081849734, 7.143242529, 7.204644725, 7.266056417, 7.327477704,
	7.388908682, 7.450349450, 7.511800105, 7.573260746, 7.634731471, 7.696212379, 7.757703569, 7.819205142, 7.880717196, 7.942239831,
	8.003773148, 8.065317248, 8.126872231, 8.188438199, 8.250015253, 8.311603494, 8.373203026, 8.434813951, 8.496436371, 8.558070389,
	8.619716109, 8.681373635, 8.743043069, 8.804724518, 8.866418085, 8.928123876, 8.989841995, 9.051572549, 9.113315643, 9.175071384,
	9.236839878, 9.298621232, 9.360415554, 9.422222951, 9.484043531, 9.545877402, 9.607724674, 9.669585454, 9.731459854, 9.793347981,
	9.855249947, 9.917165862, 9.979095836, 10.04103998, 10.10299841, 10.16497123, 10.22695856, 10.28896050, 10.35097718, 10.41300870,
	10.47505519, 10.53711674, 10.59919348, 10.66128553, 10.72339299, 10.78551598, 10.84765463, 10.90980904, 10.97197933, 11.03416562,
	11.09636803, 11.15858667, 11.22082167, 11.28307314, 11.34534120, 11.40762597, 11.46992757, 11.53224612, 11.59458174, 11.65693456,
	11.71930469, 11.78169226, 11.84409738, 11.90652019, 11.96896080, 12.03141934, 12.09389594, 12.15639071, 12.21890378, 12.28143528,
	12.34398533, 12.40655406, 12.46914160, 12.53174808, 12.59437361, 12.65701833, 12.71968237, 12.78236585, 12.84506891, 12.90779168,
	12.97053428, 13.03329685, 13.09607951, 13.15888240, 13.22170566, 13.28454941, 13.34741378, 13.41029891, 13.47320494, 13.53613200,
	13.59908021, 13.66204973, 13.72504068, 13.78805320, 13.85108743, 13.91414350, 13.97722155, 14.04032173, 14.10344416, 14.16658899,
	14.22975636, 14.29294641, 14.35615927, 14.41939509, 14.48265402, 14.54593619, 14.60924174, 14.67257082, 14.73592358, 14.79930015,
	14.86270069, 14.92612533, 14.98957422, 15.05304751, 15.11654535, 15.18006788, 15.24361525, 15.30718761, 15.37078511, 15.43440789,
	15.49805611, 15.56172992, 15.62542946, 15.68915490, 15.75290637, 15.81668404, 15.88048806, 15.94431857, 16.00817575, 16.07205973,
	16.13597067, 16.19990874, 16.26387408, 16.32786686, 16.39188723, 16.45593535, 16.52001137, 16.58411547, 16.64824779, 16.71240849,
	16.77659775, 16.84081571, 16.90506255, 16.96933842, 17.03364349, 17.09797792, 17.16234187, 17.22673551, 17.29115901, 17.35561254,
	17.42009624, 17.48461031, 17.54915490, 17.61373018, 17.67833632, 17.74297349, 17.80764186, 17.87234160, 17.93707289, 18.00183589,
	18.06663078, 18.13145773, 18.19631691, 18.26120851, 18.32613268, 18.39108962, 18.45607949, 18.52110248, 18.58615876, 18.65124850,
	18.71637190, 18.78152912, 18.84672035, 18.91194576, 18.97720555, 19.04249988, 19.10782896, 19.17319294, 19.23859203, 19.30402641,
	19.36949625, 19.43500175, 19.50054310, 19.56612047, 19.63173406, 19.69738406, 19.76307066, 19.82879404, 19.89455439, 19.96035192,
	20.02618680, 20.09205923, 20.15796941, 20.22391752, 20.28990377, 20.35592834, 20.42199144, 20.48809326, 20.55423400, 20.62041385,
	20.68663301, 20.75289168, 20.81919007, 20.88552837, 20.95190679, 21.01832552, 21.08478477, 21.15128474, 21.21782564, 21.28440767,
	21.35103104, 21.41769595, 21.48440261, 21.55115123, 21.61794202, 21.68477517, 21.75165092, 21.81856946, 21.88553100, 21.95253577,
	22.01958396, 22.08667580, 22.15381150, 22.22099127, 22.28821532, 22.35548389, 22.42279717, 22.49015540, 22.55755879, 22.62500755,
	22.69250191, 22.76004209, 22.82762831, 22.89526079, 22.96293975, 23.03066543, 23.09843804, 23.16625780, 23.23412496, 23.30203972,
	23.37000232, 23.43801299, 23.50607196, 23.57417945, 23.64233570, 23.71054094, 23.77879540, 23.84709931, 23.91545291, 23.98385644,
	24.05231012, 24.12081419, 24.18936889, 24.25797446, 24.32663114, 24.39533916, 24.46409877, 24.53291020, 24.60177370, 24.67068951,
	24.73965788, 24.80867904, 24.87775325, 24.94688074, 25.01606177, 25.08529658, 25.15458543, 25.22392855, 25.29332619, 25.36277862,
	25.43228608, 25.50184882, 25.57146710, 25.64114117, 25.71087128, 25.78065769, 25.85050066, 25.92040044, 25.99035729, 26.06037147,
	26.13044325, 26.20057287, 26.27076061, 26.34100672, 26.41131146, 26.48167511, 26.55209793, 26.62258017, 26.69312212, 26.76372403,
	26.83438617, 26.90510882, 26.97589224, 27.04673670, 27.11764248, 27.18860985, 27.25963908, 27.33073045, 27.40188422, 27.47310068,
	27.54438011, 27.61572278, 27.68712897, 27.75859896, 27.83013303, 27.90173146, 27.97339454, 28.04512254, 28.11691576, 28.18877447,
	28.26069897, 28.33268953, 28.40474645, 28.47687002, 28.54906052, 28.62131825, 28.69364350, 28.76603655, 28.83849770, 28.91102726,
	28.98362550, 29.05629273, 29.12902924, 29.20183534, 29.27471132, 29.34765747, 29.42067411, 29.49376153, 29.56692003, 29.64014992,
	29.71345151, 29.78682509, 29.86027097, 29.93378947, 30.00738088, 30.08104552, 30.15478370, 30.22859573, 30.30248192, 30.37644258,
	30.45047803, 30.52458858, 30.59877455, 30.67303626, 30.74737401, 30.82178814, 30.89627896, 30.97084678, 31.04549194, 31.12021476,
	31.19501555, 31.26989465, 31.34485237, 31.41988905, 31.49500501, 31.57020059, 31.64547610, 31.72083188, 31.79626827, 31.87178559,
	31.94738418, 32.02306437, 32.09882649, 32.17467089, 32.25059791, 32.32660787, 32.40270112, 32.47887800, 32.55513885, 32.63148402,
	32.70791384, 32.78442866, 32.86102884, 32.93771470, 33.01448661, 33.09134490, 33.16828994, 33.24532206, 33.32244163, 33.39964899,
	33.47694450, 33.55432852, 33.63180139, 33.70936348, 33.78701514, 33.86475674, 33.94258862, 34.02051117, 34.09852472, 34.17662966,
	34.25482634, 34.33311513, 34.41149640, 34.48997051, 34.56853782, 34.64719872, 34.72595358, 34.80480275, 34.88374662, 34.96278556,
	35.04191995, 35.12115015 };




// Function Name:             
// Function Function:         fill vehicle's bottom by its previous adjacent frame
// Input:
//     steering_angle:        steering_angle, unit is degree
//     wheel_turning_angle:   The vehicle's wheel turning angle, unit is degree
//     axis_length:           The vehicle's axis length, unit is mm
//     turning_radius:        The turning radius of the vehicle coordinate system center, unit is mm
// Output:
//     ret:                   the error code of the function
// Note:
//     This function convert steering wheel angle into vehicle wheel angle and vehicle turning radius
// Revision History:
//     Date          Author      Revision Content
//     2018-07-12    YanShuo     Create
static Int32_t convert_steering_wheel_angle(IN const Float64_t steering_angle,
	IN const Float64_t axis_length,
	OUT Float64_t& wheel_turning_angle,
	OUT Float64_t& turning_radius)
{
	Int32_t ret = 0;
	Int32_t curr_steering_angle, next_steering_angle;
	Float64_t curr_turning_angle, next_turning_angle, curr_wt, next_wt;

	curr_steering_angle = (Int32_t)(steering_angle - START_STEERING_WHEEL_ANGLE);
	next_steering_angle = (Int32_t)(steering_angle - START_STEERING_WHEEL_ANGLE) + 1;

	if (curr_steering_angle < 0)
	{
		next_steering_angle = 0;
	}
	if (next_steering_angle > STEERING_WHEEL_ANGLE_NUM)
	{
		next_steering_angle = STEERING_WHEEL_ANGLE_NUM;
	}

	curr_turning_angle = turning_angle_array[curr_steering_angle];
	next_turning_angle = turning_angle_array[next_steering_angle];

	next_wt = steering_angle - (Int32_t)steering_angle;
	curr_wt = 1.0 - next_wt;
	
	wheel_turning_angle = curr_turning_angle * curr_wt + next_turning_angle * next_wt;

	turning_radius =abs( axis_length / tan(wheel_turning_angle * PI / 180));

	return ret;
}

// Function Name:             calc_turning_center
// Function Function:         fill vehicle's bottom by its previous adjacent frame
// Input:
//     turning_radius:        The turning radius of the vehicle coordinate system center, unit is mm
//     steering_angle:        steering_angle, unit is degree
//     turning_center:	      The calculated turning center
//     axis_length:           The vehicle's axis length, unit is mm
// Output:
//     void:                  NULL
// Note:
//     This function calculate vehicle's turning center by turning radius and steering angle information
// Revision History:
//     Date          Author      Revision Content
//     2018-07-12    YanShuo     Create
static void calc_turning_center(IN const Float64_t turning_radius, 
	IN Float64_t steering_angle,
	OUT CvPoint2D64f1& turning_center)
{
	turning_center.y = 0;
	if (steering_angle < 0)
	{
		turning_center.x = -turning_radius;
	}
	else
	{
		turning_center.x = turning_radius;
	}
}

// Function Name:             calc_turned_angle
// Function Function:         calculate the vehicle turned angle between adjacent frame
// Input:
//     turning_center:	      The calculated turning center
//     turning_radius:        The calculated turning radius
//     speed:                 The vehicle's driving speed, unit is m/s
//     time_elapsed:	      The time interval between adjacent frame, unit is ms
//     gear:                  0:between  1:forward  2:reverse  3:neutral  4:parking
//     turned_angle:          The vehicle turned angle need to be calculated, unit is angle
// Output:
//     ret:                   The error code
// Note:
//     This function calculate the angle of current point turned
// Revision History:
//     Date          Author      Revision Content
//     2018-07-12    YanShuo     Create
static Int32_t calc_turned_angle(IN const CvPoint2D64f1 turning_center,
	IN const Float64_t turning_radius,
	IN const Float64_t speed,
	IN const Float64_t time_elapsed,
	IN const Gear_E gear,
	OUT Float64_t& turned_angle)
{
	Int32_t ret = 0;

	Float64_t arc_length;

	arc_length = speed * time_elapsed ;//* 0.001;

	if (turning_center.x < 0 && GEAR_FORWARD == gear // drive towards front left
		|| turning_center.x > 0 && GEAR_REVERSE == gear // drive towards back right
		)
	{
		turned_angle = -arc_length / turning_radius;
	}
	else if (turning_center.x < 0 && GEAR_REVERSE == gear // drive towards back left
		|| turning_center.x > 0 && GEAR_FORWARD == gear // drive towards front right
		)
	{
		turned_angle = arc_length / turning_radius;
	}

	return ret;
}

// Function Name:             calc_prev_vehicle_coord
// Function Function:         calculate current vehicle coord point's location in prev vehicle coordinate system
// Input:
//     curr_vehicle_coord:    The coordinate of the current world coordinate system
//     turning_center:        The solved turning center
//     wheel_turning_angle:	  The wheel turning angle
//     homography_matrix:     The solved homography matrix
//     gear:                  0:between  1:forward  2:reverse  3:neutral  4:parking
//     prev_vehicle_coord:    The coordinate to be solved
// Output:
//     void:                  NULL
// Note:
//     This function calculate current vehicle coord point's corresponding coordinate in prev frame
// Revision History:
//     Date          Author      Revision Content
//     2018-07-12    YanShuo     Create
static Int32_t calc_prev_vehicle_coord(
	IN Float64_t curr_vehicle_coord[3],
	IN CvPoint2D64f1 turning_center,
	IN Float64_t homography_matrix[9],
	IN Gear_E gear,
	OUT Float64_t prev_vehicle_coord[3])
{
	Int32_t ret = 0;

	curr_vehicle_coord[0] -= turning_center.x;
	curr_vehicle_coord[1] -= turning_center.y;

	// is this step cares about gear and direction? need to be taken into consideration
	Mat_Mul_Vec_3by3_DB(prev_vehicle_coord, homography_matrix, curr_vehicle_coord);
	
	prev_vehicle_coord[0] += turning_center.x;
	prev_vehicle_coord[1] += turning_center.y;

	return ret;
}

// Function Name:             uyvy_bilinear_interp
// Function Function:         uyvy image interpolation function
// Input:
//     prev_uyvy_image:		  The previous frame's corresponding uyvy image
//     yuv:                   The interpolated value
//     prev_image_coord:	  The float coordinate to be interpolation
//     result_height:	      The result image's height
//     result_width:          The result image's width
// Output:
//     ret:                   The error code
// Note:
//     This function finish uyvy image interpolation function
// Revision History:
//     Date          Author      Revision Content
//     2018-07-18    YanShuo     Create
Int32_t uyvy_bilinear_interp(IN uchar* prev_uyvy_image, 
	OUT uchar yuv[3], 
	IN Float64_t prev_image_coord[2], 
	IN Int32_t result_height, 
	IN Int32_t result_width)
{
	Int32_t ret = 0;

	Int32_t y, u, v;
	Float64_t wt_upleft, wt_upright, wt_downleft, wt_downright;
	Int32_t x0 = (Int32_t)prev_image_coord[0];
	Int32_t x1 = (Int32_t)prev_image_coord[0] + 1;
	Int32_t y0 = (Int32_t)prev_image_coord[1];
	Int32_t y1 = (Int32_t)prev_image_coord[1] + 1;
	Int32_t channel_id;

	x0 = x0 < 0 ? 0 : x0;
	x1 = x1 < 0 ? 0 : x1;
	y0 = y0 < 0 ? 0 : y0;
	y1 = y1 < 0 ? 0 : y1;

	x0 = x0 > result_width - 1 ? result_width - 1 : x0;
	x1 = x1 > result_width - 1 ? result_width - 1 : x1;
	y0 = y0 > result_height - 1 ? result_height - 1 : y0;
	y1 = y1 > result_height - 1 ? result_height - 1 : y1;

	wt_upleft = (x1 - prev_image_coord[0]) * (y1 - prev_image_coord[1]);
	wt_upright = (prev_image_coord[0] - x0) * (y1 - prev_image_coord[1]);
	wt_downleft = (x1 - prev_image_coord[0]) * (prev_image_coord[1] - y0);
	wt_downright = 1 - wt_upleft - wt_upright - wt_downleft;

	y = prev_uyvy_image[y0 * result_width * 2 + x0 * 2 + 1] * wt_upleft
		+ prev_uyvy_image[y0 * result_width * 2 + x1 * 2 + 1] * wt_upright
		+ prev_uyvy_image[y1 * result_width * 2 + x0 * 2 + 1] * wt_downleft
		+ prev_uyvy_image[y1 * result_width * 2 + x1 * 2 + 1] * wt_downright;
	u = prev_uyvy_image[y0 * result_width * 2 + (x0 >> 1 << 1) * 2];
	v = prev_uyvy_image[y0 * result_width * 2 + (x0 >> 1 << 1) * 2 + 2];

	y = y < 0 ? 0 : (y > 255 ? 255 : y);
	u = u < 0 ? 0 : (u > 255 ? 255 : u);
	v = v < 0 ? 0 : (v > 255 ? 255 : v);

	yuv[0] = (uchar)y;
	yuv[1] = (uchar)u;
	yuv[2] = (uchar)v;

	return ret;
}

Int32_t get_can_info()
{
	Int32_t ret = 0;

	return ret;
}

// Function Name:             fill_vehicle_bottom
// Function Function:         fill vehicle's bottom by its previous adjacent frame
// Input:
//     result_uyvy_image:     The generated result image in uyvy format
//     prev_uyvy_image:		  The result uyvy image of previous frame
//     result_image_width:    The width of result bev image
//     result_image_height:   The height of result bev image
//     car_up_left:           The car's up left corner's coordinate, unit is pixel
//     car_down_right:        The car's down right corner's coordinate, unit is pixel
//     mm_per_pixel_hori:     How many mm per pixel represents in x direction
//     mm_per_pixel_vert:     How many mm per pixel represents in y direction
//     vehicle_length:        The vehicle length, unit is mm
//     vehicle_width:         The vehicle width, unit is mm
//     front_bumper_length:   The front bumper's length, unit is mm
//     axis_length:           The vehicle's axis length, unit is mm
//     rear_bumper_length:    The rear bumper's length, unit is mm
//     steering_wheel_angle:  The steering_wheel_angle, unit is degree, to left is negative, to right is positive 
//     speed:                 The vehicle's driving speed, unit is (km/h)
//     time_elapsed:          The time interval of curr and prev frame, unit is ms
//     gear:                  0:between  1:forward  2:reverse  3:neutral  4:parking
// Output:
//     ret:                   the error code of the function
// Note:
//     This function is the interface function of transparent vehicle bottom algorithm.
// Revision History:
//     Date          Author      Revision Content
//     2018-07-11    YanShuo     Create


//extern CvPoint1 car_up_left,car_down_right;

Int32_t fill_vehicle_bottom(uchar* result_uyvy_image,
	uchar* prev_uyvy_image,
	Int32_t result_image_width,
	Int32_t result_image_height,
	int x1,
	int x2,
	Float64_t mm_per_pixel_hori,
	Float64_t mm_per_pixel_vert,
	Float64_t vehicle_length,
	Float64_t vehicle_width,
	Float64_t front_bumper_length,
	Float64_t axis_length,
	Float64_t rear_bumper_length,	
	Float64_t steering_angle,
	Float64_t speed,
	Float64_t time_elapsed,
	Gear_E gear)
{
	Int32_t ret = 0;
	typedef struct CvPoint2
	{
		int x;
	 	int y;
	 } 
	CvPoint2;
	CvPoint2 car_up_left,car_down_right;

	Float64_t curr_world_coord[3], prev_world_coord[3], curr_vehicle_coord[3], prev_vehicle_coord[3], prev_image_coord[2];
	Float64_t wheel_turning_angle; // unit is arc
	Float64_t turned_angle = 0, turning_radius; // turning radius unit is mm
	uchar yuv[3];
	//CvPoint2D64f1 turning_center = cvPoint2D64f(0, 0); // unit is mm
	CvPoint2D64f1 turning_center;
	turning_center.x= 0;//cvPoint2D64f(0, 0); // unit is mm
	turning_center.y= 0;
	speed = speed * 1000 / 3600; // convert speed unit from km/h into m/s
        car_up_left.x = 404;//460;
	 car_up_left.y = 458;//672;
	 car_down_right.x = 675;//619;
	 car_down_right.y = 1205;//991;
	 	
	if (abs(steering_angle) > 1) // vehicle is running along straight line 
	{
		
		// Step 1: Convert steering_wheel_angle into turning_radius and vehicle_wheel_angle	
		convert_steering_wheel_angle(steering_angle, axis_length, wheel_turning_angle, turning_radius); // maybe turning_radius is infinite value

		calc_turning_center(turning_radius, steering_angle, turning_center);

		// convert wheel_turning_angle unit into arc
		wheel_turning_angle = wheel_turning_angle * PI / 180;

		ret = calc_turned_angle(turning_center, turning_radius, speed, time_elapsed, gear, turned_angle);
	}

	Float64_t homography_matrix[9] = {
		cos(turned_angle), sin(turned_angle), 0,
		-sin(turned_angle), cos(turned_angle), 0,
		0, 0, 1 };

	// Step 2: traverse each location of car region
	for (Int32_t i = car_up_left.y; i <= car_down_right.y; i++)
	{
		for (Int32_t j = car_up_left.x-30; j <= car_down_right.x+30; j++)
		{
	//	Int8_t y1, y2, u, v;

			// convert image coordinate into world coordinate system
			curr_world_coord[0] = (j - result_image_width * 0.5) * mm_per_pixel_hori;
			curr_world_coord[1] = (result_image_height * 0.5 - i) * mm_per_pixel_vert;
			curr_world_coord[2] = 0;

			// convert image coordinate into vehicle coordinate system
			curr_vehicle_coord[0] = curr_world_coord[0];
			curr_vehicle_coord[1] = curr_world_coord[1] + (front_bumper_length + axis_length - vehicle_length * 0.5);
			curr_vehicle_coord[2] = 0;

			if (abs(steering_angle) > 1) // turning 
			{
				ret = calc_prev_vehicle_coord(curr_vehicle_coord, turning_center, homography_matrix, gear, prev_vehicle_coord);
			}
			else // not turning
			{
				prev_vehicle_coord[0] = curr_vehicle_coord[0];
				if (GEAR_FORWARD == gear)
				{
					prev_vehicle_coord[1] = curr_vehicle_coord[1] + speed * time_elapsed;
				}
				else if (GEAR_REVERSE == gear)
				{
					prev_vehicle_coord[1] = curr_vehicle_coord[1] - speed * time_elapsed;
				}
				else
				{
					prev_vehicle_coord[1] = curr_vehicle_coord[1];
				}
				prev_vehicle_coord[2] = curr_vehicle_coord[2];
			}
			
			prev_world_coord[0] = prev_vehicle_coord[0];
			prev_world_coord[1] = prev_vehicle_coord[1] - (front_bumper_length + axis_length - vehicle_length * 0.5);
			prev_world_coord[2] = 0;

			// calculate its coordinate in result image
			prev_image_coord[0] = prev_world_coord[0] / mm_per_pixel_hori + result_image_width * 0.5;
			prev_image_coord[1] = result_image_height * 0.5 - prev_world_coord[1] / mm_per_pixel_vert;

			// pixel bilinear interpolation, need to be done
			ret = uyvy_bilinear_interp(prev_uyvy_image, yuv, prev_image_coord, result_image_height, result_image_width);

			// write the interpolated value into result image
			result_uyvy_image[i * result_image_width * 2 + j * 2 + 1] = yuv[0];
			if (j % 2 == 0)
			{
				result_uyvy_image[i * result_image_width * 2 + j * 2] = yuv[1];
			}
			else
			{
				result_uyvy_image[i * result_image_width * 2 + j * 2] = yuv[2];
			}
			
		}
	}

	return ret;
}


/*
* Function Name:		Mat_Mul_Vec_3by3_DB
* Function Function:	calculate a 3*3 matrix multiply a 3*1 vector
* Input:
*     Cout:				result vector
*     Ain:				the input matrix
*     Bin:				the input vector
* Note:
*     (1) the function is designed for Float64_t type variables
* Revision:
*     (1) Shuo.Yan on 2017/11/22, create log.
*/

#if 0
void Mat_Mul_Vec_3by3_DB(
	OUT Float64_t Cout[3], IN Float64_t Ain[9], IN Float64_t Bin[3])
{
	Cout[0] = Ain[0] * Bin[0] + Ain[1] * Bin[1] + Ain[2] * Bin[2];
	Cout[1] = Ain[3] * Bin[0] + Ain[4] * Bin[1] + Ain[5] * Bin[2];
	Cout[2] = Ain[6] * Bin[0] + Ain[7] * Bin[1] + Ain[8] * Bin[2];
}

#endif
