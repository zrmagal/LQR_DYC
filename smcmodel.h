#define V2          inStates[0]
#define U2          inStates[1]
#define YAW_RATE2   inStates[2]
#define ROLL_RATE2  inStates[3]
#define ROLL2       inStates[4]


#define FL (0)
#define FR (1)
#define RL (2)
#define RR (3)


#define LAT_ACCEL2  inDStates[0]
#define LONG_ACCEL2 inDStates[1]
#define YAW_ACCEL2  inDStates[2]
#define ROLL_ACCEL2 inDStates[3]
#define ROLL_DT2    inDStates[4]

#define L_F params[0]
#define L_R params[1]
#define I_Z params[2]
#define MASS params[3]
#define C_1 params[4]
#define M_1 params[5]
#define M_2 params[6]
#define GAMMA params[7]

real_T control_aux = 0;
real_T test = 0;
