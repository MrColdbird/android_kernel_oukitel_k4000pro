#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900	
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900	
#endif

#define RBAT_PULL_UP_VOLT          1800



// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	  {0   , 4308},         
		{1   , 4282},         
		{3   , 4262},         
		{4   , 4244},         
		{6   , 4228},         
		{7   , 4213},         
		{8   , 4199},         
		{10  , 4185},         
		{11  , 4172},         
		{12  , 4158},         
		{14  , 4144},         
		{15  , 4130},         
		{17  , 4116},         
		{18  , 4104},         
		{19  , 4095},         
		{21  , 4089},         
		{22  , 4080},         
		{23  , 4058},         
		{25  , 4029},         
		{26  , 4005},         
		{28  , 3989},         
		{29  , 3977},         
		{30  , 3968},         
		{32  , 3961},         
		{33  , 3955},         
		{35  , 3948},         
		{36  , 3941},         
		{37  , 3930},         
		{39  , 3919},         
		{40  , 3907},         
		{41  , 3895},         
		{43  , 3883},         
		{44  , 3872},         
		{46  , 3862},         
		{47  , 3854},         
		{48  , 3846},         
		{50  , 3839},         
		{51  , 3833},         
		{52  , 3827},         
		{54  , 3822},         
		{55  , 3817},         
		{57  , 3813},         
		{58  , 3808},         
		{59  , 3804},         
		{61  , 3800},         
		{62  , 3797},         
		{64  , 3794},         
		{65  , 3791},         
		{66  , 3789},         
		{68  , 3788},         
		{69  , 3786},         
		{70  , 3785},         
		{72  , 3784},         
		{73  , 3783},    
		{75  , 3781},
		{76  , 3779},
		{77  , 3776},       
		{79  , 3771},
		{80  , 3766},       
    {81  , 3758},
    {83  , 3748}, 
    {84  , 3739},
    {86  , 3728},
    {87  , 3717},
    {88  , 3710},
    {90  , 3707},
    {91  , 3704},
    {93  , 3701},
    {94  , 3698},
    {95  , 3690},
    {97  , 3659},
    {98  , 3586},
    {99  , 3479},
    {100 , 3398},
    {100 , 3398},
    {100 , 3398},
    {100 , 3398},
    {100 , 3398},  
    {100 , 3398},  
    {100 , 3398},  
    {100 , 3398},   
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0   , 4308},         
	{1   , 4282},         
	{3   , 4262},         
	{4   , 4244},         
	{6   , 4228},         
	{7   , 4213},         
	{8   , 4199},         
	{10  , 4185},         
	{11  , 4172},         
	{12  , 4158},         
	{14  , 4144},         
	{15  , 4130},         
	{17  , 4116},         
	{18  , 4104},         
	{19  , 4095},         
	{21  , 4089},         
	{22  , 4080},         
	{23  , 4058},         
	{25  , 4029},         
	{26  , 4005},         
	{28  , 3989},         
	{29  , 3977},         
	{30  , 3968},         
	{32  , 3961},         
	{33  , 3955},         
	{35  , 3948},         
	{36  , 3941},         
	{37  , 3930},         
	{39  , 3919},         
	{40  , 3907},         
	{41  , 3895},         
	{43  , 3883},         
	{44  , 3872},         
	{46  , 3862},         
	{47  , 3854},         
	{48  , 3846},         
	{50  , 3839},         
	{51  , 3833},         
	{52  , 3827},         
	{54  , 3822},         
	{55  , 3817},         
	{57  , 3813},         
	{58  , 3808},         
	{59  , 3804},         
	{61  , 3800},         
	{62  , 3797},         
	{64  , 3794},         
	{65  , 3791},         
	{66  , 3789},         
	{68  , 3788},         
	{69  , 3786},         
	{70  , 3785},         
	{72  , 3784},         
	{73  , 3783},    
	{75  , 3781},
	{76  , 3779},
	{77  , 3776},       
	{79  , 3771},
	{80  , 3766},       
  {81  , 3758},
  {83  , 3748},   
  {84  , 3739},
  {86  , 3728},
  {87  , 3717},
  {88  , 3710},
  {90  , 3707},
  {91  , 3704},
  {93  , 3701},
  {94  , 3698},
  {95  , 3690},
  {97  , 3659},
  {98  , 3586},
  {99  , 3479},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
  {100 , 3398},
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0   , 4329},         
	{1   , 4311},         
	{3   , 4292},         
	{4   , 4276},         
	{5   , 4259},         
	{7   , 4244},         
	{8   , 4229},         
	{9   , 4214},         
	{11  , 4200},         
	{12  , 4185},         
	{13  , 4171},         
	{15  , 4157},         
	{16  , 4143},         
	{17  , 4129},         
	{19  , 4116},         
	{20  , 4102},         
	{22  , 4089},         
	{23  , 4080},         
	{24  , 4074},         
	{26  , 4062},         
	{27  , 4040},         
	{28  , 4017},         
	{30  , 4000},         
	{31  , 3989},         
	{32  , 3981},         
	{34  , 3976},         
	{35  , 3970},         
	{36  , 3962},         
	{38  , 3952},         
	{39  , 3943},         
	{40  , 3934},         
	{42  , 3923},         
	{43  , 3911},         
	{44  , 3896},         
	{46  , 3882},         
	{47  , 3870},         
	{48  , 3860},         
	{50  , 3851},         
	{51  , 3844},         
	{52  , 3837},         
	{54  , 3831},         
	{55  , 3826},         
	{57  , 3821},         
	{58  , 3815},         
	{59  , 3811},         
	{61  , 3807},         
	{62  , 3802},         
	{63  , 3799},         
	{65  , 3795},         
	{66  , 3792},         
	{67  , 3789},         
	{69  , 3786},         
	{70  , 3783},         
	{71  , 3780},    
	{73  , 3778},
	{74  , 3776},
	{75  , 3773},       
	{77  , 3769},
	{78  , 3764},       
  {79  , 3758},  
  {81  , 3752},   
	{82  , 3747},
	{83  , 3739},
	{85  , 3730},
	{86  , 3720},
	{87  , 3707},
	{89  , 3699},
	{90  , 3697},
	{92  , 3696},
	{93  , 3695},
	{94  , 3693},
	{96  , 3687},
	{97  , 3651},
	{98  , 3579},
	{100 , 3469},
	{100 , 3469},
	{100 , 3469},
	{100 , 3469},
	{100 , 3469},
	{100 , 3469},
	{100 , 3469},
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0    , 4335},        
	{1    , 4319},        
	{3    , 4301},        
	{4    , 4284},        
	{5    , 4268},        
	{7    , 4253},        
	{8    , 4237},        
	{9    , 4222},        
	{11   , 4207},        
	{12   , 4192},        
	{13   , 4178},        
	{15   , 4163},        
	{16   , 4149},        
	{17   , 4135},        
	{19   , 4121},        
	{20   , 4108},        
	{22   , 4094},        
	{23   , 4081},        
	{24   , 4068},        
	{26   , 4056},        
	{27   , 4045},        
	{28   , 4031},        
	{30   , 4018},        
	{31   , 4006},        
	{32   , 3997},        
	{34   , 3986},        
	{35   , 3976},        
	{36   , 3966},        
	{38   , 3956},        
	{39   , 3946},        
	{40   , 3937},        
	{42   , 3927},        
	{43   , 3916},        
	{44   , 3904},        
	{46   , 3890},        
	{47   , 3875},        
	{48   , 3864},        
	{50   , 3855},        
	{51   , 3848},        
	{52   , 3841},        
	{54   , 3835},        
	{55   , 3829},        
	{57   , 3823},        
	{58   , 3818},        
	{59   , 3813},        
	{61   , 3808},        
	{62   , 3804},        
	{63   , 3800},        
	{65   , 3796},        
	{66   , 3792},        
	{67   , 3789},        
	{69   , 3786},        
	{70   , 3782},        
	{71   , 3778},   
	{73   , 3771},
	{74   , 3762},
	{75   , 3758},      
	{77   , 3753},
	{78   , 3748},      
  {79   , 3742},
  {81   , 3737},  
  {82   , 3732},  
  {84   , 3725},  
  {85   , 3715},  
  {86   , 3705},  
  {88   , 3693},  
  {89   , 3684},  
  {90   , 3682},  
  {92   , 3682},  
  {93   , 3681},  
  {94   , 3679},  
  {96   , 3671},  
  {97   , 3631},  
  {98   , 3561},  
  {100  , 3453},  
  {100  , 3453},  
  {100  , 3453},  
  {100  , 3453},  
  {100  , 3453},  
  {100  , 3453},  
  {100  , 3453},  
};           

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{321   , 4308},         
	{321   , 4282},         
	{327   , 4262},         
	{333   , 4244},         
	{336   , 4228},         
	{340   , 4213},         
	{345   , 4199},         
	{349   , 4185},         
	{354   , 4172},         
	{358   , 4158},         
	{363   , 4144},         
	{367   , 4130},         
	{371   , 4116},         
	{370   , 4104},         
	{377   , 4095},         
	{398   , 4089},         
	{411   , 4080},         
	{404   , 4058},         
	{391   , 4029},         
	{387   , 4005},         
	{388   , 3989},         
	{388   , 3977},         
	{389   , 3968},         
	{393   , 3961},         
	{397   , 3955},         
	{400   , 3948},         
	{399   , 3941},         
	{393   , 3930},         
	{385   , 3919},         
	{377   , 3907},         
	{369   , 3895},         
	{360   , 3883},         
	{354   , 3872},         
	{350   , 3862},         
	{347   , 3854},         
	{346   , 3846},         
	{347   , 3839},         
	{349   , 3833},         
	{351   , 3827},         
	{354   , 3822},         
	{356   , 3817},         
	{360   , 3813},         
	{362   , 3808},         
	{366   , 3804},         
	{370   , 3800},         
	{374   , 3797},         
	{375   , 3794},         
	{379   , 3791},         
	{381   , 3789},         
	{386   , 3788},         
	{388   , 3786},         
	{393   , 3785},         
	{398   , 3784},         
	{403   , 3783},    
	{408   , 3781},
	{412   , 3779},
	{419   , 3776},       
	{422   , 3771},
	{428   , 3766},       
  {433   , 3758}, 
  {438   , 3748}, 
  {448   , 3739}, 
  {459   , 3728}, 
  {468   , 3717}, 
  {483   , 3710}, 
  {505   , 3707}, 
  {544   , 3704}, 
  {598   , 3701},     
  {674   , 3698},     
  {770   , 3690},     
  {862   , 3659},     
  {984   , 3586},     
  {1198  , 3479},     
  {995   , 3398},     
  {995   , 3398},     
  {995   , 3398},     
  {995   , 3398},     
  {995   , 3398},     
  {995   , 3398}, 
  {995   , 3398}, 
  {995   , 3398}, 
};      

// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{321   , 4308},       
	{321   , 4282},       
	{327   , 4262},       
	{333   , 4244},       
	{336   , 4228},       
	{340   , 4213},       
	{345   , 4199},       
	{349   , 4185},       
	{354   , 4172},       
	{358   , 4158},       
	{363   , 4144},       
	{367   , 4130},       
	{371   , 4116},       
	{370   , 4104},       
	{377   , 4095},       
	{398   , 4089},       
	{411   , 4080},       
	{404   , 4058},       
	{391   , 4029},       
	{387   , 4005},       
	{388   , 3989},       
	{388   , 3977},       
	{389   , 3968},       
	{393   , 3961},       
	{397   , 3955},       
	{400   , 3948},       
	{399   , 3941},       
	{393   , 3930},       
	{385   , 3919},       
	{377   , 3907},       
	{369   , 3895},       
	{360   , 3883},       
	{354   , 3872},       
	{350   , 3862},       
	{347   , 3854},       
	{346   , 3846},       
	{347   , 3839},       
	{349   , 3833},       
	{351   , 3827},       
	{354   , 3822},       
	{356   , 3817},       
	{360   , 3813},       
	{362   , 3808},       
	{366   , 3804},       
	{370   , 3800},       
	{374   , 3797},       
	{375   , 3794},       
	{379   , 3791},       
	{381   , 3789},       
	{386   , 3788},       
	{388   , 3786},       
	{393   , 3785},       
	{398   , 3784},       
	{403   , 3783},  
	{408   , 3781},
	{412   , 3779},
	{419   , 3776},     
	{422   , 3771},
	{428   , 3766},
	{433   , 3758},
	{438   , 3748},
	{448   , 3739},
	{459   , 3728},
	{468   , 3717},
	{483   , 3710},
	{505   , 3707},
	{544   , 3704},
	{598   , 3701},
	{674   , 3698},
	{770   , 3690},
	{862   , 3659},
	{984   , 3586},
	{1198  , 3479},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
	{995   , 3398},
};     

// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{177   , 4329},         
	{177   , 4311},         
	{178   , 4292},         
	{178   , 4276},         
	{177   , 4259},         
	{179   , 4244},         
	{180   , 4229},         
	{181   , 4214},         
	{183   , 4200},         
	{185   , 4185},         
	{186   , 4171},         
	{188   , 4157},         
	{191   , 4143},         
	{193   , 4129},         
	{196   , 4116},         
	{198   , 4102},         
	{200   , 4089},         
	{204   , 4080},         
	{214   , 4074},         
	{214   , 4062},         
	{209   , 4040},         
	{208   , 4017},         
	{211   , 4000},         
	{214   , 3989},         
	{217   , 3981},         
	{223   , 3976},         
	{227   , 3970},         
	{228   , 3962},         
	{229   , 3952},         
	{230   , 3943},         
	{231   , 3934},         
	{229   , 3923},         
	{223   , 3911},         
	{211   , 3896},         
	{198   , 3882},         
	{189   , 3870},         
	{184   , 3860},         
	{181   , 3851},         
	{179   , 3844},         
	{179   , 3837},         
	{180   , 3831},         
	{181   , 3826},         
	{182   , 3821},         
	{182   , 3815},         
	{184   , 3811},         
	{186   , 3807},         
	{186   , 3802},         
	{188   , 3799},         
	{190   , 3795},         
	{191   , 3792},         
	{192   , 3789},         
	{192   , 3786},         
	{193   , 3783},         
	{194   , 3780},    
	{194   , 3778},
	{194   , 3776},
	{195   , 3773},       
	{192   , 3769},
	{187   , 3764},       
  {185   , 3758}, 
  {187   , 3752}, 
  {191   , 3747}, 
  {192   , 3739}, 
  {188   , 3730}, 
  {192   , 3720}, 
  {187   , 3707}, 
  {181   , 3699}, 
  {183   , 3697}, 
  {192   , 3696}, 
  {202   , 3695}, 
  {212   , 3693}, 
  {219   , 3687}, 
  {204   , 3651}, 
  {212   , 3579}, 
  {230   , 3469}, 
  {230   , 3469}, 
  {230   , 3469}, 
  {230   , 3469}, 
  {230   , 3469}, 
  {230   , 3469}, 
  {230   , 3469}, 
}; 

// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{147 , 4335},         
	{147 , 4319},         
	{148 , 4301},         
	{149 , 4284},         
	{150 , 4268},         
	{149 , 4253},         
	{151 , 4237},         
	{150 , 4222},         
	{152 , 4207},         
	{153 , 4192},         
	{154 , 4178},         
	{153 , 4163},         
	{155 , 4149},         
	{157 , 4135},         
	{157 , 4121},         
	{160 , 4108},         
	{161 , 4094},         
	{162 , 4081},         
	{163 , 4068},         
	{166 , 4056},         
	{171 , 4045},         
	{169 , 4031},         
	{171 , 4018},         
	{174 , 4006},         
	{178 , 3997},         
	{179 , 3986},         
	{181 , 3976},         
	{184 , 3966},         
	{185 , 3956},         
	{188 , 3946},         
	{191 , 3937},         
	{193 , 3927},         
	{192 , 3916},         
	{186 , 3904},         
	{174 , 3890},         
	{163 , 3875},         
	{158 , 3864},         
	{156 , 3855},         
	{155 , 3848},         
	{156 , 3841},         
	{155 , 3835},         
	{156 , 3829},         
	{157 , 3823},         
	{160 , 3818},         
	{161 , 3813},         
	{163 , 3808},         
	{165 , 3804},         
	{168 , 3800},         
	{169 , 3796},         
	{171 , 3792},         
	{173 , 3789},         
	{173 , 3786},         
	{174 , 3782},         
	{172 , 3778},    
	{164 , 3771},
	{156 , 3762},
	{160 , 3758},       
	{160 , 3753},
	{159 , 3748},       
  {158 , 3742}, 
  {158 , 3737},        
  {161 , 3732}, 
  {161 , 3725}, 
  {158 , 3715}, 
  {160 , 3705}, 
  {157 , 3693}, 
  {150 , 3684}, 
  {154 , 3682}, 
  {161 , 3682}, 
  {167 , 3681}, 
  {171 , 3679}, 
  {171 , 3671}, 
  {165 , 3631}, 
  {172 , 3561}, 
  {184 , 3453}, 
  {184 , 3453}, 
  {184 , 3453}, 
  {184 , 3453}, 
  {184 , 3453}, 
  {184 , 3453}, 
  {184 , 3453}, 
}; 

// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
  {0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 }, 
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },  
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 },
	{0  , 0 }
};    

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H
