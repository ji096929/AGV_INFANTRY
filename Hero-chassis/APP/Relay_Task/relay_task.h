#define	Supercap_Power_Supply	1
#define	Direct_Power_Supply		2	//从电管直接取电，不通过电容

extern int Relay_Set_State;

void Relay_Task(int S_Cnt, int MS_Cnt);
	