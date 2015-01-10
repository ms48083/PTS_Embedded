/***************************************************************************

	 Samples\Random.c

	 Z-World, Inc, 2000

    This program generates pseudo-random integers between 2000 and 2999
    The library rand() function returns a floating-point value (not POSIX).

***************************************************************************/
#class auto
#define STDIO_ENABLE_LONG_STRINGS

int UID_Encode(char *UID, char *Result);
int UID_Decode(char *UID, char *Result);
void show_me(char *UID);
void main ()
{
	int i;
   char id[14];
   char Res[16];

   // demo some numbers
   show_me("154224091045");
   show_me("155224091045");
   show_me("156224091045");
   show_me("157224091045");
   show_me("150224091045");
   show_me("151000001045");
   show_me("150024001005");
   show_me("154000000000");
   show_me("154999999999");
   show_me("154000000000");
   show_me("154618822656");
   show_me("154618822655");

}
void show_me(char *id)
{
	int i;
   char Res[14];
   char Reid[14];

	i = UID_Encode(id, Res);
   printf("%d %s = %02X%02X%02X%02X%02X\n", i, id, Res[0], Res[1], Res[2], Res[3], Res[4]);
	i = UID_Decode(Res, Reid);
   printf("%d %02X%02X%02X%02X%02X = %s\n\n", i, Res[0], Res[1], Res[2], Res[3], Res[4], Reid);
   return;
}

int UID_Encode(char *UID, char *Result)
{  // convert 12 byte string into 5 byte decimal
   // Result must be sized to handle 5 bytes
   // Return -1 if invalid UID or 0 if success
	int i;
   int hval;
   int carry;
   unsigned long lval;
   unsigned long llast;
   unsigned long incr;
   char hdig[4];  // high 3 digits
   char ldig[10]; // low 9 digits

   // assuming length of 12
   // split at 3/9
   strncpy(hdig, UID, 3); hdig[3]=0;
   strcpy(ldig, &UID[3]);

   lval = atol(ldig);
   hval = atoi(hdig);

   incr = 1000000000;
   llast = lval;
   carry=0;
	for (i=0; i<hval; i++)
	{
   	lval = lval + incr;
      if (lval<llast) carry++;
      llast = lval;
	}
   Result[0] = (char)carry;

	Result[1] = (char)((lval >> 24) & 0xFF) ;
	Result[2] = (char)((lval >> 16) & 0xFF) ;
	Result[3] = (char)((lval >> 8) & 0XFF);
	Result[4] = (char)((lval & 0XFF));

   //memcpy(&Result[1], &lval, 4);
   return 0;
}
int UID_Decode(char *UID, char *Result)
{  // convert 5 byte UID into 12 byte string
   // Result must be sized to handle 12+1 bytes
   // Return -1 if invalid UID or 0 if success
   int i;
   unsigned long lval;
   unsigned long ba;
   unsigned long bm;
   unsigned long lo;
   int hi;
   int hval;

   //memcpy(&c, &UID[1], 4);
   lval = (unsigned long)UID[1] << 24;
   lval = lval | (unsigned long)UID[2] << 16;
   lval = lval | (unsigned long)UID[3] << 8;
   lval = lval | (unsigned long)UID[4];
   hval = (int)(UID[0]<<4) + (int)((lval & 0xF0000000)>>28);  // how many times to loop
   lo = lval & 0x0FFFFFFF; // highest byte in hi
   hi=0;
   ba = 0x10000000; // add this
   bm = 1000000000; // mod this
   for (i=0; i<hval; i++)
   {
      lo = lo + ba;   // add
      hi += (int)(lo / bm);  // mod
      lo = lo % bm;   // div
   }
   // now put into string
   snprintf(Result, 13, "%d%09lu", hi, lo);
   Result[12]=0;

   return 0;
}