#use "bl25xx.lib"          Controller library
/************************************************************************
SF_BL25_SPD.C  Used the BL2500 and the SF1016
*************************************************************************/
#define SPI_SER_C
#define SPI_CLK_DIVISOR 20

#define SF1000_CS_PORT			PADR		// CS = PA0  J3:1 pulled high with
													// 4.7K resistor.
#define SF1000_CS_PORTSHADOW	PADRShadow
#define SF1000_CS_BIT			0

#define CS_ENABLE	 BitWrPortI ( SF1000_CS_PORT, &SF1000_CS_PORTSHADOW, 1, SF1000_CS_BIT );\
	SPIxor = 0xFF;		// invert the received bits
#define CS_DISABLE BitWrPortI ( SF1000_CS_PORT, &SF1000_CS_PORTSHADOW, 0, SF1000_CS_BIT );

#use SF1000.lib
union
{	char	cData[1024];
	int	iData[512];
} Block;

union
{	int ival;
	char cval[2];
} Value;

main()
{
	unsigned long Tstart, Telap;
	int block, i, j, k, TestBlock, NbrOfBlocks;
	int BlockCount;
	long FlashAddr, iLong;
	float Tblock;
   int attempts;



   brdInit();              // Initialize the controller

   attempts = 0;

   do
   {
   	i = SF1000Init ();						// initialize the flash
		if ( i )
		{	if ( i == -1 ) printf ( "Invalid configuration value: %d\n", i );
			if ( i == -2 ) printf ( "Unknown response: %d\n", (int)SF1000_String[0] );
			if ( i == -3 ) printf ( "Device not responding: %d\n", (int)SF1000_String[0] );
   		attempts++;
   	}
   }while(i && attempts < 20);

   if(i)
   {
   	printf("failure\n");
      while(1);
   }

	iLong = (long)SF1000_Block_size*(long)SF1000_Nbr_of_Blocks/1000000L;
	printf ( "Density = %d,  Block Size = %d,\n\rNbr of Blocks = %d,  Total Memory = %ldMB\n\r",
			SF1000_Density_Value, SF1000_Block_size, SF1000_Nbr_of_Blocks, iLong );



	printf ( "\n\nShort Test...............\n\r" );

	TestBlock = SF1000_Nbr_of_Blocks/2 + 1;

	iLong = SF1000CheckWrites (TestBlock);

	printf ( "\nBlock %d has been written %ld times\n\rWrite once more",
		TestBlock, iLong );

	FlashAddr = (long)(TestBlock)*SF1000_Block_size;
	while ( SF1000Write ( FlashAddr, (char*)&i, 2 ) == -3 );

	iLong = SF1000CheckWrites (TestBlock);
	printf ( "\nBlock %d has been written %ld times\n\rWrite once more",
		TestBlock, iLong );
	while ( SF1000Write ( FlashAddr, (char*)&i, 2 ) == -3 );
	iLong = SF1000CheckWrites (TestBlock);

	printf ( "\n\rBlock %d has been written %ld times", TestBlock, iLong );

	printf ( "\n\r\nWrite 0xAA to block 0     " );
	memset ( Block.cData, 0xAA, SF1000_Block_size );
	while ( SF1000Write ( 0, Block.cData, SF1000_Block_size ) == -3 );
	iLong = SF1000CheckWrites (0);
	printf ( "\n\rBlock # 0 has been written %ld times", iLong );

	printf ( "\n\rWrite 0x55 to block 1     " );
	memset ( Block.cData, 0x55, SF1000_Block_size );
	while ( SF1000Write ( SF1000_Block_size, Block.cData, SF1000_Block_size ) == -3 );
	iLong = SF1000CheckWrites (1);
	printf ( "\n\rBlock # 1 has been written %ld times", iLong );

	SF1000Read ( 0, Block.cData, SF1000_Block_size );
	printf ( "\n\rVerify block 0" );
	for ( i=0; i<SF1000_Block_size; i++ )
		if ( Block.cData[i] != 0xAA ) printf ( " %d %x\n", i, Block.cData[i] );

	SF1000Read ( SF1000_Block_size, Block.cData, SF1000_Block_size );
	printf ( "\n\rVerify block 1" );
	for ( i=0; i<SF1000_Block_size; i++ )
		if ( Block.cData[i] != 0x55 ) printf ( " %d %x\n", i, Block.cData[i] );


// This section writes and verifys only 2 bytes to each block

	printf ( "\n\nWrite 2 bytes to each block\n\r" );
	Tstart = MS_TIMER;
	for ( block = 0; block < SF1000_Nbr_of_Blocks; block++ )
	{	if ( block%128 == 0 ) printf ( "block %d\r", block );
		FlashAddr = (long)block * SF1000_Block_size;
		Value.ival = block*2;
		while ( SF1000Write ( FlashAddr, Value.cval, 2 ) == -3 );
	}
	Telap = MS_TIMER - Tstart;
	Tblock = (float)Telap/(float)SF1000_Nbr_of_Blocks;
	printf ( "\n\relapsed time = %ldms,  time per block = %fms", Telap, Tblock );

	printf ( "\n\rRead and verify the value from each block\n" );
	Tstart = MS_TIMER;
	for ( block = 0; block < SF1000_Nbr_of_Blocks; block++ )
	{	if ( block%128 == 0 ) printf ( "\rblock %d", block );
		FlashAddr = (long)block * SF1000_Block_size;
		SF1000Read ( FlashAddr, Value.cval, 2 );
		if ( Value.ival != block*2 )
		{	printf ( " Error: written= %d   read= %d\n\r", block, Value.ival );
		}
	}
	Telap = MS_TIMER - Tstart;
	Tblock = (float)Telap/(float)SF1000_Nbr_of_Blocks;
	printf ( "\n\relapsed time = %ldms,  time per block = %fms", Telap, Tblock );
	printf ( "\n\r............Done Testing............\n" );

}