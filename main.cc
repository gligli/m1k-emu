//
//	main.cc
//

#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <ctype.h>
#include "mc6850.h"
#include "mc6809.h"

#include "pit82c54.h"

#include <queue>

#ifdef __unix
#include <unistd.h>
#endif

#ifdef __osf__
extern "C" unsigned int alarm(unsigned int);
#endif

//#ifndef sun
//typedef void SIG_FUNC_TYP(int);
//typedef SIG_FUNC_TYPE *SIG_FP;
//#endif

using namespace std;

class sys : virtual public mc6809 {

protected:

	virtual Byte			 read(Word);
	virtual void			 write(Word, Byte);

protected:

	mc6850			 uart;

public:
	Byte presets[0x10000];
	Word presetsBase;
	pit_82C54 timers[5];		
	Byte leds[4];
	Byte swa,swb;
	queue<Byte> aciaQueue;
	Word dacCV[6][10];

	virtual void status(void);

} sys;

Byte sys::read(Word addr)
{
//	if (addr>=0x1400 && addr<0x1600 && addr!=0x1406) printf("R %04x %04x\n", addr, pc);
	
	switch(addr&0xe000)
	{
	case 0x0000: //io
		switch(addr&0x1c00)
		{
		case 0x0000: //T1
			return timers[1].read(addr&3);
		case 0x0400: //T2
			return timers[2].read(addr&3);
		case 0x0800: //T3
			return timers[3].read(addr&3);
		case 0x0C00: //T4
			return timers[4].read(addr&3);
		case 0x1000: //DAC
			return 0;
		case 0x1400: //URDV
			if(addr&0x0200)
				return timers[0].read(addr&3);
			else if((addr&0x7)==6)
				return 0b00000010 | ((aciaQueue.size()>0)?1:0);
			else if((addr&0x7)==7)
			{
				Byte b = aciaQueue.front();
				aciaQueue.pop();
				return b;
			}
			else
				return 0;
			break;
		case 0x1800: //SW
			switch(addr&3)
			{
			case 0:
				return swa;
			case 1:
				return swb;
			default:
				return 0xff;
			}
		case 0x1C00: //moreio
			switch(addr&0x0380)
			{
			case 0x000: // L1
				return 0;
			case 0x080: // L2
				return 0;
			case 0x100: // L3
				return 0;
			case 0x180: // MISC
				return 0;
			case 0x200: // LED1
				return 0;
			case 0x280: // LED2
				return 0;
			case 0x300: // LED3
				return 0;
			case 0x380: // LED4
				return 0;
			}
		}
	case 0x2000: //presets
		return presets[presetsBase|(addr&0x1fff)];
	case 0x4000: //empty
		return 0;
	case 0x6000: //ram
		return mc6809::read((addr&0x1fff)|(presetsBase&0x7fff));
	}
	return mc6809::read(addr);
}

void sys::write(Word addr, Byte x)
{
//	if(addr< 0x1000) printf("W %04x %02x %04x\n",addr,x,pc);
//	if(pc==0x894a || pc==0x894c) printf("W %04x %02x %04x\n",addr,x,pc);

	if(addr>=0x8000) printf("W %04x %02x %04x\n",addr,x,pc); // write rom (debug)
	
	switch(addr&0xe000)
	{
	case 0x0000: //io
		switch(addr&0x1c00)
		{
		case 0x0000: //T1
			timers[1].write(addr&3,x);
			break;
		case 0x0400: //T2
			timers[2].write(addr&3,x);
			break;
		case 0x0800: //T3
			timers[3].write(addr&3,x);
			break;
		case 0x0C00: //T4
			timers[4].write(addr&3,x);
			break;
		case 0x1000: //DAC
//			printf("W DAC  %04x %02x\n",addr,x);
			
			int v,c;
			
			v=(addr>>4)&0x7;
			c=(addr>>1)&0x7;
			
			if(v>=6)
			{
				int nv=((v-6)*8+c)%6;
				int nc=8+((v-6)*8+c)/6;
				
				v=nv;
				c=nc;
			}
			
			if(!(addr&1))
			{
				dacCV[v][c]&=0x001f;
				dacCV[v][c]|=((Word)x&0x7f)<<5;
			}
			else
			{
				dacCV[v][c]&=0xffe0;
				dacCV[v][c]|=((Word)x)>>3;
			}
			break;
		case 0x1400: //URDV
//			printf("W URDV %04x %02x\n",addr,x);
			if(addr&0x0200)
				timers[0].write(addr&3,x);
			else if((addr&0x6)==6)
				;
			break;
		case 0x1800: //SW
			break;
		case 0x1C00: //moreio
			switch(addr&0x0380)
			{
			case 0x000: // L1
				break;
			case 0x080: // L2
				break;
			case 0x100: // L3
				break;
			case 0x180: // MISC
				presetsBase=((Word)x&7)<<13;
//				printf("PB %04x\n",presetsBase);
				break;
			case 0x200: // LED1
				leds[0] = ~x;
				break;
			case 0x280: // LED2
				leds[1] = ~x;
				break;
			case 0x300: // LED3
				leds[2] = ~x;
				break;
			case 0x380: // LED4
				leds[3] = ~x;
				
				break;
			}
			break;
		}
		break;
	case 0x2000: //presets
		halt();
		break;
	case 0x4000: //empty
		halt();
		break;
	case 0x6000: //ram
		mc6809::write((addr&0x1fff)|(presetsBase&0x7fff), x);
	}
}

#ifdef SIGALRM
#ifdef sun
void update(int, ...)
#else
void update(int)
#endif
{
	sys.status();
	(void)signal(SIGALRM, update);
	alarm(1);
}
#endif // SIGALRM

void doACIAByte(Byte b)
{
//	printf("doACIAByte %02x\n",b);
	sys.aciaQueue.push(b);
}

void CALLBACK MidiInProc(HMIDIIN hMidiIn, UINT wMsg, DWORD dwInstance, DWORD dwParam1, DWORD dwParam2)
{
	switch(wMsg) {
	case MIM_OPEN:
		printf("wMsg=MIM_OPEN\n");
		break;
	case MIM_CLOSE:
		printf("wMsg=MIM_CLOSE\n");
		break;
	case MIM_DATA:
		doACIAByte(dwParam1);
		doACIAByte(dwParam1>>8);
		doACIAByte(dwParam1>>16);
		break;
	case MIM_LONGDATA:
	{
		MIDIHDR * midiHdr = (MIDIHDR*)dwParam1;
	
		midiInUnprepareHeader(hMidiIn, midiHdr, sizeof(MIDIHDR));
		
		for(int i=0;i<(int)midiHdr->dwBytesRecorded;++i)
			doACIAByte(midiHdr->lpData[i]);

		midiInPrepareHeader(hMidiIn, midiHdr, sizeof(MIDIHDR));
		midiInAddBuffer(hMidiIn, midiHdr, sizeof(MIDIHDR));
		midiInStart(hMidiIn);
		
		break;
	}
	case MIM_ERROR:
		printf("wMsg=MIM_ERROR\n");
		break;
	case MIM_LONGERROR:
		printf("wMsg=MIM_LONGERROR\n");
		break;
	case MIM_MOREDATA:
		printf("wMsg=MIM_MOREDATA\n");
		break;
	default:
		printf("wMsg = unknown\n");
		break;
	}
	return;
}

void PrintMidiDevices()
{
	UINT nMidiDeviceNum;
	MIDIINCAPS caps;

	nMidiDeviceNum = midiInGetNumDevs();
	if (nMidiDeviceNum == 0) {
		fprintf(stderr, "midiInGetNumDevs() return 0...");
		return;
	}

	printf("\nMidi device numbers :\n");
	for (unsigned int i = 0; i < nMidiDeviceNum; ++i) {
		midiInGetDevCaps(i, &caps, sizeof(MIDIINCAPS));
		printf("\t%d : %s\n", i, caps.szPname);
	}
}

#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 16
#endif
 
void hexdump(void *mem, unsigned int len)
{
        unsigned int i, j;
        
        for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++)
        {
                /* print offset */
                if(i % HEXDUMP_COLS == 0)
                {
                        printf("0x%06x: ", i);
                }
 
                /* print hex data */
                if(i < len)
                {
                        printf("%02x ", 0xFF & ((char*)mem)[i]);
                }
                else /* end of block, just aligning for ASCII dump */
                {
                        printf("   ");
                }
                
                /* print ASCII dump */
                if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1))
                {
                        for(j = i - (HEXDUMP_COLS - 1); j <= i; j++)
                        {
                                if(j >= len) /* end of block, not really printing */
                                {
                                        putchar(' ');
                                }
                                else if(isprint(((char*)mem)[j])) /* printable char */
                                {
                                        putchar(0xFF & ((char*)mem)[j]);        
                                }
                                else /* other char */
                                {
                                        putchar('.');
                                }
                        }
                        putchar('\n');
                }
        }
}

int main(int argc, char *argv[])
{
	printf("-= Matrix 1000 emulator =-\n");

	printf("\nKeyboard mapping :\n");
	printf("\tKeypad 0-9,+,-,Enter : Same on Matrix 1000\n");
	printf("\tKeypad * : Select\n");
	printf("\tKeypad / : Bank lock\n");
	printf("\tT : Show timer infos\n");
	printf("\tD : Toggle DCOs infos\n");
	printf("\tC : Toggle CVs infos\n");
	printf("\tL : Toggle LEDs infos\n");
	printf("\tR : Dump RAM\n");
	
	PrintMidiDevices();
	
	if (argc != 4) {
		fprintf(stderr, "\nUsage : %s <firmware> <presets> <midi device #>\n", argv[0]);
		return EXIT_FAILURE;
	}

	(void)signal(SIGINT, SIG_IGN);
#ifdef SIGALRM
	(void)signal(SIGALRM, update);
	alarm(1);
#endif

	FILE		*fp;
	HMIDIIN hMidiDevice = NULL;;
	DWORD nMidiPort = atoi(argv[3]);
	MMRESULT rv;
	MIDIHDR midiHdr;

	rv = midiInOpen(&hMidiDevice, nMidiPort, (DWORD)(void*)MidiInProc, 0, CALLBACK_FUNCTION);
	if (rv != MMSYSERR_NOERROR) {
		fprintf(stderr, "midiInOpen() failed...rv=%d", rv);
		return -1;
	}

	fp = fopen(argv[1], "rb");
	if (!fp) {
		perror("filename f");
		exit(EXIT_FAILURE);
	}

	if (fread(&sys.getMemory()[0x8000],0x8000,1,fp)!=1) {
		perror("fread f");
		exit(EXIT_FAILURE);
	}

	fclose(fp);

	fp = fopen(argv[2], "rb");
	if (!fp) {
		perror("filename p");
		exit(EXIT_FAILURE);
	}

	if (fread(sys.presets,0x10000,1,fp)!=1) {
		perror("fread p");
		exit(EXIT_FAILURE);
	}

	fclose(fp);

	sys.swa = 0;
	sys.swb = 4; // ENTER pressed, to do a reinit
	
	sys.reset();
	
	for(int i=0;i<5;++i)
		for(int j=0;j<3;++j)
			sys.timers[i].set_GATE(j,true);


	midiHdr.lpData = (LPTSTR)malloc(65536);
	midiHdr.dwBufferLength = 65536;
	midiHdr.dwFlags = 0;
	midiInPrepareHeader(hMidiDevice, &midiHdr, sizeof(MIDIHDR));
	midiInAddBuffer(hMidiDevice, &midiHdr, sizeof(MIDIHDR));
	midiInStart(hMidiDevice);
	
	sys.run();

	midiInStop(hMidiDevice);
	midiInClose(hMidiDevice);

	printf("[END]");
	for(;;);
	
	return EXIT_SUCCESS;
}

#define BYTETOBINARYPATTERN "%d%d%d%d%d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0) 

#define SS_A 16
#define SS_B 1
#define SS_C 64
#define SS_D 32
#define SS_E 2
#define SS_F 128
#define SS_G 8
#define SS_DP 4

#define QUANTUM 100

void sys::status(void)
{
	if(cycle%200000==0)
	{
		static bool dLED=true;
		static bool dDCO=false;
		static bool dCV=false;
		
		swa=swb=0;
		if(GetAsyncKeyState(VK_NUMPAD0)&0x8000) swa|=0x01;
		if(GetAsyncKeyState(VK_NUMPAD1)&0x8000) swa|=0x02;
		if(GetAsyncKeyState(VK_NUMPAD2)&0x8000) swa|=0x04;
		if(GetAsyncKeyState(VK_NUMPAD3)&0x8000) swa|=0x08;
		if(GetAsyncKeyState(VK_NUMPAD4)&0x8000) swa|=0x10;
		if(GetAsyncKeyState(VK_NUMPAD5)&0x8000) swa|=0x20;
		if(GetAsyncKeyState(VK_NUMPAD6)&0x8000) swa|=0x40;
		if(GetAsyncKeyState(VK_NUMPAD7)&0x8000) swa|=0x80;

		if(GetAsyncKeyState(VK_NUMPAD8)&0x8000) swb|=0x01;
		if(GetAsyncKeyState(VK_NUMPAD9)&0x8000) swb|=0x02;
		if(GetAsyncKeyState(VK_RETURN)&0x8000) swb|=0x04;
		if(GetAsyncKeyState(VK_SUBTRACT)&0x8000) swb|=0x08;
		if(GetAsyncKeyState(VK_ADD)&0x8000) swb|=0x10;
		if(GetAsyncKeyState(VK_DIVIDE)&0x8000) swb|=0x20;
		if(GetAsyncKeyState(VK_MULTIPLY)&0x8000) swb|=0x40;
		
		
		if(GetAsyncKeyState('T')&0x8000)
			for(int i=0;i<5;++i)
				for(int j=0;j<3;++j)
					timers[i].print_cnum(j);

		if(GetAsyncKeyState('D')&0x8000)
		{
			dDCO=!dDCO;
			Sleep(200);
		}

		if(GetAsyncKeyState('L')&0x8000)
		{
			dLED=!dLED;
			Sleep(200);
		}

		if(GetAsyncKeyState('C')&0x8000)
		{
			dCV=!dCV;
			Sleep(200);
		}

		if(GetAsyncKeyState('R')&0x8000)
		{
			hexdump(&memory[0x0000],0x2000);
			Sleep(200);
		}

		if(dLED)
		{
			printf("PC %04x %d %d\n",pc,cc.bit.i,cc.bit.f);
			printf("LEDS %02x "BYTETOBINARYPATTERN"\n",leds[3],BYTETOBINARY(leds[3]));
			printf(" %c   %c   %c\n",leds[0]&SS_A?'-':' ',leds[1]&SS_A?'-':' ',leds[2]&SS_A?'-':' ');
			printf("%c %c %c %c %c %c\n",leds[0]&SS_F?'|':' ',leds[0]&SS_B?'|':' ',leds[1]&SS_F?'|':' ',leds[1]&SS_B?'|':' ',leds[2]&SS_F?'|':' ',leds[2]&SS_B?'|':' ');
			printf("%c %c %c %c %c %c\n",leds[0]&SS_F?'|':' ',leds[0]&SS_B?'|':' ',leds[1]&SS_F?'|':' ',leds[1]&SS_B?'|':' ',leds[2]&SS_F?'|':' ',leds[2]&SS_B?'|':' ');
			printf(" %c   %c   %c\n",leds[0]&SS_G?'-':' ',leds[1]&SS_G?'-':' ',leds[2]&SS_G?'-':' ');
			printf("%c %c %c %c %c %c\n",leds[0]&SS_E?'|':' ',leds[0]&SS_C?'|':' ',leds[1]&SS_E?'|':' ',leds[1]&SS_C?'|':' ',leds[2]&SS_E?'|':' ',leds[2]&SS_C?'|':' ');
			printf("%c %c %c %c %c %c\n",leds[0]&SS_E?'|':' ',leds[0]&SS_C?'|':' ',leds[1]&SS_E?'|':' ',leds[1]&SS_C?'|':' ',leds[2]&SS_E?'|':' ',leds[2]&SS_C?'|':' ');
			printf(" %c %c %c %c %c %c\n",leds[0]&SS_D?'-':' ',leds[0]&SS_DP?'.':' ',leds[1]&SS_D?'-':' ',leds[1]&SS_DP?'.':' ',leds[2]&SS_D?'-':' ',leds[2]&SS_DP?'.':' ');
		}
		
		if(dDCO)
		{
			for(int i=1;i<3;++i)
				for(int j=0;j<3;++j)
					printf("%dA %5.2fHz ",j+3*(i-1)+1,2000000.0/timers[i].counter[j].inlatch);
			printf("\n");
			for(int i=3;i<5;++i)
				for(int j=0;j<3;++j)
					printf("%dB %5.2fHz ",j+3*(i-3)+1,2000000.0/timers[i].counter[j].inlatch);
			printf("\n");
		}
		
		if(dCV)
		{
			for(int i=0;i<6;++i)
			{
				printf("%d  ",i+1);
				for(int j=0;j<10;++j)
					printf("%5d  ",dacCV[i][j]);
				printf("\n");
			}
		}
		
		Sleep(50);
	}
	
	if(cycle%QUANTUM==0)
	{
		timers[0].clock_multiple(1,QUANTUM);
		timers[0].clock_multiple(2,QUANTUM);
		for(int i=1;i<5;++i)
			timers[i].clock_all(QUANTUM);

		if(!timers[0].read_OUT(2))
			do_irq();

		static bool ppulse=false;
		bool pulse=(cycle>>12)!=0;

		timers[0].set_GATE(1,pulse);
		if(ppulse!=pulse)
			timers[0].clock_multiple(0,1);
		ppulse=pulse;
		
		if(aciaQueue.size()>0)
			do_firq();
	}
}
