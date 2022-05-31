/*

adaptation depuis 
https://github.com/G6EJD/ESP32-Morse-Decoder/blob/master/ESP32_Morse_Code_Decoder_02.ino

input 111100111100000001110110000

gcc -Wall -Wextra mo.c -o mo -lm

définir un format de sortie sur USB_CDC-ACM genre:
<SOF>;<MORSE CODE>;<ASCII>;<EOT>;\n
pour avoir une  GUI Windows qui affiche sur 2 lignes : morse et ASCII

*/



#include <stdio.h>
#include<stdint.h>
#include<math.h>
#include <string.h>

#if 0
#define debug_printf(args,...) printf(__VA_ARGS__)
#else
#define debug_printf(args,...)
#endif

static int   realstate           = 0;
static int   realstatebefore     = 0;
static int   filteredstate       = 0;
static int   filteredstatebefore = 0;


// Noise Blanker time which shall be computed so this is initial 
static int nbtime = 6;  /// 6 ms  noise blanker

static long starttimehigh;
static long highduration;
static long lasthighduration;
static long hightimesavg;
static long startttimelow;
static long lowduration;
static long laststarttime = 0;

/** ligne ascii décodée  */
#define D_NUM_CHAR (16)
static char DisplayLine[D_NUM_CHAR+1];

/** raw data filtered in morse code */
#define D_NUM_MORSECODE (10)
static char CodeBuffer[D_NUM_MORSECODE];

static int  stop = 0;
static long  wpm;

/***********************************/
/* input source */
static char *inputDataStream;
static int posInStream = 0;
static int endStream;

int analogRead(int pin) {
    (void) pin;
    char rawIn;
    int devIn;
    if(posInStream >= endStream)
    {
        // end
        return -1;
    }

    /* '0' ou '1' */
    rawIn = inputDataStream[posInStream++];

    /* conversion ascii vers int */
    devIn = (rawIn == '0') ? 0 : 1;

    return devIn;
}

/***********************************/
int base_time=0;

/* shall return time */
int millis(void){
    return base_time++;
}


/***********************************/

/* Add a decoded char to output sequence, avec rotation des caracteres */
static void AddCharacter(char newchar){
  int i;
  for(i = 0; i < D_NUM_CHAR; i++) 
    DisplayLine[i] = DisplayLine[i+1];

  DisplayLine[D_NUM_CHAR] = newchar;
}

/* Convert morse to ascii char */
static void CodeToChar(void) { // translate cw code to ascii character//
  char decode_char = '{'; // default error code

  if (strcmp(CodeBuffer,".-") == 0)      decode_char = 'a';
  if (strcmp(CodeBuffer,"-...") == 0)    decode_char = 'b';
  if (strcmp(CodeBuffer,"-.-.") == 0)    decode_char = 'c';
  if (strcmp(CodeBuffer,"-..") == 0)     decode_char = 'd'; 
  if (strcmp(CodeBuffer,".") == 0)       decode_char = 'e'; 
  if (strcmp(CodeBuffer,"..-.") == 0)    decode_char = 'f'; 
  if (strcmp(CodeBuffer,"--.") == 0)     decode_char = 'g'; 
  if (strcmp(CodeBuffer,"....") == 0)    decode_char = 'h'; 
  if (strcmp(CodeBuffer,"..") == 0)      decode_char = 'i';
  if (strcmp(CodeBuffer,".---") == 0)    decode_char = 'j';
  if (strcmp(CodeBuffer,"-.-") == 0)     decode_char = 'k'; 
  if (strcmp(CodeBuffer,".-..") == 0)    decode_char = 'l'; 
  if (strcmp(CodeBuffer,"--") == 0)      decode_char = 'm'; 
  if (strcmp(CodeBuffer,"-.") == 0)      decode_char = 'n'; 
  if (strcmp(CodeBuffer,"---") == 0)     decode_char = 'o'; 
  if (strcmp(CodeBuffer,".--.") == 0)    decode_char = 'p'; 
  if (strcmp(CodeBuffer,"--.-") == 0)    decode_char = 'q'; 
  if (strcmp(CodeBuffer,".-.") == 0)     decode_char = 'r'; 
  if (strcmp(CodeBuffer,"...") == 0)     decode_char = 's'; 
  if (strcmp(CodeBuffer,"-") == 0)       decode_char = 't'; 
  if (strcmp(CodeBuffer,"..-") == 0)     decode_char = 'u'; 
  if (strcmp(CodeBuffer,"...-") == 0)    decode_char = 'v'; 
  if (strcmp(CodeBuffer,".--") == 0)     decode_char = 'w'; 
  if (strcmp(CodeBuffer,"-..-") == 0)    decode_char = 'x'; 
  if (strcmp(CodeBuffer,"-.--") == 0)    decode_char = 'y'; 
  if (strcmp(CodeBuffer,"--..") == 0)    decode_char = 'z'; 
  
  if (strcmp(CodeBuffer,".----") == 0)   decode_char = '1'; 
  if (strcmp(CodeBuffer,"..---") == 0)   decode_char = '2'; 
  if (strcmp(CodeBuffer,"...--") == 0)   decode_char = '3'; 
  if (strcmp(CodeBuffer,"....-") == 0)   decode_char = '4'; 
  if (strcmp(CodeBuffer,".....") == 0)   decode_char = '5'; 
  if (strcmp(CodeBuffer,"-....") == 0)   decode_char = '6'; 
  if (strcmp(CodeBuffer,"--...") == 0)   decode_char = '7'; 
  if (strcmp(CodeBuffer,"---..") == 0)   decode_char = '8'; 
  if (strcmp(CodeBuffer,"----.") == 0)   decode_char = '9'; 
  if (strcmp(CodeBuffer,"-----") == 0)   decode_char = '0'; 

  if (strcmp(CodeBuffer,"..--..") == 0)  decode_char = '?'; 
  if (strcmp(CodeBuffer,".-.-.-") == 0)  decode_char = '.'; 
  if (strcmp(CodeBuffer,"--..--") == 0)  decode_char = ','; 
  if (strcmp(CodeBuffer,"-.-.--") == 0)  decode_char = '!'; 
  if (strcmp(CodeBuffer,".--.-.") == 0)  decode_char = '@'; 
  if (strcmp(CodeBuffer,"---...") == 0)  decode_char = ':'; 
  if (strcmp(CodeBuffer,"-....-") == 0)  decode_char = '-'; 
  if (strcmp(CodeBuffer,"-..-.") == 0)   decode_char = '/'; 

  if (strcmp(CodeBuffer,"-.--.") == 0)   decode_char = '('; 
  if (strcmp(CodeBuffer,"-.--.-") == 0)  decode_char = ')'; 
  if (strcmp(CodeBuffer,".-...") == 0)   decode_char = '_'; 
  if (strcmp(CodeBuffer,"...-..-") == 0) decode_char = '$'; 
  if (strcmp(CodeBuffer,"...-.-") == 0)  decode_char = '>'; 
  if (strcmp(CodeBuffer,".-.-.") == 0)   decode_char = '<'; 
  if (strcmp(CodeBuffer,"...-.") == 0)   decode_char = '~'; 
  if (strcmp(CodeBuffer,".-.-") == 0)    decode_char = 'a'; // a umlaut
  if (strcmp(CodeBuffer,"---.") == 0)    decode_char = 'o'; // o accent
  if (strcmp(CodeBuffer,".--.-") == 0)   decode_char = 'a'; // a accent
  if (decode_char != '{') {
    // output append
    AddCharacter(decode_char);
  } else 
    {
        debug_printf( "failure decode data: %s\n", CodeBuffer );
    }
}



void setup(void) {
  int i;
  for (i = 0; i <= D_NUM_CHAR; i++)
    DisplayLine[i] = ' ';

  for (i = 0; i <= D_NUM_MORSECODE; i++)
    CodeBuffer[i] = ' ';
}

int loop(void) {
    int refresh = 0;

    // read GPIO input
    realstate = analogRead( 42 );

    // end condition
    if(realstate == -1)  return -1;

    // really verbose
//    debug_printf("raw in: %d ", realstate );


  // Clean up the state with a noise blanker //
  
  if (realstate != realstatebefore) {
    laststarttime = millis();
  }
  if ((millis() - laststarttime) > nbtime) {
    if (realstate != filteredstate) {
      filteredstate = realstate;
    }
  }

  if (filteredstate != filteredstatebefore) {
    if (filteredstate == 1) {
      starttimehigh = millis();
      lowduration = (millis() - startttimelow);
    }

    if (filteredstate == 0) {
      startttimelow = millis();
      highduration = (millis() - starttimehigh);
      if (highduration < (2 * hightimesavg) || hightimesavg == 0) {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3; // now we know avg dit time ( rolling 3 avg)
      }
      if (highduration > (5 * hightimesavg) ) {
        hightimesavg = highduration + hightimesavg;   // if speed decrease fast ..
      }
    }
  }

  // Now check the baud rate based on dit or dah duration either 1, 3 or 7 pauses
  if (filteredstate != filteredstatebefore) {
    printf("trans. %d -> %d\n", filteredstatebefore, filteredstate);
    refresh= 1;
    stop = 0;
    if (filteredstate == 0) { // we did end on a HIGH
      if (highduration < (hightimesavg * 2) && highduration > (hightimesavg * 0.6)) { /// 0.6 filter out false dits
        strcat(CodeBuffer,".");
        debug_printf("IN: .\n");
      }
      if (highduration > (hightimesavg * 2) && highduration < (hightimesavg * 6)) {
        strcat(CodeBuffer,"-");
        debug_printf("IN: -\n");
        wpm = (wpm + (1200 / ((highduration) / 3))) / 2; //// the most precise we can do ;o)
      }
    }

    if (filteredstate == 1) { //// we did end a LOW
      float lacktime = 1;
      printf("filt!\n");
      if (wpm > 25)lacktime = 1.0; ///  when high speeds we have to have a little more pause before new letter or new word
      if (wpm > 30)lacktime = 1.2;
      if (wpm > 35)lacktime = 1.5;
      if (lowduration > (hightimesavg * (2 * lacktime)) && lowduration < hightimesavg * (5 * lacktime)) { // letter space
        printf("check Char 1, reinit!\n");
        CodeToChar();
        CodeBuffer[0] = '\0';
      }

      if (lowduration >= hightimesavg * (5 * lacktime)) { // word space
        printf("****space****\n");
        CodeToChar();
        CodeBuffer[0] = '\0';
        AddCharacter(' ');
        refresh =1;
      }
    }
  }
  if ((millis() - startttimelow) > (highduration * 6) && stop == 0) {
    printf("check Char 3 stop reinit!\n");
    CodeToChar();
    CodeBuffer[0] = '\0';
    stop = 1;
  }

  // the end of main loop clean up//
  realstatebefore     = realstate;
  lasthighduration    = highduration;
  filteredstatebefore = filteredstate;

    // Refresh a chaque loop infos a afficher : 
    if(refresh == 1)
        printf(" WPM = %ld (time: %4d) -%s-\n", wpm, base_time, DisplayLine);

    return 0;
}




int main( int argc, char *argv[] ) {

    int cont;
    if(argc != 2) {
        printf("\n ERROR: enter sequence input test as arg: like 00010111101001\n");
        return -1;
    }

    setup();

    endStream = (int) strlen(argv[1]);
    inputDataStream = argv[1];

    do {
        cont = loop();
    }while( cont != -1);

    return 0;
}
