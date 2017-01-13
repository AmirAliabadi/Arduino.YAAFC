# YAAFC
YAAFC - Yet Another Arduino Flight Controller

## Arduino Notes:
### PIN outs:
Arduino Digital 
Pin   Port Pin
0     PD0
1     PD1
2     PD2
3     PD3
4     PD4
5     PD5
6     PD6
7     PD7

8     PB0
9     PB1
10    PB2
11    PB3
12    PB4
13    PB5  

Examples:
DDRD |= B00001000;	//Configure digital poort 3 as output
DDRD |= B00100000;	//Configure digital poort 5 as output
DDRD |= B01000000;	//Configure digital poort 6 as output
DDRB |= B00001110;	//Configure digital poort 9, 10, 11 as output.

PORTD |= B00001000; // Set digital port 3 high
PORTD |= B01000000; // Set digital port 6 high
PORTB |= B00001110; // Set digital port 9,10,11 high

